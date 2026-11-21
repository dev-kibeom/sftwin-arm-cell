"""Durable Conveyor observations and inspection history; never a PLC or Robot authority."""
from contextlib import contextmanager
from pathlib import Path
from threading import RLock
from datetime import datetime, timezone
import json
import sqlite3


def timestamp():
    return datetime.now(timezone.utc).isoformat()


class ProductionLedger:
    """One serialized local writer; WAL permits concurrent reporting readers."""

    def __init__(self, filename, stream_id):
        if not stream_id or '/' in stream_id:
            raise ValueError('Invalid production stream')
        self.stream_id = stream_id
        self.path = Path(filename).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = RLock()
        self.db = sqlite3.connect(self.path, timeout=5, isolation_level=None, check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.execute('PRAGMA foreign_keys=ON')
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=FULL')
        version = self.db.execute('PRAGMA user_version').fetchone()[0]
        if version not in (0, 1):
            raise ValueError(f'Unsupported production history version: {version}')
        self.db.executescript("""
            CREATE TABLE IF NOT EXISTS materials (
                instance_id TEXT PRIMARY KEY,
                stream_id TEXT NOT NULL,
                plc_number INTEGER NOT NULL,
                type_id TEXT NOT NULL,
                created_at TEXT NOT NULL,
                UNIQUE(stream_id, plc_number)
            );
            CREATE TABLE IF NOT EXISTS reservations (
                reservation_id TEXT PRIMARY KEY,
                instance_id TEXT NOT NULL REFERENCES materials(instance_id),
                supply_epoch TEXT NOT NULL,
                slot_id TEXT NOT NULL,
                status TEXT NOT NULL CHECK(status IN ('ACTIVE','ABANDONED','RELEASED')),
                created_at TEXT NOT NULL,
                closed_at TEXT
            );
            CREATE UNIQUE INDEX IF NOT EXISTS active_conveyor_slot
                ON reservations(slot_id, supply_epoch) WHERE status='ACTIVE';
            CREATE TABLE IF NOT EXISTS supply_snapshots (
                stream_id TEXT PRIMARY KEY,
                epoch TEXT NOT NULL,
                slot_id TEXT NOT NULL,
                plc_number INTEGER NOT NULL,
                type_id TEXT NOT NULL,
                available INTEGER NOT NULL,
                reservation_id TEXT NOT NULL,
                updated_at TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS plc_cycles (
                stream_id TEXT NOT NULL,
                plc_number INTEGER NOT NULL,
                phase INTEGER NOT NULL,
                result_state INTEGER NOT NULL,
                reason INTEGER NOT NULL,
                placement INTEGER NOT NULL,
                committed INTEGER NOT NULL,
                observed_at TEXT NOT NULL,
                PRIMARY KEY(stream_id, plc_number)
            );
            CREATE TABLE IF NOT EXISTS inspections (
                instance_id TEXT NOT NULL REFERENCES materials(instance_id),
                inspection_id TEXT NOT NULL,
                station_id TEXT NOT NULL,
                verdict TEXT NOT NULL CHECK(verdict IN ('PASS','FAIL','INCONCLUSIVE')),
                evidence_ref TEXT NOT NULL,
                recorded_at TEXT NOT NULL,
                PRIMARY KEY(instance_id, inspection_id)
            );
            CREATE TABLE IF NOT EXISTS history_events (
                ordinal INTEGER PRIMARY KEY AUTOINCREMENT,
                stream_id TEXT NOT NULL,
                instance_id TEXT,
                source TEXT NOT NULL,
                event_type TEXT NOT NULL,
                details TEXT NOT NULL,
                recorded_at TEXT NOT NULL
            );
        """)
        if version == 0:
            self.db.execute('PRAGMA user_version=1')

    @contextmanager
    def transaction(self):
        with self.lock:
            self.db.execute('BEGIN IMMEDIATE')
            try:
                yield
            except BaseException:
                self.db.execute('ROLLBACK')
                raise
            else:
                self.db.execute('COMMIT')

    def close(self):
        with self.lock:
            self.db.close()

    def _event(self, instance_id, source, kind, details, *, stream_id=None):
        self.db.execute('INSERT INTO history_events(stream_id,instance_id,source,event_type,details,recorded_at) VALUES (?,?,?,?,?,?)',
                        (stream_id or self.stream_id, instance_id, source, kind, json.dumps(details, sort_keys=True), timestamp()))

    @staticmethod
    def _identifier(snapshot):
        if snapshot['plc_number'] <= 0:
            return None
        return f"{snapshot['stream_id']}/{snapshot['plc_number']}"

    def _material(self, snapshot):
        instance = self._identifier(snapshot)
        if not instance:
            return None
        material_type = snapshot['type_id']
        if not material_type:
            raise ValueError('A physical material requires its original type')
        row = self.db.execute('SELECT type_id FROM materials WHERE instance_id=?', (instance,)).fetchone()
        if row:
            if row['type_id'] != material_type:
                raise ValueError('Material type cannot change after creation')
        else:
            self.db.execute('INSERT INTO materials VALUES (?,?,?,?,?)',
                            (instance, self.stream_id, snapshot['plc_number'], material_type, timestamp()))
            self._event(instance, 'equipment', 'MATERIAL_IDENTIFIED',
                        dict(type_id=material_type, plc_number=snapshot['plc_number']))
        return instance

    def record_created(self, plc_number, material_type_id):
        """A positive physical creation receipt is not a Pick arrival."""
        if type(plc_number) is not int or plc_number <= 0 or not material_type_id:
            return
        with self.transaction():
            self._material(dict(stream_id=self.stream_id, plc_number=plc_number,
                                type_id=material_type_id))

    def supply_snapshot(self):
        with self.lock:
            row = self.db.execute('SELECT * FROM supply_snapshots WHERE stream_id=?',
                                  (self.stream_id,)).fetchone()
            return dict(row) if row else None

    def save_supply(self, snapshot, *, released=False):
        """Atomically save material, reservation lifecycle and current projection."""
        if snapshot['stream_id'] != self.stream_id or not snapshot['epoch'] or not snapshot['slot_id']:
            raise ValueError('Supply snapshot identity mismatch')
        with self.transaction():
            previous = self.db.execute('SELECT * FROM supply_snapshots WHERE stream_id=?',
                                       (self.stream_id,)).fetchone()
            if previous and snapshot['plc_number'] < previous['plc_number']:
                raise ValueError('Cannot regress the physical arrival number')
            instance = self._material(snapshot)
            previous_token = previous['reservation_id'] if previous else ''
            token = snapshot['reservation_id']
            if previous_token and previous_token != token:
                self.db.execute(
                    "UPDATE reservations SET status=?,closed_at=? WHERE reservation_id=? AND status='ACTIVE'",
                    ('RELEASED' if released else 'ABANDONED', timestamp(), previous_token))
                self._event(f"{self.stream_id}/{previous['plc_number']}", 'supply',
                            'RESERVATION_CLOSED', dict(reservation_id=previous_token, released=bool(released)))
            if token and token != previous_token:
                if not instance or not snapshot['available']:
                    raise ValueError('Cannot reserve an unavailable material')
                self.db.execute(
                    "INSERT INTO reservations VALUES (?,?,?,?,?,?,?)",
                    (token, instance, snapshot['epoch'], snapshot['slot_id'],
                     'ACTIVE', timestamp(), None))
                self._event(instance, 'supply', 'RESERVATION_OPENED', dict(reservation_id=token, epoch=snapshot['epoch']))
            current = (snapshot['epoch'], snapshot['slot_id'], snapshot['plc_number'],
                       snapshot['type_id'], int(bool(snapshot['available'])), token)
            old = ((previous['epoch'], previous['slot_id'], previous['plc_number'],
                    previous['type_id'], previous['available'], previous_token) if previous else None)
            if current != old:
                self.db.execute("""INSERT INTO supply_snapshots VALUES (?,?,?,?,?,?,?,?)
                    ON CONFLICT(stream_id) DO UPDATE SET epoch=excluded.epoch,
                    slot_id=excluded.slot_id,plc_number=excluded.plc_number,
                    type_id=excluded.type_id,available=excluded.available,
                    reservation_id=excluded.reservation_id,updated_at=excluded.updated_at""",
                    (self.stream_id, *current, timestamp()))
                if previous is None or (snapshot['plc_number'],snapshot['available']) != (
                        previous['plc_number'], previous['available']):
                    self._event(instance, 'supply', 'ASSIGNMENT_CHANGED',
                                dict(slot_id=snapshot['slot_id'], available=bool(snapshot['available'])))

    def record_plc(self, command):
        """Record PLC-reported facts, not a second Cell Commit decision."""
        plc_number = int(command.cycle)
        if plc_number <= 0:
            return
        fields = (int(command.phase), int(command.result_state),
                  int(command.reason), int(command.placement))
        with self.transaction():
            row = self.db.execute('SELECT * FROM plc_cycles WHERE stream_id=? AND plc_number=?',
                                  (self.stream_id, plc_number)).fetchone()
            committed = int(command.committed == plc_number)
            committed = max(committed, int(row['committed'])) if row else committed
            if row and fields == tuple(row[k] for k in ('phase','result_state','reason','placement')) and committed == row['committed']:
                return
            self.db.execute("""INSERT INTO plc_cycles VALUES (?,?,?,?,?,?,?,?)
                ON CONFLICT(stream_id,plc_number) DO UPDATE SET
                phase=excluded.phase,result_state=excluded.result_state,reason=excluded.reason,
                placement=excluded.placement,committed=excluded.committed,observed_at=excluded.observed_at""",
                (self.stream_id, plc_number, *fields, committed, timestamp()))
            self._event(f'{self.stream_id}/{plc_number}', 'plc', 'PROCESS_OBSERVED',
                        dict(phase=fields[0], result_state=fields[1], reason=fields[2],
                             placement=fields[3], committed=bool(committed)))
            # The PLC may commit the previous part while it prepares the next one.
            previous_commit = int(command.committed)
            if 0 < previous_commit < plc_number:
                prior = self.db.execute('SELECT * FROM plc_cycles WHERE stream_id=? AND plc_number=?',
                                        (self.stream_id, previous_commit)).fetchone()
                if prior and not prior['committed']:
                    self.db.execute('UPDATE plc_cycles SET committed=1,observed_at=? WHERE stream_id=? AND plc_number=?',
                                    (timestamp(), self.stream_id, previous_commit))
                    self._event(f'{self.stream_id}/{previous_commit}', 'plc', 'COMMIT_OBSERVED',
                                dict(plc_number=previous_commit))

    def inspect(self, *, instance_id, inspection_id, station_id, verdict, evidence_ref):
        if not all(isinstance(s, str) and s.strip()
                   for s in (instance_id, inspection_id, station_id, evidence_ref)):
            raise ValueError('Inspection requires instance, inspection, station and evidence')
        if verdict not in ('PASS', 'FAIL', 'INCONCLUSIVE'):
            raise ValueError('Unsupported inspection verdict')
        with self.transaction():
            material = self.db.execute('SELECT stream_id FROM materials WHERE instance_id=?',
                                       (instance_id,)).fetchone()
            if material is None:
                raise ValueError('Unknown physical material')
            row = self.db.execute(
                'SELECT station_id,verdict,evidence_ref FROM inspections WHERE instance_id=? AND inspection_id=?',
                (instance_id, inspection_id)).fetchone()
            if row:
                if tuple(row) != (station_id, verdict, evidence_ref):
                    raise ValueError('Conflicting inspection ID')
                return False
            self.db.execute('INSERT INTO inspections VALUES (?,?,?,?,?,?)',
                            (instance_id, inspection_id, station_id, verdict, evidence_ref, timestamp()))
            self._event(instance_id, station_id, 'INSPECTION_RECORDED',
                        dict(inspection_id=inspection_id, verdict=verdict, evidence_ref=evidence_ref),
                        stream_id=material['stream_id'])
            return True

    def history(self, instance_id):
        with self.lock:
            material = self.db.execute('SELECT * FROM materials WHERE instance_id=?',
                                       (instance_id,)).fetchone()
            if material is None:
                return None
            rows = {}
            for key, query in (
                ('reservations', 'SELECT * FROM reservations WHERE instance_id=? ORDER BY created_at'),
                ('inspections', 'SELECT * FROM inspections WHERE instance_id=? ORDER BY recorded_at'),
                ('events', 'SELECT * FROM history_events WHERE instance_id=? ORDER BY ordinal')):
                rows[key] = [dict(row) for row in self.db.execute(query, (instance_id,))]
            rows['material'] = dict(material)
            row = self.db.execute('SELECT * FROM plc_cycles WHERE stream_id=? AND plc_number=?',
                                  (material['stream_id'], material['plc_number'])).fetchone()
            rows['plc'] = dict(row) if row else None
            return rows

    def summary(self):
        with self.lock:
            return dict(
                materials=self.db.execute('SELECT COUNT(*) FROM materials').fetchone()[0],
                plc_commits=self.db.execute('''SELECT COUNT(*) FROM plc_cycles p
                    JOIN materials m ON p.stream_id=m.stream_id AND p.plc_number=m.plc_number
                    WHERE p.committed=1''').fetchone()[0],
                inspected=self.db.execute('SELECT COUNT(DISTINCT instance_id) FROM inspections').fetchone()[0],
                unresolved_reservations=self.db.execute("SELECT COUNT(*) FROM reservations WHERE status='ACTIVE'").fetchone()[0],
                streams=self.db.execute('SELECT COUNT(DISTINCT stream_id) FROM materials').fetchone()[0])
