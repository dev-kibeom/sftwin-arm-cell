"""Durable command to Action UUID correlation for one PLC request stream."""
from contextlib import ExitStack, closing
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
import fcntl
import json
import sqlite3
import uuid


class State(IntEnum):
    IDLE = 0
    SUBMITTED = 1
    ACCEPTED = 2
    RUNNING = 3
    SUCCEEDED = 4
    CANCELED = 5
    ABORTED = 6
    REJECTED = 7
    UNCERTAIN = 8


TERMINAL = (State.SUCCEEDED, State.CANCELED, State.ABORTED, State.REJECTED)


@dataclass(frozen=True)
class Receipt:
    command_id: int
    goal_uuid: bytes
    state: State
    reason: int = 0
    placement: int = 0  # unknown, observed false, observed true


def read_material_attempts(path, instance_id):
    """Read exact Mission/Task links without claiming or recovering any command."""
    uri = Path(path).expanduser().resolve().as_uri() + '?mode=ro'
    with closing(sqlite3.connect(uri, uri=True, timeout=0)) as db:
        db.row_factory = sqlite3.Row
        rows = db.execute("""
            SELECT r.command_id, lower(hex(r.goal_uuid)) AS mission_goal_uuid,
                   r.state, r.reason, r.placement, t.reservation_id, t.task_goal_uuid, t.task_outcome
              FROM receipt AS r JOIN configured_task AS t USING (command_id)
             WHERE t.material_instance_id=? ORDER BY r.command_id""", (instance_id,))
        attempts = []
        for row in rows:
            attempt = dict(row)
            if attempt['task_outcome'] is not None:
                outcome = json.loads(attempt['task_outcome'])
                if not isinstance(outcome, dict) or row['placement'] not in (0, 1, 2):
                    raise ValueError('Invalid Task outcome in PLC receipt')
                outcome['material_placed'] = None if row['placement'] == 0 else row['placement'] == 2
                attempt['task_outcome'] = outcome
            attempts.append(attempt)
        return attempts


class Receipts:
    def __init__(self, path):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        # Only the stream owner may recover or submit. WAL readers need no owner lock.
        with ExitStack() as pending:
            self.owner_lock = pending.enter_context(Path(str(path.resolve()) + '.writer.lock').open('a+b'))
            try:
                fcntl.flock(self.owner_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise sqlite3.OperationalError('PLC receipt writer already locked') from error
            self.db = sqlite3.connect(path, timeout=0, isolation_level=None)
            pending.callback(self.db.close)
            self._initialize()
            pending.pop_all()

    def _initialize(self):
        self.db.execute('PRAGMA foreign_keys=ON')
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('''CREATE TABLE IF NOT EXISTS receipt (
            command_id INTEGER PRIMARY KEY, goal_uuid BLOB NOT NULL, state INTEGER NOT NULL,
            reason INTEGER NOT NULL DEFAULT 0, placement INTEGER NOT NULL DEFAULT 0)''')
        self.db.execute('''CREATE TABLE IF NOT EXISTS configured_task (
            command_id INTEGER PRIMARY KEY REFERENCES receipt(command_id),
            material_instance_id TEXT NOT NULL, reservation_id TEXT NOT NULL,
            task_goal_uuid TEXT, task_outcome TEXT)''')
        self.db.execute('CREATE INDEX IF NOT EXISTS task_material ON configured_task(material_instance_id)')
        self.db.execute('UPDATE receipt SET state=? WHERE state IN (?, ?, ?)',
                        (State.UNCERTAIN, State.SUBMITTED, State.ACCEPTED, State.RUNNING))

    def close(self):
        self.db.close()
        self.owner_lock.close()

    def observe_idle(self):
        if self.latest() is None:
            self.db.execute('INSERT INTO receipt VALUES (0, ?, ?, 0, 0)', (b'', State.IDLE))

    def latest(self):
        row = self.db.execute('SELECT * FROM receipt ORDER BY command_id DESC LIMIT 1').fetchone()
        return Receipt(*row) if row else None

    def claim(self, command_id):
        last = self.latest()
        if last is None or command_id != last.command_id + 1:
            return None
        if last.state != State.IDLE and last.state not in TERMINAL:
            return None
        receipt = Receipt(command_id, uuid.uuid4().bytes, State.SUBMITTED)
        self.db.execute('INSERT INTO receipt (command_id, goal_uuid, state) VALUES (?, ?, ?)',
                        (command_id, receipt.goal_uuid, receipt.state))
        return receipt

    def update(self, command_id, state, reason=0, placement=0, trace=None):
        with self.db:
            self.db.execute('BEGIN IMMEDIATE')
            self.db.execute('UPDATE receipt SET state=?, reason=?, placement=? WHERE command_id=?',
                            (state, reason, placement, command_id))
            if trace is None or not trace.material_instance_id:
                return
            goal_id = bytes(trace.task_goal_id.uuid)
            outcome = None
            if trace.task_status in (4, 5, 6):
                outcome = json.dumps(dict(status=trace.task_status, result_code=trace.result_code,
                                          failure_stage=trace.failure_stage, diagnostic_detail=trace.diagnostic_detail))
            self.db.execute("""
                INSERT INTO configured_task VALUES (?,?,?,?,?)
                ON CONFLICT(command_id) DO UPDATE SET
                    task_goal_uuid=COALESCE(excluded.task_goal_uuid,configured_task.task_goal_uuid),
                    task_outcome=COALESCE(excluded.task_outcome,configured_task.task_outcome)
                WHERE configured_task.material_instance_id=excluded.material_instance_id
                  AND configured_task.reservation_id=excluded.reservation_id""",
                (command_id, trace.material_instance_id, trace.reservation_id,
                 goal_id.hex() if any(goal_id) else None, outcome))

    def reply(self, requested_id):
        if requested_id:
            row = self.db.execute('SELECT * FROM receipt WHERE command_id=?',
                                  (requested_id,)).fetchone()
            if row:
                return Receipt(*row)
        return self.latest() or Receipt(0, b'', State.UNCERTAIN)
