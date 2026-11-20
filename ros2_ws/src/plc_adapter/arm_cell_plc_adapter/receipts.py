"""Durable command to Action UUID correlation for one PLC request stream."""
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
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


class Receipts:
    def __init__(self, path):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(path, timeout=0, isolation_level=None)
        # A second adapter must not recover a live sender's pending goal.
        self.db.execute('PRAGMA locking_mode=EXCLUSIVE')
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('''CREATE TABLE IF NOT EXISTS receipt (
            command_id INTEGER PRIMARY KEY, goal_uuid BLOB NOT NULL, state INTEGER NOT NULL,
            reason INTEGER NOT NULL DEFAULT 0, placement INTEGER NOT NULL DEFAULT 0)''')
        self.db.execute('UPDATE receipt SET state=? WHERE state IN (?, ?, ?)',
                        (State.UNCERTAIN, State.SUBMITTED, State.ACCEPTED, State.RUNNING))

    def close(self):
        self.db.close()

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

    def update(self, command_id, state, reason=0, placement=0):
        self.db.execute('UPDATE receipt SET state=?, reason=?, placement=? WHERE command_id=?',
                        (state, reason, placement, command_id))

    def reply(self, requested_id):
        if requested_id:
            row = self.db.execute('SELECT * FROM receipt WHERE command_id=?',
                                  (requested_id,)).fetchone()
            if row:
                return Receipt(*row)
        return self.latest() or Receipt(0, b'', State.UNCERTAIN)
