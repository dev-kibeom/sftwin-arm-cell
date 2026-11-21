"""Read closed Conveyor owner artifacts. Never open a runtime owner for reporting."""
from collections import defaultdict
from contextlib import closing
import json
from pathlib import Path
import sqlite3


def rows(path, statement, args=()):
    uri = Path(path).expanduser().resolve().as_uri() + '?mode=ro'
    with closing(sqlite3.connect(uri, uri=True)) as db:
        db.row_factory = sqlite3.Row
        for row in db.execute(statement, args):
            yield dict(row)


def object_value(value):
    data = json.loads(value)
    if not isinstance(data, dict):
        raise ValueError('expected a JSON object')
    return data


class Evidence:
    def __init__(self, production, receipts, directories, stream, first, count):
        self.issues = []
        self.coverage = []
        self.incomplete = False
        self.next_cycles = {}
        self.leases = defaultdict(list)
        self.events = defaultdict(list)
        self.attempts = defaultdict(list)
        self.missions = defaultdict(list)
        self.tools = defaultdict(list)
        self.recovery = []
        self.duplicate_commits = set()
        self.instances = {f'{stream}/{n}' for n in range(first, first + count)}
        args = (stream, first, first + count)
        self.sources = dict(production=str(production), receipts=None, diagnostic=list(map(str, directories)))
        self.materials = {m['plc_number']: m for m in self.read(production, """
            SELECT * FROM materials WHERE stream_id=? AND plc_number>=? AND plc_number<?""", args)}
        for lease in self.read(production, """SELECT r.* FROM reservations r JOIN materials m
            USING(instance_id) WHERE m.stream_id=? AND m.plc_number>=? AND m.plc_number<?""", args):
            self.leases[lease['instance_id']].append(lease)
        self.plc = {p['plc_number']: p for p in self.read(production, """SELECT * FROM plc_cycles
            WHERE stream_id=? AND plc_number>=? AND plc_number<?""", args)}
        self.by_lease = {r['reservation_id']: instance for instance, leases in self.leases.items() for r in leases}

        last_register = None
        seen_registers = set()
        for row in self.read(production, """SELECT ordinal, instance_id, source, event_type, details, recorded_at
                FROM history_events WHERE stream_id=? AND source IN ('plc','equipment','supply','sidecar')
                AND event_type!='INSPECTION_RECORDED' ORDER BY ordinal""", (stream,)):
            instance = row['instance_id']
            try:
                facts = object_value(row.pop('details'))
            except (TypeError, ValueError) as error:
                if instance in self.instances:
                    self.incomplete = True
                self.issues.append(f"history_events #{row['ordinal']}: {error}")
                continue
            if row['source'] == 'plc' and row['event_type'] == 'PROCESS_OBSERVED':
                register = facts.get('commit_register')
                if (type(register) is int and register > 0 and instance == f'{stream}/{register + 1}'
                        and facts.get('phase') in (10, 20, 21, 22)):
                    prior = f'{stream}/{register}'
                    if prior in self.instances:
                        self.next_cycles.setdefault(prior, dict(ordinal=row['ordinal'], instance_id=instance,
                                                              commit_register=register))
                if type(register) is int and register != last_register:
                    if register > 0 and register in seen_registers:
                        self.duplicate_commits.add(f'{stream}/{register}')
                    if first <= register < first + count:
                        seen_registers.add(register)
                    last_register = register
            if instance in self.instances:
                self.events[instance].append(dict(row, facts=facts))
            if row['source'] == 'sidecar' and (instance in self.instances or not instance):
                self.recovery.append(dict(row, facts=facts))

        if receipts is None:
            bound = next(self.read(production, 'SELECT receipt_file FROM receipt_sources WHERE stream_id=?', (stream,)), {})
            receipts = bound.get('receipt_file')
        has_task_table = False
        if receipts:
            self.sources['receipts'] = str(receipts)
            has_task_table = bool(next(self.read(receipts, "SELECT name FROM sqlite_master WHERE name='configured_task'", ()), None))
            if has_task_table:
                self.read_receipts(receipts)
        else:
            self.issues.append('receipt source not configured')
        self.read_diagnostics(directories)
        if receipts:
            self.link_mission_receipts(receipts, has_task_table)
        if any(r.get('capture_complete') is False for r in self.coverage):
            self.incomplete = True

    def read(self, path, sql, args):
        try:
            yield from rows(path, sql, args)
        except (OSError, sqlite3.Error) as error:
            self.incomplete = True
            self.issues.append(f'{path} ({sql.split("FROM")[-1].split()[0]}): {error}')

    def targets(self, instance, token):
        return self.instances.intersection((instance, self.by_lease.get(token)))

    def read_receipts(self, path):
        # Include conflicting material links via their reservation. A command alone has no material identity.
        for row in self.read(path, """SELECT r.command_id,lower(hex(r.goal_uuid)) AS mission_goal_uuid,
                r.state,r.reason,r.placement,t.* FROM receipt r JOIN configured_task t USING(command_id)""", ()):
            targets = self.targets(row['material_instance_id'], row['reservation_id'])
            if not targets:
                continue
            try:
                if row['task_outcome'] is not None:
                    row['task_outcome'] = object_value(row['task_outcome'])
            except (TypeError, ValueError) as error:
                self.incomplete = True
                self.issues.append(f"{path} command {row['command_id']}: {error}")
                row['task_outcome'] = None
            for instance in targets:
                self.attempts[instance].append(row)

    def link_mission_receipts(self, path, has_task_table):
        # Earlier receipts only contain command -> Mission. Join the explicit lease/Task
        # links in that Mission's saved diagnostics, including when a configured trace was lost.
        for instance, records in self.missions.items():
            known = {a['mission_goal_uuid'] for a in self.attempts[instance]}
            grouped = defaultdict(list)
            for r in records:
                mission = r.get('execute_cycle_id')
                if mission and mission not in known:
                    grouped[mission].append(r)
            for mission, group in grouped.items():
                confirmed = [r for r in group if r.get('material_instance_id') == instance
                             and self.by_lease.get(r.get('reservation_id')) == instance]
                tokens = {r['reservation_id'] for r in confirmed}
                if len(tokens) != 1:
                    continue
                task_ids = {r['task_goal_uuid'] for r in confirmed if r.get('event') == 'task_accepted'
                            and r.get('task_goal_uuid')}
                outcomes = [r for r in confirmed if r.get('event') == 'task_outcome']
                for row in self.read(path, """SELECT command_id, state, reason, placement FROM receipt
                        WHERE lower(hex(goal_uuid))=?""", (mission,)):
                    stored = None
                    if has_task_table:
                        stored = next(self.read(path, 'SELECT * FROM configured_task WHERE command_id=?',
                                                (row['command_id'],)), None)
                    if stored:
                        # Its Mission is linked but the stored material/lease points elsewhere.
                        row.update(stored, task_outcome=None, link_source='conflicting configured_task')
                    else:
                        outcome = None
                        if len(outcomes) == 1:
                            r = outcomes[0]
                            outcome = dict(status=r.get('task_status'), result_code=r.get('task_result_code'),
                                           failure_stage=r.get('failure_stage'), diagnostic_detail=r.get('diagnostic_detail'))
                        row.update(material_instance_id=instance, reservation_id=next(iter(tokens)),
                                   task_goal_uuid=next(iter(task_ids)) if len(task_ids) == 1 else None,
                                   task_outcome=outcome, link_source='mission diagnostics')
                    self.attempts[instance].append(dict(row, mission_goal_uuid=mission))

    def read_diagnostics(self, directories):
        by_mission = defaultdict(set)
        by_task = defaultdict(set)
        for instance, attempts in self.attempts.items():
            for a in attempts:
                by_mission[a['mission_goal_uuid']].add(instance)
                if a['task_goal_uuid']:
                    by_task[a['task_goal_uuid']].add(instance)
        for directory in directories:
            if not directory.is_dir():
                self.incomplete = True
                self.issues.append(f'{directory}: diagnostic directory unavailable')
                continue
            for path in sorted((directory / 'orchestration').glob('configured-supply-*.jsonl')):
                records = list(self.read_json(path, lines=True))
                for record in records:
                    mission = record.get('execute_cycle_id')
                    if mission and record.get('component') == 'orchestration':
                        by_mission[mission].update(self.targets(record.get('material_instance_id'), record.get('reservation_id')))
                for record in records:
                    if record.get('component') != 'orchestration':
                        continue
                    targets = self.targets(record.get('material_instance_id'), record.get('reservation_id'))
                    targets.update(by_mission.get(record.get('execute_cycle_id'), ()))
                    if record.get('event') == 'task_accepted' and record.get('task_goal_uuid'):
                        by_task[record['task_goal_uuid']].update(targets)
                    if record.get('event') == 'supply_observation_delay':
                        if targets or not record.get('material_instance_id'):
                            self.recovery.append(record)
                    else:
                        for instance in targets:
                            self.missions[instance].append(record)
                self.coverage.extend(self.read_json(path.with_suffix('.summary.json'), summary=True))
        for directory in directories:
            for path in sorted((directory / 'gripper').glob('suction-*.json')):
                for record in self.read_json(path):
                    ref = record.get('reference')
                    if not isinstance(ref, dict) or ref.get('strategy') != 'surface_suction':
                        continue
                    targets = self.targets(ref.get('material_instance_id'), ref.get('reservation_id'))
                    targets.update(by_task.get(record.get('execution_id'), ()))
                    for instance in targets:
                        self.tools[instance].append(record)
            summary = directory / 'gripper/capture_summary.json'
            if (directory / 'gripper').is_dir():
                histories = sorted((summary.parent / 'coverage').glob('*.json'))
                for producer_summary in histories or [summary]:
                    self.coverage.extend(self.read_json(producer_summary, summary=True))

    def read_json(self, path, lines=False, summary=False):
        try:
            with path.open(encoding='utf-8') as source:
                for index, raw in enumerate(source if lines else (source.read(),), 1):
                    try:
                        record = object_value(raw)
                        for field in ('reference', 'coverage'):
                            if field in record and not isinstance(record[field], dict):
                                raise ValueError(f'invalid {field}')
                        for obj in (record, record.get('reference', {})):
                            for key in ('execute_cycle_id', 'execution_id', 'task_goal_uuid', 'material_instance_id', 'reservation_id'):
                                if obj.get(key) is not None and not isinstance(obj[key], str):
                                    raise ValueError(f'invalid {key}')
                    except ValueError as error:
                        self.incomplete = True
                        self.issues.append(f'{path}:{index}: {error}')
                        continue
                    record['file'] = str(path)
                    if lines:
                        record['line'] = index
                    yield record
        except (OSError, UnicodeError) as error:
            if not summary or not isinstance(error, FileNotFoundError):
                self.incomplete = True
            self.issues.append(f'{path}: {error}')
