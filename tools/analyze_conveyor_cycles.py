#!/usr/bin/env python3
"""Report observations for an explicit contiguous window of PLC material numbers."""
import argparse
from collections import Counter
import json
from pathlib import Path
from statistics import mean, median

from conveyor_cycle_evidence import Evidence


def elapsed(start, end):
    if type(start) is int and type(end) is int and 0 <= start <= end:
        return (end - start) / 1e9
    return None


def interval(start, end, producer):
    if start and end and start.get(producer) and start[producer] == end.get(producer):
        return elapsed(start.get('observed_monotonic_ns'), end.get('observed_monotonic_ns'))
    return None


def first(records):
    return next(iter(records), {})


class Cycle:
    def __init__(self, instance):
        self.instance = instance
        self.failures = []
        self.gaps = []
        self.stages = {}
        self.durations = {}

    def fail(self, stage, reason):
        finding = dict(stage=stage, reason=reason)
        if finding not in self.failures:
            self.failures.append(finding)

    def require(self, condition, description):
        if not condition and description not in self.gaps:
            self.gaps.append(description)

    def identity(self, record, expected, source):
        for key, value in expected.items():
            observed = record.get(key)
            if observed and value and observed != value:
                self.fail('identity', f'{source}: {key}={observed}, expected {value}')
            self.require(observed and value, f'{source}: missing {key}')

    def check_attempts(self, attempts, lease_by_id, executed):
        successful = []
        for a in attempts:
            label = f"command {a['command_id']}"
            self.identity(a, {'material_instance_id': self.instance}, label)
            lease = lease_by_id.get(a['reservation_id'])
            self.require(lease, f'{label}: missing matching reservation')
            self.require(a.get('mission_goal_uuid'), f'{label}: missing Mission UUID')
            outcome = a['task_outcome'] or {}
            # Rejected requests and this specific pre-BT BUSY can legally precede a retry.
            if a['state'] == 7 or (a['state'], a['reason'], a['placement']) == (6, 9, 1):
                continue
            task = a['task_goal_uuid']
            if a['state'] in (5, 6):
                self.fail(outcome.get('failure_stage') or 'mission',
                          outcome.get('diagnostic_detail') or f"mission state={a['state']} reason={a['reason']}")
            if outcome.get('status') in (5, 6) or (
                    outcome.get('status') == 4 and outcome.get('result_code') not in (0, None)):
                self.fail(outcome.get('failure_stage') or 'task',
                          outcome.get('diagnostic_detail') or f"task result={outcome.get('result_code')}")
            if task and a['placement'] == 2 and outcome.get('status') in (4, 5, 6):
                executed.add(task)
            if task and outcome.get('status') == 4 and outcome.get('result_code') == 0:
                executed.add(task)
                if a['placement'] == 1:
                    self.fail('place', 'successful Task reported material_placed=false')
                if (a['state'], a['reason'], a['placement']) == (4, 7, 2):
                    successful.append(a)
            self.require(a['state'] in (4, 5, 6), f'{label}: Mission terminal unavailable')
            self.require(task and outcome.get('status') in (4, 5, 6) and outcome.get('result_code') is not None,
                         f'{label}: Task identity or terminal unavailable')

        return successful

    def check_missions(self, missions, attempts, lease_by_id, material_type):
        jobs = {a['mission_goal_uuid']: a for a in attempts}
        for record in missions:
            event = record.get('event')
            a = jobs.get(record.get('execute_cycle_id'))
            # A linked owner terminal remains evidence even if the adapter file was lost.
            if event == 'task_outcome':
                busy = (record.get('task_status'), record.get('task_result_code'), record.get('failure_stage'),
                        record.get('material_placed')) == (6, 10, 'configured_supply:execution_admission', False)
                if not busy and (record.get('task_status') in (5, 6) or
                        (record.get('task_status') == 4 and record.get('task_result_code') not in (None, 0))):
                    self.fail(record.get('failure_stage') or 'task',
                              record.get('diagnostic_detail') or f"task result={record.get('task_result_code')}")
            if event == 'mission_outcome' and record.get('sequence_completed') is False:
                if record.get('exit_reason') != 'MISSION_EXIT_TASK_ADMISSION_BUSY':
                    self.fail('mission', record.get('exit_reason') or 'Mission reported incomplete sequence')
            if not a:
                self.require(False, f"{event}: Mission UUID has no matching receipt")
                continue
            if event in ('reservation_confirmed', 'task_accepted', 'task_outcome'):
                identity = dict(material_instance_id=self.instance, reservation_id=a['reservation_id'],
                                material_type_id=material_type)
                lease = lease_by_id.get(a['reservation_id'])
                if lease:
                    identity.update(supply_epoch=lease['supply_epoch'], slot_id=lease['slot_id'])
                if event == 'task_accepted':
                    identity['task_goal_uuid'] = a['task_goal_uuid']
                self.identity(record, identity, event)
            if event == 'task_outcome':
                outcome = a['task_outcome'] or {}
                for key, known in [('task_status', outcome.get('status')),
                                   ('task_result_code', outcome.get('result_code')),
                                   ('material_placed', {1: False, 2: True}.get(a['placement']))]:
                    if known is not None and record.get(key) is not None and record[key] != known:
                        self.fail('task', f'{event}: {key} conflicts with receipt')

    def suction_observations(self, tools, attempts, lease_by_id, material_type, executed):
        tasks = {a['task_goal_uuid']: a for a in attempts if a['task_goal_uuid']}
        observations = {}
        for rec in tools:
            ref = rec['reference']
            task = rec.get('execution_id')
            a = tasks.get(task)
            token = ref.get('reservation_id')
            lease = lease_by_id.get(token)
            expected = dict(material_instance_id=self.instance, material_type_id=material_type,
                            motion_goal_uuid=task)
            if a:
                expected['reservation_id'] = a['reservation_id']
            if lease:
                expected.update(supply_epoch=lease['supply_epoch'], slot_id=lease['slot_id'])
            self.identity(ref, expected, 'surface suction')
            self.require(a and lease, 'surface suction: missing Task or reservation link')
            if not task or ref.get('material_instance_id') != self.instance or not lease:
                continue
            state = (rec.get('phase'), rec.get('holding_state'))
            if state not in (('capture', 'held'), ('release', 'released')):
                continue
            if state == ('capture', 'held'):
                executed.add(task)
            group = observations.setdefault(task, {'held': [], 'released': []})
            group[state[1]].append(rec)

        return observations

    def robot(self, attempts, missions, tools, leases, material_type):
        lease_by_id = {r['reservation_id']: r for r in leases}
        executed = set()
        successful = self.check_attempts(attempts, lease_by_id, executed)
        self.check_missions(missions, attempts, lease_by_id, material_type)
        observations = self.suction_observations(tools, attempts, lease_by_id, material_type, executed)
        if len(executed) > 1:
            self.fail('task', 'distinct Task executions for one material')
        self.require(len(successful) == 1, 'missing or ambiguous successful Mission/Task receipt')
        if len(successful) > 1:
            self.fail('mission', 'multiple successful commands for one material')
        a = first(successful)
        self.stages['task'] = bool(a)
        # Preserve physical observations even when a later Task/retreat failed.
        selected = a or first(attempt for attempt in attempts if attempt['task_goal_uuid'] in observations)
        task = selected.get('task_goal_uuid')
        if not task and len(observations) == 1:
            task = next(iter(observations))
        states = observations.get(task, {})
        held = earliest(states.get('held', []))
        # Opening an empty cup before capture is a valid RELEASED observation.
        releases = states.get('released', [])
        if held.get('producer') and type(held.get('observed_monotonic_ns')) is int:
            releases = [r for r in releases if (r.get('coverage') or {}).get('producer_instance') != held['producer']
                        or type(r.get('observed_monotonic_ns')) is not int
                        or r['observed_monotonic_ns'] >= held['observed_monotonic_ns']]
        release = earliest(releases)
        self.stages['pick'] = bool(held)
        self.stages['place'] = bool(release and selected.get('placement') == 2)
        reservation = lease_by_id.get(selected.get('reservation_id'), {})
        self.stages['retirement'] = reservation.get('status') == 'RELEASED'
        self.durations['held_to_release'] = interval(held, release, 'producer')
        accepted = [r for r in missions if task and r.get('task_goal_uuid') == task
                    and r.get('event') == 'task_accepted']
        self.durations['task_accepted_to_outcome'] = None
        if len(accepted) == 1:
            start = accepted[0]
            outcomes = [r for r in missions if r.get('event') == 'task_outcome'
                        and r.get('execute_cycle_id') == start.get('execute_cycle_id')
                        and r.get('reservation_id') == start.get('reservation_id')]
            if len(outcomes) == 1:
                self.durations['task_accepted_to_outcome'] = interval(start, outcomes[0], 'file')
        return selected

    def supply(self, material, leases, events):
        lease_by_id = {r['reservation_id']: r for r in leases}
        assignments = {}
        opened = {}
        for e in events:
            facts = e['facts']
            kind = e['event_type']
            if (e['source'], kind) == ('equipment', 'MATERIAL_IDENTIFIED') and material:
                self.identity(facts, {key: material[key] for key in ('type_id', 'plc_number')}, kind)
            if e['source'] != 'supply':
                continue
            if kind == 'ASSIGNMENT_CHANGED':
                assignments[facts.get('slot_id')] = facts.get('available')
            elif kind in ('RESERVATION_OPENED', 'RESERVATION_CLOSED'):
                token = facts.get('reservation_id')
                lease = lease_by_id.get(token)
                self.require(lease, f'{kind}: reservation row unavailable')
                if not lease:
                    continue
                if kind == 'RESERVATION_OPENED':
                    self.identity(facts, {'epoch': lease['supply_epoch']}, kind)
                    availability = assignments.get(lease['slot_id'])
                    if availability is False:
                        self.fail('reservation', 'reservation opened on a retired assignment')
                    self.require(availability is True, 'missing prior assignment of reserved slot')
                    opened[token] = True
                else:
                    self.require(token in opened, 'reservation close without observed open')
                    if opened.get(token) is False:
                        self.fail('reservation', 'duplicate close for the same reservation')
                    opened[token] = False
                    expected = {'RELEASED': True, 'ABANDONED': False}.get(lease['status'])
                    if facts.get('released') is not expected:
                        self.fail('reservation', 'release history conflicts with reservation row')
        for token, lease in lease_by_id.items():
            self.require(token in opened, 'missing reservation lifecycle events')
            if lease['status'] != 'ACTIVE':
                self.require(opened.get(token) is False, 'missing reservation close')

    def production(self, events, projection, selected, duplicate_commit, next_cycle):
        def facts(source, kind):
            return [e['facts'] for e in events if (e['source'], e['event_type']) == (source, kind)]

        plc = facts('plc', 'PROCESS_OBSERVED')
        commits = facts('plc', 'COMMIT_OBSERVED')
        discharge = facts('equipment', 'DISCHARGED')
        phases = [p.get('phase') for p in plc]
        self.stages.update(supply=bool(facts('equipment', 'MATERIAL_IDENTIFIED')),
                           assignment=any(f.get('available') is True for f in facts('supply', 'ASSIGNMENT_CHANGED')))
        commit = first(e['facts'] for e in events if e['source'] == 'plc' and (
            e['event_type'] == 'COMMIT_OBSERVED' or e['facts'].get('committed') is True))
        reusable = first(p for p in plc if p.get('phase') == 0 and p.get('committed') is True)
        self.stages.update(commit=bool(commit or projection.get('committed') == 1),
                           discharge=bool(discharge or reusable or next_cycle),
                           plc_reusable=bool(reusable or next_cycle))
        if any(p.get('committed') is True and p.get('phase') in (10, 20, 21, 22, 23, 30) for p in plc):
            self.fail('PLC sequence', 'PLC returned to pre-discharge work for the same material')

        if projection.get('committed') == 0 and commit:
            self.fail('commit', 'PLC commit history conflicts with final projection')
        if selected.get('state') in (4, 5, 6) and projection.get('phase') in (0, 50, 51):
            for plc_field, receipt_field in [('placement', 'placement'), ('result_state', 'state'), ('reason', 'reason')]:
                left, right = projection.get(plc_field), selected.get(receipt_field)
                if left and right and left != right:
                    self.fail('PLC terminal', f'{plc_field}={left} conflicts with receipt {right}')
        if duplicate_commit or len(commits) > 1:
            self.fail('commit', 'repeated PLC Commit observation after a distinct register value or explicit event')
        if len(discharge) > 1:
            self.fail('discharge', 'multiple physical removals for one material')
        for p in plc:
            if p.get('phase') == 39:
                if p.get('result_state') in (5, 6):
                    self.fail('PLC terminal', f"reconciliation: state={p['result_state']} reason={p.get('reason')}")
                else:
                    self.require(False, 'PLC awaiting field reconciliation (phase 39)')
        resolved = facts('sidecar', 'RESERVATION_RECONCILED')
        for reconnect in facts('sidecar', 'SIDECAR_RECONNECTED'):
            if reconnect.get('reservation_id'):
                self.require(any(r.get('writer_id') == reconnect.get('writer_id')
                                 and r.get('reservation_id') == reconnect['reservation_id']
                                 and r.get('result') == 'retained' for r in resolved),
                             'restored reservation: physical retention unverified')
        for r in resolved:
            if r.get('result') == 'identity_conflict':
                self.fail('reservation', 'Sidecar reported a physical identity conflict')
        for phase_name, phase_ids in [('P1', (10,)), ('P2', (20, 21, 22)), ('P3', (30, 40))]:
            self.require(any(p in phase_ids for p in phases), f'missing PLC {phase_name} observation')
        physical = discharge[0] if len(discharge) == 1 else {}
        self.durations['cycle'] = elapsed(physical.get('spawned_ns'), physical.get('removed_ns'))
        p1 = first(p for p in plc if p.get('phase') == 10)
        assignment = first(f for f in facts('supply', 'ASSIGNMENT_CHANGED') if f.get('available') is True)
        opened = first(facts('supply', 'RESERVATION_OPENED'))
        for name, start, end in [('p1_to_pick_assignment', p1, assignment),
                                 ('assignment_to_reservation', assignment, opened),
                                 ('p1_to_plc_reusable', p1, reusable), ('p1_to_commit', p1, commit), ('commit_to_plc_reusable', commit, reusable)]:
            self.durations[name] = interval(start, end, 'writer_id')
        return phases


def earliest(records):
    if not records:
        return {}
    # Times from different writer instances cannot be ordered.
    annotated = [dict(r, producer=(r.get('coverage') or {}).get('producer_instance')) for r in records]
    stamps = [r.get('observed_monotonic_ns') for r in annotated]
    if len({r['producer'] for r in annotated}) != 1 or any(type(t) is not int for t in stamps):
        return {'observed': True}
    return min(annotated, key=lambda r: r['observed_monotonic_ns'])


def evaluate(evidence, stream, number):
    instance = f'{stream}/{number}'
    result = Cycle(instance)
    material = evidence.materials.get(number, {})
    result.require(material, 'missing material')
    if material:
        result.identity(material, {'instance_id': instance}, 'ledger')
    leases = evidence.leases[instance]
    result.stages['reservation'] = bool(leases)
    events = evidence.events[instance]
    result.supply(material, leases, events)
    attempts = evidence.attempts[instance]
    missions = evidence.missions[instance]
    tools = evidence.tools[instance]
    selected = result.robot(attempts, missions, tools, leases, material.get('type_id'))
    projection = evidence.plc.get(number, {})
    result.require(projection, 'missing final PLC projection')
    phases = result.production(events, projection, selected, instance in evidence.duplicate_commits,
                               evidence.next_cycles.get(instance))
    if evidence.incomplete:
        result.require(False, 'selected evidence truncated, unreadable or dropped (see input_issues and coverage)')
    result.require(not any(r['status'] == 'ACTIVE' for r in leases), 'reservation still active')
    for stage, observed in result.stages.items():
        result.require(observed, f'{stage}: evidence missing or incomplete')
    verdict = 'FAIL' if result.failures else 'UNKNOWN' if result.gaps else 'PASS'
    return dict(material_instance_id=instance, verdict=verdict, stages=result.stages, plc_phases=phases,
                failures=result.failures, gaps=result.gaps, durations_s=result.durations,
                reservations=leases, attempts=attempts, plc=projection, next_allocation=evidence.next_cycles.get(instance),
                history=events, mission_records=missions,
                suction_files=[r['file'] for r in tools])


def analyze(production, receipts, directories, stream, first_number, count):
    evidence = Evidence(production, receipts, directories, stream, first_number, count)
    cycles = [evaluate(evidence, stream, n) for n in range(first_number, first_number + count)]
    counts = {v: sum(c['verdict'] == v for c in cycles) for v in ('PASS', 'FAIL', 'UNKNOWN')}
    stats = {}
    for name in cycles[0]['durations_s']:
        values = [c['durations_s'][name] for c in cycles if c['durations_s'][name] is not None]
        stats[name] = dict(n=len(values), mean=mean(values) if values else None,
                           median=median(values) if values else None, maximum=max(values, default=None))
    failures = Counter((f['stage'], f['reason']) for c in cycles for f in c['failures'])
    known = counts['PASS'] + counts['FAIL']
    return dict(stream=stream, first=first_number, expected=count, sources=evidence.sources,
                commit_write_count=None, commit_evidence='sampled retained PLC register',
                verdict='FAIL' if counts['FAIL'] else 'UNKNOWN' if counts['UNKNOWN'] else 'PASS',
                counts=counts, success_rate=counts['PASS'] / known if known else None,
                measured_cycles=known, durations_s=stats,
                failures=[dict(stage=s, reason=r, cycles=n) for (s, r), n in failures.items()],
                recovery=evidence.recovery, coverage=evidence.coverage, input_issues=evidence.issues, cycles=cycles)


def summary(report):
    counts = report['counts']
    rate = report['success_rate']
    yield (f"{report['verdict']}  {report['stream']}  PASS {counts['PASS']}  FAIL {counts['FAIL']}  "
           f"UNKNOWN {counts['UNKNOWN']} / {report['expected']}")
    yield 'Success among measured cycles: ' + ('unmeasured' if rate is None else f"{rate:.1%} (n={report['measured_cycles']})")
    for name, stats in report['durations_s'].items():
        values = 'unmeasured' if not stats['n'] else (
            f"mean {stats['mean']:.3f}, median {stats['median']:.3f}, max {stats['maximum']:.3f} s")
        yield f"{name} (n={stats['n']}): {values}"
    for c in report['cycles']:
        if c['verdict'] != 'PASS':
            why = [f"{f['stage']}: {f['reason']}" for f in c['failures']] + c['gaps']
            yield f"{c['material_instance_id']} {c['verdict']}  " + ', '.join(why)
    if not report['recovery']:
        yield 'Recovery evidence: no linked transitions observed'
    for failure in report['failures']:
        yield f"{failure['stage']} ({failure['cycles']}): {failure['reason']}"
    for r in report['recovery']:
        yield f"{r.get('instance_id') or r.get('material_instance_id') or 'unassigned'}  " + str(
            r.get('transition') or r.get('facts', {}).get('result') or r.get('event_type'))
    yield from report['input_issues']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('production', type=Path)
    parser.add_argument('--stream', required=True)
    parser.add_argument('--first', type=int, default=1)
    parser.add_argument('--count', type=int, default=30)
    parser.add_argument('--receipts', type=Path, help='archived copy of the linked adapter database')
    parser.add_argument('--diagnostic', action='append', type=Path, default=[], help='session diagnostic root (repeatable)')
    parser.add_argument('--json', action='store_true', help='write the full machine-readable report to stdout')
    args = parser.parse_args()
    if min(args.first, args.count) < 1 or not args.stream or '/' in args.stream:
        parser.error('supply a stream ID and positive first material number and count')
    directories = list(dict.fromkeys(p.resolve() for p in args.diagnostic))
    report = analyze(args.production, args.receipts, directories, args.stream, args.first, args.count)
    print(json.dumps(report, ensure_ascii=False, indent=2) if args.json else '\n'.join(summary(report)))


if __name__ == '__main__':
    main()
