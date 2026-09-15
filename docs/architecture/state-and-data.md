# [SF-Twin] State and Data Architecture

- **Document ID:** `[SF-Twin]_ARCH-STATE-DATA_v1.1.0`
- **Document Type:** Architecture
- **Architecture Scope:** SYSTEM
- **Version:** 1.1.0
- **Status:** Draft
- **Owner:** Architecture Authority
- **Source:** `[SF-Twin]_SAD_v13.0` decomposition

- **Related Capabilities:** `BF-LITE`, `BF-STD`, `GF-D1`, `GF-D2`

## 1. Purpose

System-level state authority, asset identity, replication, arbitration,
offline continuity, data lifecycle, storage responsibility를 정의한다.

## 2. Canonical State / Data Authority

### 2.1 PackML Local Master

Field Edge의 PackML Local Master가 canonical operational-state authority를 가진다.

Cloud MES/APS state는 Read Replica이며 직접적인 field state-transition authority를 갖지 않는다.

### 2.2 Production Count Arbitration

`FR-BF-STD-02`에 따라 Vision count와 CNC M-code count가 충돌하면:

- CNC M-code count가 canonical Master production count다.
- Vision count는 validation / diagnostic log로 유지한다.
- Cloud KPI/OEE 계산은 canonical Master count를 사용해야 한다.

### 2.3 Hardware Safety State

Physical safety actuation authority는 hardware safety circuit에 있다.

Software safety state는 observation / coordination / audit representation이며
physical safety authority로 승격되지 않는다.

### 2.4 Asset Identity

`GPC-02`에 따라 Brownfield → Greenfield lifecycle에서 asset identity와 canonical metadata continuity를 유지한다.

Asset metadata는 IDTA AAS Metamodel V3.0에 정합되는 canonical identity model을 사용한다.

Exact submodel/schema mapping은 downstream data design이 소유한다.

## 3. State / Data Classes

| State / Data Class | Canonical Scope | System Role |
|---|---|---|
| PackML operational state | Edge | field process authority |
| Cloud PackML replica | Cloud | monitoring / scheduling read model |
| Production Master count | CNC / Edge arbitration | OEE/production canonical count |
| Vision count | Edge / Cloud log | validation / diagnostic evidence |
| Safety relay physical state | Hardware | physical safety authority |
| Asset identity / AAS metadata | Product/Digital Twin canonical model | lifecycle continuity |
| Calibration / physical baseline | Digital Twin | engineering / optimization reference |
| Local control state | Edge | runtime application / validation |
| Optimization proposal | Cloud | non-authoritative correction candidate |
| Accepted correction/parameter | Edge acceptance boundary | runtime/twin update input |
| Artifact metadata | Cloud | release/version lifecycle |
| Offline operational log | Edge → Cloud | continuity and audit |

## 4. Ordering and Restart Continuity

Distributed operational events는 out-of-order/backfill로 current state를 regress시키지 않도록
logical ordering metadata를 가져야 한다.

Edge restart 시 last committed ordering state를 local durable storage에서 복구해
operational continuity를 유지해야 한다.

Exact epoch/sequence representation은 ICD/data design이 소유한다.

## 5. Storage Tiering

### Cloud

- relational transactional/master data
- session/idempotency data
- aggregated time-series
- object/binary artifacts
- digital-twin / simulation assets

### Edge

- low-latency memory/ring buffer
- durable WAL/store-and-forward
- local time-series/log buffer
- runtime state/cache

Concrete DB product와 table/entity mapping은 downstream design이 소유한다.

## 6. Offline Continuity and Back-Fill

`GF-D2` field cell은 external network 단절 시 최소 72시간 autonomous operation을 지속할 수 있어야 한다.

Disconnected operation 중:

- required operational state는 local authority를 유지한다.
- logs는 local safe buffer에 보존한다.
- connectivity 복구 후 automatic back-fill한다.

Back-fill은 current state를 과거 상태로 되돌려서는 안 된다.

## 7. Data Priority / Lifecycle

Storage pressure 또는 reconnect backlog에서 최소한 다음 priority를 유지한다.

1. safety / sanity / Process Pause audit
2. PackML transition / canonical production result
3. accepted correction / parameter history
4. validation/diagnostic data
5. lower-priority raw telemetry

Privacy-sensitive clip은 bounded retention을 가져야 한다.

Exact retention period와 purge procedure는 policy/downstream design에서 정의한다.

## 8. Idempotency and Consistency

Delayed retry / back-fill에 의해 동일 transaction이 중복 적용되지 않도록
Cloud ingest path는 idempotency를 보장해야 한다.

Exact TTL/window는 implementation-specific이며 requirement가 없는 한 HLD에서 고정하지 않는다.

## 9. State and Data Invariants

- 동일 operational state에 둘 이상의 canonical writer가 존재해서는 안 된다.
- Cloud replica는 Edge Local Master보다 높은 field-state authority로 승격되어서는 안 된다.
- CNC/Vision count conflict에서 Vision count가 canonical production count를 덮어써서는 안 된다.
- out-of-order/back-fill event가 current state를 regression시켜서는 안 된다.
- network outage가 required local control state continuity를 제거해서는 안 된다.
- rejected optimization proposal이 accepted parameter history로 기록되어서는 안 된다.
- Tier transition에서 asset identity가 새 identity로 임의 재발급되어서는 안 된다.
- safety/state evidence는 low-priority telemetry보다 높은 retention priority를 가져야 한다.

## 10. Product Traceability

- `FR-BF-STD-02`
- `NFR-BF-STD-03`
- `FR-GF-D2-03`, `FR-GF-D2-04`
- `NFR-GF-D2-06`, `NFR-GF-D2-07`, `NFR-GF-D2-08`
- `GPC-01`, `GPC-02`

## 11. Open Decisions

- logical ordering identifier의 exact representation은 current implementation과 downstream ICD/data design에서 확정한다.
- AAS submodel/schema profile은 product `GPC-02`를 보존하는 범위에서 별도 data design이 소유한다.
