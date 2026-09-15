# [SF-Twin] Deployment Architecture

- **Document ID:** `[SF-Twin]_ARCH-DEPLOYMENT_v1.1.0`
- **Document Type:** Architecture
- **Architecture Scope:** SYSTEM
- **Version:** 1.1.0
- **Status:** Draft
- **Owner:** Architecture Authority
- **Source:** `[SF-Twin]_SAD_v13.0` decomposition
- **Related Product Definition:** `[SF-Twin]_PRD-DEFINITION_v1.0.0`
- **Related Capabilities:** `BF-LITE`, `BF-STD`, `GF-D1`, `GF-D2`

## 1. Purpose

SF-Twin의 Cloud, Edge, field network, OT hardware, client runtime,
Day-1 engineering environment의 deployment boundary와 failure domain을 정의한다.

## 2. Scope

### In Scope

- Cloud / Edge / Client / Engineering / OT zones
- Brownfield → Greenfield migration
- WAN/LAN/control-network separation
- Day-1 Isaac engineering environment
- Day-2 Cloud optimization deployment boundary
- disconnected Edge autonomy
- hardwired safety path

### Out of Scope

- service 내부 framework layering
- exact ROS 2 topic/message/QoS
- DB schema
- component 내부 process/thread 구조
- simulator 내부 implementation detail

## 3. Deployment Zones

### 3.1 Client Zone

- Web browser
- Operator tablet
- engineering/monitoring client

### 3.2 Cloud SaaS Zone

Cloud SaaS는 다음 responsibility를 호스팅할 수 있다.

- API / ingress
- Digital Twin
- MES/APS/KPI/B2B
- artifact/deployment management
- relational/cache/time-series/object storage
- Day-2 MuJoCo Batch SysID workers
- accepted-parameter twin resynchronization orchestration

### 3.3 Day-1 Engineering Validation Zone

`GF-D1`의 field deployment 전 engineering validation environment다.

- LiDAR/point-cloud ingestion
- OpenUSD full-scale replica
- Isaac Sim Track 1 — Macro Line
- Isaac Sim Track 2 — Micro Cell
- engineering evidence generation

이 zone의 validation result는 field deployment input이지만,
simulation runtime 자체가 field control authority는 아니다.

### 3.4 Tier 1/2 Brownfield Edge Zone

`BF-LITE`, `BF-STD`를 위한 non-invasive Edge다.

- edge vision / non-invasive sensing
- legacy machine Read-Only communication
- Virtual Bridge
- PackML Local Master
- local buffering

Brownfield deployment는 factory intranet construction 없이 embedded LTE 기반 remote onboarding을 지원한다.

### 3.5 Tier 3 Greenfield Edge Control Zone

`GF-D1`, `GF-D2` runtime zone이다.

- real-time Edge control
- Local Sanity Guard
- DSP/observer
- parameter/artifact hot-swap runtime
- local state/cache/buffer
- disconnected autonomous operation

Field control IPC는 No-GPU architecture를 기본 constraint로 갖는다.

### 3.6 OT / Hardware Safety Zone

- CNC / machine tools
- robots / AMR
- physical E-Stop devices
- Hardwired Dual-Channel safety relay

Legal E-Stop path는 software deployment와 독립된 physical circuit이다.

## 4. Connectivity Model

### 4.1 Brownfield WAN

`BF-LITE`, `BF-STD`:

- embedded LTE
- no mandatory factory-intranet construction
- non-invasive / read-only integration
- normal operation의 high-volume video WAN streaming 금지

### 4.2 Greenfield Control Network

`GF-D1`, `GF-D2`:

- TSN wired network
- ROS 2 DDS control communication
- sensor/bridge traffic과 deterministic control traffic 분리

Exact QoS / topology는 ICD/deployment-local design이 소유한다.

### 4.3 Cloud-Edge Boundary

Cloud는 field NAT/firewall을 우회하는 arbitrary inbound control path를 기본으로 하지 않는다.

Edge-established secure channel을 통해 telemetry, artifact, async optimization response를 교환한다.

Cloud-originated control-related result는 Local Sanity Guard를 우회할 수 없다.

### 4.4 Local Monitoring

Local on-demand video는 factory LAN boundary에서 제공한다.
WAN normal-operation video path와 분리한다.

## 5. On-Premise Separation

Architecture-level intent:

- non-deterministic sensing / bridge traffic과 deterministic control traffic을 분리한다.
- sensor congestion이 control timing을 침해하지 않아야 한다.
- hardwired safety path는 network path와 독립한다.

구체 VLAN 번호는 deployment-local configuration이며 HLD invariant가 아니다.

## 6. Brownfield → Greenfield Migration

Tier 1/2 lightweight Edge device는 Tier 3 전환 시 반드시 철거하지 않는다.

환경 sensing / legacy communication gateway로 존치할 수 있으며,
Tier 3 control IPC와 runtime authority 및 network responsibility를 분리한다.

Asset identity와 metadata는 `GPC-02`에 따라 lifecycle transition에서 continuity를 유지한다.

## 7. Failure Domains

- Cloud outage가 GF-D2 field autonomous operation을 즉시 제거해서는 안 된다.
- external network disconnect 시 field cell은 최소 72시간 autonomous operation을 지속할 수 있어야 한다.
- disconnected logs는 local buffer에 저장하고 connectivity 복구 후 back-fill한다.
- sensor/bridge network fault가 deterministic control network를 전파적으로 침해해서는 안 된다.
- software service failure가 hardwired E-Stop physical authority를 제거해서는 안 된다.

## 8. Deployment Invariants

- Brownfield는 non-invasive/read-only deployment principle을 유지해야 한다.
- Greenfield deterministic control은 TSN + ROS 2 DDS control class를 사용해야 한다.
- Day-1 Isaac engineering environment와 Day-2 field runtime responsibility를 혼동해서는 안 된다.
- Cloud MuJoCo optimization은 field real-time control runtime에 직접 참여해서는 안 된다.
- Edge control network와 sensor/bridge traffic은 timing isolation을 유지해야 한다.
- Hardwired E-Stop authority는 network topology에 의존해서는 안 된다.
- Tier migration에서 asset identity continuity를 유지해야 한다.

## 9. Product Traceability

- `NFR-BF-LITE-01` ~ `NFR-BF-LITE-03`
- `NFR-BF-STD-01` ~ `NFR-BF-STD-03`
- `NFR-GF-D1-01`, `NFR-GF-D1-06`, `NFR-GF-D1-07`
- `NFR-GF-D2-01`, `NFR-GF-D2-06`, `NFR-GF-D2-07`, `NFR-GF-D2-09`
- `GPC-01`, `GPC-02`

## 10. Open Decisions

Cloud/service concrete products, versions, orchestration platform은 current implementation audit 이후 downstream deployment design에서 결정한다.
