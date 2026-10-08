# [SF-Twin] Communication Architecture

- **Document ID:** `[SF-Twin]_ARCH-COMMUNICATION_v1.1.0`
- **Document Type:** Architecture
- **Architecture Scope:** SYSTEM
- **Version:** 1.1.0
- **Status:** Draft
- **Owner:** Architecture Authority
- **Source:** `[SF-Twin]_SAD_v13.0` decomposition
- **Related Capabilities:** `WEB`, `BF-LITE`, `BF-STD`, `GF-D1`, `GF-D2`

## 1. Purpose

Client, Cloud, Edge, Engineering, OT 사이의 system-level communication boundary,
traffic class, control path, delivery/timing class를 정의한다.

Exact message schema, field, unit, frame/TF, timestamp, QoS는 ICD가 소유한다.

## 2. Communication Classes

### 2.1 Client / Cloud

- HTTPS / WebSocket class
- Web 3D / business API / monitoring
- dashboard state update target ≤ 1 s where `GF-D1` monitoring requirement applies

### 2.2 Local Operator / Edge

Process Pause와 field work input은 local operational-control boundary로 전달한다.

- Process Pause target delivery ≤ 500 ms for `BF-LITE`
- Cloud round-trip을 mandatory Process Pause path로 두지 않는다.

### 2.3 Brownfield Edge / Cloud WAN

`BF-LITE`, `BF-STD`는 embedded LTE 기반 remote deployment를 지원한다.

- normal operation WAN video streaming 금지
- event clip / telemetry / state / business data 중심
- `BF-LITE` monthly LTE traffic target ≤ 100 MB/device

### 2.4 Local Monitoring Video

`GF-D1` on-demand video는 factory LAN에서 제공한다.

- WAN normal-operation video path와 분리
- target stream latency < 500 ms

### 2.5 Brownfield Machine / Bridge

FOCAS / S7 / signal-wrapping 등의 non-invasive Read-Only acquisition class를 사용한다.

Machine control authority를 획득하는 write/control path로 승격하지 않는다.

### 2.6 Greenfield Sensor / Control Network

- sensing / bridge traffic과 deterministic control traffic을 timing class로 분리한다.
- Tier 3 control network는 TSN wired + ROS 2 DDS control class를 사용한다.
- Tier 3 emergency-control packet target ≤ 50 ms

Exact DDS QoS와 topology는 ICD가 소유한다.

### 2.7 Cloud / Edge Day-2 Optimization

Cloud MuJoCo Batch SysID 결과는 asynchronous correction proposal로 전달한다.

- Cloud는 direct trajectory command stream을 소유하지 않는다.
- Edge Local Sanity Guard가 acceptance를 결정한다.
- accepted parameter만 hot-swap/twin resync path로 진행한다.

### 2.8 Hardware Safety

Legal E-Stop actuation path는 software communication architecture와 독립된 hardwired path다.

Hardwired relay interruption target ≤ 100 ms는 network packet delivery requirement와 별개다.

## 3. System-Level Protocol Mapping

| Boundary | Protocol Class / Example | System Intent |
|---|---|---|
| Client ↔ Cloud | HTTPS / WSS | UI, API, Web 3D, monitoring |
| Tablet → Edge | Local REST/WebSocket or equivalent | Process Pause / work input |
| Local Client ↔ Edge Vision | WebRTC / RTSP class | LAN-only on-demand video |
| Brownfield Edge ↔ Cloud | HTTPS / MQTT / gRPC class over LTE | telemetry/events/deployment |
| Machine ↔ Bridge | FOCAS / S7 / signal acquisition | non-invasive read |
| Edge sensor → Tier 3 runtime | ROS 2 / DDS sensing class | sensor flow |
| Tier 3 runtime ↔ Robot/AMR | TSN + deterministic ROS 2 / DDS class | bounded control |
| Cloud MuJoCo ↔ Edge | secure async request/response | Batch SysID / correction proposal |
| Safety Relay → Edge | isolated DI/state observation | hardware safety state observation |
| E-Stop → Safety/Power Circuit | hardwired | physical safety action |

Concrete protocol version, topic/path, QoS, compatibility는 ICD/deployment design이 소유한다.

## 4. Contract Governance

### 4.1 State Replication

PackML state는 Edge Local Master → Cloud Read Replica 방향으로 replication한다.

### 4.2 Production Count Semantics

Count arbitration 결과의 canonical Master는 `state-and-data.md`를 따른다.

Communication layer는 Vision count와 CNC count를 전달할 수 있으나
transport가 authority를 결정하지 않는다.

### 4.3 Control Path Isolation

- Process Pause는 local operational path를 사용한다.
- Cloud-originated optimization response는 local safety/sanity boundary를 우회하지 않는다.
- Hardwired E-Stop physical path는 software/network availability에 의존하지 않는다.

### 4.4 Ordering

Replicated operational event는 out-of-order/back-fill regression을 방지할 ordering metadata를 가져야 한다.

### 4.5 Adaptive Control Boundary

Cloud는 correction/parameter proposal을 제공할 수 있다.

Current field state에 대한 runtime application/trajectory authority는 Edge에 있다.

## 5. HLD vs ICD Boundary

본 문서는 다음을 정의한다.

- boundary purpose
- participant class
- protocol family
- timing/delivery class
- authority direction

다음은 ICD가 정의한다.

- exact topic/path/service/action
- exact schema/field
- unit
- frame/TF semantics
- timestamp semantics
- exact QoS profile
- retry/compatibility semantics
- invalid payload semantics

## 6. Product Traceability

- `NFR-BF-LITE-02`, `NFR-BF-LITE-03`, `NFR-BF-LITE-06`
- `NFR-BF-STD-01`, `NFR-BF-STD-02`
- `NFR-GF-D1-01`, `NFR-GF-D1-03`, `NFR-GF-D1-04`, `NFR-GF-D1-05`
- `FR-GF-D2-02`, `FR-GF-D2-03`
- `GPC-01`

## 7. Open Decisions

Exact protocol/version/QoS tuning은 target deployment와 component ICD에서 확정한다.
HLD-level communication authority는 현재 Product family와 정합된다.
