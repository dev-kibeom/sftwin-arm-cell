# [SF-Twin] Product Definition

- **Document ID:** `[SF-Twin]_PRD-DEFINITION_v1.0.0`
- **Document Type:** Product Definition
- **Status:** Draft
- **Authority:** Product identity, lifecycle, global product constraints, canonical code registry
- **Last Updated:** 2026-09-13

## 1. Product Definition

SF-Twin은 무설치 Web 3D simulation을 진입 관문(Top of Funnel)으로 삼아, 기존 라인의 무개조 비대면 관제(Brownfield)부터 신규 무인 셀의 정밀 turnkey 구축 및 24시간 autonomous maintenance(Greenfield)까지 공장 성숙도에 맞춰 단계적으로 확장하는 스마트 제조 DX 플랫폼이다.

## 2. Product Lifecycle / Tier Model

### 2.1 Top of Funnel — Web 3D Concept Design

별도 프로그램 설치 없이 browser에서 factory layout을 구성하고, SF-Twin의 3D digital twin viewer, monitoring console, process optimization 효과, downtime reduction, ROI를 사전 체험하는 진입 관문이다.

### 2.2 Brownfield Track

#### Tier 1 — Lite

진동, 차광 vision 등 비침습 sensor를 셀프 장착하여 equipment utilization, production counting, optional safety advisory monitoring을 제공하는 remote subscription solution이다.

#### Tier 2 — Standard

CNC external Ethernet Read-Only interface, signal-light wrapping, Virtual OPC UA Bridge를 이용해 official OEE/FPY와 Light APS를 제공한다.

### 2.3 Greenfield Track

#### Tier 3 Day-1 — Turnkey Engineering

Remote consultation → ROM → 3D LiDAR survey → Isaac Sim full-scale 2-Track validation → field deployment → FAT/SAT lifecycle을 갖는다.

- **Track 1 — Macro Line:** PackML state, AMR/VDA 5050 logistics, kinematic mock, routing/recovery, line MTTR
- **Track 2 — Micro Cell:** 3D vision, contact/friction/slip, grasp/regrasp, cell-level physical validation

#### Tier 3 Day-2 — Autonomous Operation

Edge DSP/FFT predictive maintenance와 in-cycle correction을 수행하고, cycle 사이 idle interval에는 Cloud MuJoCo Batch SysID를 수행한다.

Cloud correction은 Local Sanity Guard를 통과한 경우에만 hot-swap할 수 있으며, guard violation 시 correction을 Reject하고 Ramp-down Safe Mode로 전환한다.

Accepted physical parameters는 Isaac Sim digital twin에 Auto-Resync한다.

## 3. Core Values

### 3.1 Interactive Form & Outcome Preview

사용자는 Web에서 solution 형태와 monitoring 방식을 직접 체험하고 layout interference, downtime reduction, ROI를 확인할 수 있어야 한다.

### 3.2 Zero-Downtime & Non-Invasive

Brownfield 도입은 equipment control cabinet modification과 PLC ladder modification 없이 이루어져야 한다.

### 3.3 Full-Scale Digital Replica

도면 없는 현장도 3D LiDAR와 Isaac Sim OpenUSD를 통해 1:1 full-scale replica로 복원하고, Macro Line / Micro Cell 2-Track으로 분리 검증한다.

### 3.4 Continuous Accuracy & Zero Changeover

Edge in-cycle correction과 Cloud inter-cycle optimization을 결합하여 24시간 연속 정밀 운용을 지원한다.

### 3.5 Resilient Orchestration

PackML, ROS2, VDA 5050 및 Behavior Tree 기반으로 process-logistics를 연결하고 communication, routing, grasp fault에 대해 recovery 가능해야 한다.

### 3.6 Safety & Modular Architecture

Software/wireless stop은 Process Pause로 한정하고, legal Emergency Stop authority는 Hardwired Dual-Channel E-Stop safety circuit에 둔다.

## 4. Actors

| Actor | Responsibility |
|---|---|
| 기존 공장 의사결정자 | Brownfield Tier 1/2 도입 및 Tier 3 증설 투자 결정 |
| 개인 제조 창업자 | Web 3D layout 구성 및 Tier 3 Turnkey 의뢰 |
| 현장 실무자 | count correction, alert 확인, work instruction, maintenance order, Process Pause |
| SF-Twin Robotics Specialist | Tier 3 consultation, LiDAR survey, Isaac Sim validation, deployment, FAT/SAT |
| SF-Twin Platform Engine | Web 3D, bridge synthesis, arbitration, MLOps, MuJoCo workers, twin synchronization |

## 5. Global Product Constraints

### GPC-01 — Safety Authority Separation

- Process Pause는 operational stop이다.
- Vision safety monitoring은 advisory-only이다.
- software/wireless 기능은 legal protective device 또는 E-Stop을 대체해서는 안 된다.
- Tier 3 legal Emergency Stop authority는 Hardwired Dual-Channel E-Stop safety relay circuit이 소유한다.

### GPC-02 — Asset Identity Continuity

Asset identity와 canonical metadata는 Brownfield → Greenfield lifecycle 전환 시 유지되어야 한다.

설비 asset metadata는 IDTA AAS Metamodel V3.0에 정합되도록 관리해야 한다.

## 6. Project / Capability Code Registry

이 표는 SF-Twin product/design document ID와 requirement ID에서 사용하는 canonical abbreviation registry다.

새 code는 임의로 추가하지 않는다.
기존 code의 의미를 변경하거나 다른 의미로 재사용해서는 안 된다.

| Category | Code | Meaning | Scope |
|---|---|---|---|
| Project | `SF-Twin` | Smart Factory Twin | 전체 프로젝트 |
| Product Entry | `WEB` | Web Concept Design | Top of Funnel |
| Product Track | `BF` | Brownfield | 기존 생산 라인 |
| Capability | `BF-LITE` | Brownfield Lite | Tier 1 |
| Capability | `BF-STD` | Brownfield Standard | Tier 2 |
| Product Track | `GF` | Greenfield | 신규 무인 line / cell |
| Capability | `GF-D1` | Greenfield Day-1 | Engineering / Turnkey |
| Capability | `GF-D2` | Greenfield Day-2 | Autonomous Operation |
| Engineering Scope | `ARM` | Arm Cell | robot arm workcell |
| Component | `VISION` | Vision | sensing / perception |
| Component | `MOTION` | Motion | motion planning / execution |
| Component | `ORCH` | Orchestration | Behavior Tree / workflow coordination |
| Component | `SAFETY` | Safety | safety control boundary |
| Integration | `VDA` | VDA 5050 Integration | AMR mission / logistics interface |

## 7. Identifier Examples

### Requirement IDs

- `UC-GF-D1-01`
- `FR-GF-D1-03`
- `NFR-GF-D2-04`

### Document IDs

- `[SF-Twin]_PRD-CAP-GF-D1_v1.0.0`
- `[SF-Twin]_CDS-ARM-VISION_v1.0.0`
- `[SF-Twin]_FDS-ARM-ORCH_v1.0.0`
- `[SF-Twin]_ICD-ARM-VDA_v1.0.0`

## 8. Capability Map

| Capability | Document |
|---|---|
| `WEB` | `capabilities/web-concept-design.md` |
| `BF-LITE` | `capabilities/brownfield-lite.md` |
| `BF-STD` | `capabilities/brownfield-standard.md` |
| `GF-D1` | `capabilities/greenfield-day1.md` |
| `GF-D2` | `capabilities/greenfield-day2.md` |
