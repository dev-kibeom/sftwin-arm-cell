# [SF-Twin] Safety and Control Architecture

- **Document ID:** `[SF-Twin]_ARCH-SAFETY-CONTROL_v1.1.0`
- **Document Type:** Architecture
- **Architecture Scope:** SYSTEM
- **Version:** 1.1.0
- **Status:** Draft
- **Owner:** Architecture Authority
- **Source:** `[SF-Twin]_SAD_v13.0` decomposition
- **Related Product Definition:** `[SF-Twin]_PRD-DEFINITION_v1.0.0`
- **Related Capabilities:** `BF-LITE`, `GF-D1`, `GF-D2`

## 1. Purpose

Physical safety, advisory safety observation, operational pause,
adaptive control validation, correction acceptance, fallback의
system-level authority와 boundary를 정의한다.

## 2. Safety Authority Model

### 2.1 Legal E-Stop

`GPC-01`에 따라 Hardwired Dual-Channel safety relay가 legal Emergency Stop의 physical authority를 가진다.

- software/network path를 경유하지 않고 safety/power/STO path에 작용한다.
- target relay interruption ≤ 100 ms
- Tier 3 safety implementation은 ISO 13849-1 Cat.3/PL d 이상과 IEC 60204-1 요구에 정합되어야 한다.
- Edge software는 auxiliary/observation path를 통해 E-Stop state를 관찰하고 전파할 수 있다.

### 2.2 Software Process Pause

Tablet/UI의 stop 기능은 legal E-Stop이 아니라 Process Pause다.

- local operational-control path를 사용한다.
- PackML / Edge runtime coordination에 반영한다.
- BF-LITE target signal delivery ≤ 500 ms

### 2.3 Vision Safety Monitoring

Vision-based safety monitoring은 advisory-only다.

- alarm / beacon / operator notification을 생성할 수 있다.
- legal protective device나 E-Stop authority를 가져서는 안 된다.
- Vision detection failure가 hardwired safety protection을 대체하거나 약화해서는 안 된다.

## 3. Adaptive Control Authority

Cloud는 Day-2 heavy optimization / Batch SysID를 수행할 수 있다.

Cloud output은 correction/physical-parameter **proposal**이며
runtime control에 직접 canonical하게 적용되는 command가 아니다.

Edge는 current local state와 Local Sanity Guard를 사용해 proposal acceptance를 결정한다.

## 4. Local Sanity Guard

Cloud-originated correction/parameter proposal은 local validation을 통과해야 한다.

Architecture-level validation dimensions:

- proposal freshness / computation age
- bounded correction magnitude
- current local state
- applicable operating/safety tolerance

Product-defined hard bounds:

- per-update origin correction ≤ ±0.05 mm
- joint-angle error bound ≤ ±0.5°
- torque correction bound ≤ ±5%

Guard validation 실패 시:

1. proposal을 Reject한다.
2. invalid correction을 runtime control에 적용하지 않는다.
3. Ramp-down Safe Mode로 전환한다.

## 5. Graceful Fallback / Ramp-down

다음 condition은 local fallback 판단을 유발할 수 있다.

- Cloud timeout
- malformed/invalid optimization result
- Sanity Guard violation
- required dependency loss

Sanity Guard violation의 canonical behavior는 `Reject → Ramp-down Safe Mode`다.

Ramp-down의 exact state transition, duration, recovery criteria는 FDS가 소유한다.

## 6. Authority Matrix

| Concern | Canonical Authority |
|---|---|
| Legal E-Stop actuation | Hardwired Safety Relay |
| E-Stop software observation | Edge software |
| Vision safety alert | Advisory Vision / Operator notification |
| Process Pause request/execution | Local operational-control path |
| PackML operational state | Edge Local Master |
| Cloud optimization computation | Cloud calculation authority only |
| Correction acceptance | Edge Local Sanity Guard |
| Local runtime control application | Edge control |
| Cloud operational state | Read Replica |

## 7. Safety / Control Invariants

- software stop을 legal E-Stop으로 표현해서는 안 된다.
- Vision advisory를 legal protective device로 표현해서는 안 된다.
- hardwired E-Stop path는 Cloud/Edge network availability에 의존해서는 안 된다.
- Cloud correction은 Local Sanity Guard를 우회해서는 안 된다.
- rejected correction은 runtime control에 적용되어서는 안 된다.
- Sanity Guard violation은 uncontrolled full-speed continuation으로 이어져서는 안 된다.
- Sanity Guard violation은 Ramp-down Safe Mode로 이어져야 한다.
- physical safety event의 software propagation failure가 physical safety actuation을 방해해서는 안 된다.

## 8. Dynamic Design Boundary

다음 상세 sequence는 FDS가 소유한다.

- Process Pause sequence
- E-Stop observation/propagation
- Vision advisory reaction
- adaptive optimization request/response
- correction acceptance/rejection
- Ramp-down transition/recovery
- dependency-loss fallback

## 9. Product Traceability

- `GPC-01`
- `FR-BF-LITE-03`, `FR-BF-LITE-04`
- `NFR-BF-LITE-06`
- `FR-GF-D1-04`
- `NFR-GF-D1-04`, `NFR-GF-D1-05`, `NFR-GF-D1-07`
- `FR-GF-D2-01`, `FR-GF-D2-03`
- `NFR-GF-D2-02`, `NFR-GF-D2-05`

## 10. Open Decisions

- Ramp-down exact duration, allowed intermediate states, recovery/acknowledgement semantics는 Arm-cell / Edge FDS에서 확정한다.
- correction freshness threshold의 exact value는 downstream control design과 Verification Requirement에서 확정한다.
