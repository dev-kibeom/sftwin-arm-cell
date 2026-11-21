# PLC

`common/`에는 셀의 설비·자재·공정 단계를 몰라도 사용할 수 있는 통신 기능을 둔다. 각 셀의 폴더는 이 디렉터리 바로 아래에 있으며 ROS bringup의 셀 프로필과 같은 이름을 사용한다.

| 위치 | 용도 |
|---|---|
| [common/commissioning](common/commissioning/PLC_PRG.prg.st) | Modbus 요청 반향과 연결 중단을 시험하는 ST 프로그램. 식별값은 IR7의 `0xC04E`이며 HR6·HR7은 시험 입력이다. [시운전 절차](../../ros2_ws/src/plc_adapter/README.md#codesys-4-로컬-실통신-시험) |
| [conveyor_pnp_cell](conveyor_pnp_cell/README.md) | 런처가 여는 Conveyor 공정 프로젝트와 CODESYS에서 열어 비교하는 [실패 사례 PLC](conveyor_pnp_cell/README.md#실패-사례-plc) |
| [isaac_pnp_cnc](isaac_pnp_cnc/README.md) | CNC 자재 공급 및 셀 설정 위치 |
| [tape_roll_cell](tape_roll_cell/README.md) | 테이프 공급 및 셀 설정 위치 |
