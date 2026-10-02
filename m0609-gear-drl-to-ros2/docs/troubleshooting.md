# DRL → ROS 2 Python 변환 트러블슈팅 (M0609 기어 조립)

- 원본: 티치 펜던트(Task Writer)로 만든 `m0609_homework_gear_1` ([`../src/drl/`](../src/drl/): `.drl` + `System.drvar`)
- 변환 결과: [`../src/my_gear.py`](../src/my_gear.py) (ROS 2 Jazzy, `dsr_controller2`, `DSR_ROBOT2` 라이브러리)
- 로봇: Doosan M0609 + OnRobot RG2 (컨트롤러 디지털 출력으로 열고 닫음)
- 작업: 기어 3개 옮기기 → 4번째 기어를 순응·힘제어 + Z축 회전 흔들기로 맞물려 끼우기
- 근거: 2026-09-16 오후 실로봇 실행 19회의 노드 로그와 브링업 터미널 알람([`logs/`](logs/))

펜던트에서 만든 프로그램을 Python으로 옮기자 **그리퍼 타이밍, 힘제어 조건, 홈 복귀**가 모두 어긋났다.
원인은 크게 두 가지였다.

- DRL과 ROS 2 라이브러리에서 **이름이 같은 함수가 다르게 동작**하는 것 (1~7번)
- 디버깅하면서 바꾼 **테스트 값과 편집 실수**가 만든 문제 (8~12번)

> 추정이라고 적은 항목은 로그로 원인을 하나로 확정하지 못한 것이다.

---

## 한눈에 보기

### DRL ↔ ROS 2 Python 차이 (옮기면서 드러난 것)

| 증상 | 원인 | 고친 방법 | 항목 |
|---|---|---|---|
| 그리퍼가 닫히기 전에 팔이 올라감 | ROS 2의 `mwait(time)`은 시간 인자를 쓰지 않고 모션 완료만 기다림(라이브러리 소스 주석으로 확인) | 그리퍼 대기는 `wait(2)` | 2 |
| 조건이 안 맞는데 흔들기가 실행되고 반복문이 바로 끝남 → 원본 `release()`의 50mm 하강으로 바닥 충돌(`Collision occurred at Joint Axis2`, 4회) | `check_force_condition`/`check_position_condition`이 참=0, 거짓=-1을 돌려줌. Python에서 `if -1:`은 참 | `== 0`으로 비교, 힘은 `get_tool_force()` 값으로 직접 판정, 삽입 후엔 하강 없이 그리퍼만 열기 | 4, 5 |
| `set_tool`/`set_tcp`가 -1, 접촉 전 Fz가 약 -12N이고 Z가 약 208mm 높게 나옴 | 이 함수들은 수동 모드에서만 받음(알람 `this command can only be used in manual mode`). 브링업을 다시 켠 뒤 펜던트의 툴/TCP가 적용되지 않은 채 실행된 경우도 4번 있었음(정확한 조건은 미확인) | 시작할 때 수동 → 설정 → 자동. 실패하면 로봇을 움직이지 않고 종료 | 6 |
| 첫 명령에서 응답 없이 멈춤 | 노드가 컨트롤러 서비스를 찾기 전에 보낸 요청이 유실됨 | `wait_for_service` 후 시작 | 7 |
| 집는 위치에 서기 전에 곡선으로 다음 동작 시작 | 원본에서 가져온 블렌딩 반경 `radius=10` | 집기·놓기 구간은 `radius=0` | 3 |
| DRL 인자를 그대로 쓸 수 없음 | ROS 2 `movel`에는 `app_type`이 없고, `set_velx`는 세 번째 인자가 없음 | 함수 정의와 대조해 인자 정리 | 1 |

### 실로봇 디버깅 중에 생긴 문제 (테스트 값, 편집 실수, 힘제어 조합)

| 증상 | 원인 | 고친 방법 | 항목 |
|---|---|---|---|
| 흔들기가 거부되고 힘이 2N 안팎에서 멈춤(브링업 알람 `TASK_MOTION_w_FORCE_CONTROL rejected event[eMove_Periodic]`, 오후 동안 10회) | 원본처럼 힘제어 중 Z 방향 `amovel`을 함께 보내면 컨트롤러가 "이동 중 + 힘제어" 상태로 남아 흔들기를 받지 않음 | `amovel`을 빼고 힘제어만으로 하강 → 이후 거부 없음 | 9 |
| 흔들기가 즉시 반환되고 1초에 약 30번 다시 호출됨(`Acc.Time(0.500) should be smaller than 1/2 of total Time(1.000)`) | 테스트로 넣은 `atime=0.5, repeat=1`이 `atime < period×repeat/2` 규칙 위반 | `atime=0.3, repeat=2`, 흔들기 사이 2.5초 간격 | 10 |
| 목표 -5N인데 약 -3.9N에서 더 커지지 않음 | (추정) 테스트로 힘을 낮추자 `DR_FC_MOD_REL`(켠 시점 측정값 기준 상대 힘)의 영향이 드러남. 이 실행(17:25)은 툴/TCP도 빠져 있어 원인을 확정하지 못함 | `DR_FC_MOD_ABS` | 8 |
| 삽입 후 홈 복귀 거부(`TASK_FORCE_CONTROL rejected event[eMoveJ]`), 그런데 노드는 "작업 완료" 출력 | 코드를 고치다 `release_force`/`release_compliance_ctrl` 호출이 빠짐 | 그리퍼 열기 → 힘제어 해제 → 순응제어 해제 → 상승 → 홈 | 11 |

---

## 원본 DRL과 최종 Python 비교

| 항목 | 원본 DRL | 최종 Python (18:36 성공 버전) |
|---|---|---|
| 그리퍼 대기 | `mwait(time=0.25)` | `wait(2)` |
| 집기·놓기 구간 블렌딩 반경 | 10 | 0 |
| 힘 조건 판정 | `if check_force_condition(..., min=7, max=12)` | `3.0 <= abs(get_tool_force()[2]) <= 8.0` |
| 삽입 판정 | `if check_position_condition(...)` | `check_position_condition(...) == 0` (Z ≤ 80mm) |
| 목표 힘 | -10N, `DR_FC_MOD_REL` | -5N, `DR_FC_MOD_ABS` |
| 힘제어 중 하강 | `amovel` -20mm + 힘제어 | 힘제어만 |
| 흔들기 | `atime=1.0, repeat=3` | `atime=0.3, repeat=2`, 2.5초 간격 |
| 삽입 후 | `release()` (50mm 하강 포함) | 그리퍼만 열기 |
| 반복문 | `while True` | 15초 시간 제한 (`while ... else`로 시간 초과 기록) |
| 툴/TCP | 펜던트 선택 사용 | 스크립트 시작 시 직접 설정, 실패하면 중단 |
| 시작 전 | - | 컨트롤러 서비스 대기 |

최종 값 가운데 목표 힘, 판정 범위, `atime/repeat`는 디버깅 중 낮춘 값이다. 원본 값으로 되돌려 다시 검증하지는 않았다.

---

## 실행 기록 요약 (2026-09-16, KST)

| 시각 | 결과 | 근거 |
|---|---|---|
| 16:16 ~ 16:45 | 4번 기어에서 반복문이 바로 끝나고 `release()` 하강으로 바닥 충돌 (4회) | 브링업 `Collision occurred at Joint Axis2` |
| 17:14 ~ 18:05 | 툴/TCP가 빠진 실행에서 접촉 전 Fz 약 -12N, Z 약 294mm | 노드 로그 |
| 17:26 ~ 17:55 | 흔들기가 즉시 반환되며 반복 호출 | 브링업 `Acc.Time ...` 341줄 |
| 17:15 ~ 18:27 | 흔들기 거부. 툴/TCP가 정상인 실행(17:51~18:27)에서도 힘이 3~4N까지 올랐다가 2N 안팎으로 떨어지고 Z가 84mm대에서 멈춰 15초 시간 초과 | 브링업 `rejected event[eMove_Periodic]`, 노드 로그 |
| 18:23, 18:24 | 툴/TCP 설정 실패를 감지하고 로봇을 움직이지 않고 종료 | 노드 로그 |
| 18:33 | 삽입 성공(Z 79.9mm), 홈 복귀 거부 | 브링업 `rejected event[eMoveJ]` |
| **18:36** | **기어 4개 전체 성공, 홈 복귀. 알람 없음** | [`run_20260916_1836_full_success.log`](logs/run_20260916_1836_full_success.log) |
| 19:03 | 재실행. 로봇 네트워크가 끊긴 상태(18:54)라 진행되지 않아 중단 | 브링업 `Network is unreachable` |

---

## 1. 단순 API 차이 (옮길 때 바로 고칠 것)

ROS 2 라이브러리(`DSR_ROBOT2.py`)의 함수 정의와 대조해서 고쳤다.

| DRL | ROS 2 Python | 비고 |
|---|---|---|
| `movel(..., app_type=DR_MV_APP_NONE)` | `app_type` 인자 삭제 | ROS 2 `movel`에는 이 인자가 없다 |
| `set_velx(500, 101.25, DR_OFF)` | `set_velx(500, 101.25)` | 세 번째 인자 없음 |
| `ra=DR_MV_RA_DUPLICATE` | 생략 (기본값) | |
| `ref=0`, `ref=1` | `ref=DR_BASE`, `ref=DR_TOOL` | 숫자도 되지만 상수가 읽기 쉽다 |
| `System_gj1` (drvar 변수) | `gj1 = posj(...)` 로 직접 정의 | `System.drvar` 값을 그대로 옮김 |
| `_alias_sub_name_grasp()` | `grasp()` | |
| `while gLoop < 1:` | 삭제 (1회 실행) | |
| 툴/TCP 펜던트 선택 | 스크립트에서 직접 설정 | 6번 참고 |

---

## 2. `mwait`가 시간을 기다리지 않음 → 그리퍼가 닫히기 전에 팔이 올라감

- **증상**: 내려가서 그리퍼 신호를 보낸 직후 바로 올라감. 올라가는 중에 그리퍼가 닫혀 순서가 바뀐 것처럼 보임.
- **원인**: ROS 2용 `mwait(time)`은 **시간 인자를 쓰지 않는다**(라이브러리 소스 주석: "ROS 에서는 time 인자를 사용하지 않음"). 앞 모션이 끝났는지만 기다리는데, ROS 2의 `movel`은 동작이 끝나야 반환하므로 사실상 기다리는 시간이 없다.
- **해결**: 그리퍼처럼 팔 밖의 장치가 움직일 시간은 `wait(초)`(내부적으로 `time.sleep`)로 준다. 이 실습에서는 2초를 줬다.

```python
set_digital_output(1, ON)
set_digital_output(2, OFF)
wait(2)            # 원본: mwait(time=0.25)
```

- **정리**: `mwait` = 모션 완료 대기, `wait` = 시간 대기. ROS 2에서는 `movel/movej`가 끝날 때까지 반환하지 않으므로 `mwait`가 필요한 곳은 `amovel` 같은 비동기 모션 뒤뿐이다.

---

## 3. 블렌딩 반경 때문에 목표점에 멈추지 않음

- **증상**: 집는 위치에 도착하기 전에 곡선을 그리며 다음 동작(하강)으로 넘어감.
- **원인**: 원본에서 그대로 가져온 `radius=10`은 "목표점 10mm 앞에서 다음 동작과 이어라"는 뜻이다. 집기·놓기 직전에는 정확히 멈춰야 한다.
- **해결**: 집기·놓기 직전 `movej`와 그 안의 하강 `movel`은 `radius=0`.

---

## 4. `check_force_condition` / `check_position_condition` 반환값이 DRL과 다름

- **증상**: 조건이 안 맞는데 흔들기가 실행되고, 조건이 맞기도 전에 반복문이 `break`됨. 반복문이 곧바로 끝나고 다음 동작으로 넘어가 바닥에 충돌(5번).
- **원인**: ROS 2 라이브러리는 **조건이 참이면 0, 거짓이면 -1**을 돌려준다. Python에서 `if -1:`은 참이라 `if check_force_condition(...):`이 조건이 안 맞을 때 실행된다.
- **해결**: `== 0`으로 비교한다. 힘 조건은 `get_tool_force()` 값을 Python에서 직접 비교했다. 부호 문제가 없고, 로그에 찍는 값과 판정 기준이 같아 확인하기 쉽다.

```python
def force_in_range(fz):
    return 3.0 <= abs(fz) <= 8.0      # 원본 DRL은 7~12N (8번 참고)

def inserted():
    return check_position_condition(DR_AXIS_Z, max=80, ref=DR_BASE, mod=DR_MV_MOD_ABS, pos=None) == 0
```

---

## 5. 4번 문제로 원본 `release()`의 하강이 그대로 실행되어 충돌

- **증상**: 4번 기어에서 로봇이 빠르게 내려가 바닥에 부딪히고 그 자리에서 그리퍼를 놓음. 충돌 감지로 정지(`Collision occurred at Joint Axis2`, 16:16~16:45 사이 4회).
- **원인**: 원본 `release()`는 50mm 하강 → 놓기 → 50mm 상승 구조다. 4번 문제로 반복문이 바로 끝나자, 기어가 아직 끼워지지 않은 높이에서 `release()`의 50mm 하강이 실제로 실행됐다.
  - (추정) 펜던트에서는 이 시점에 힘제어가 Z축을 잡고 있어 하강이 그대로 일어나지 않았을 것으로 보인다. 펜던트 실행과 비교해 확인하지는 못했다.
- **해결**: 삽입 후에는 하강 없이 그리퍼만 연다(`set_digital_output` 두 줄 + `wait`). 힘제어에 기대어 "무시되기를 바라는 이동"은 남기지 않는다.

---

## 6. 툴 무게 / TCP 설정이 풀려 힘 값과 Z 좌표가 틀어짐

- **증상 A**: `set_tool`/`set_tcp`가 -1을 돌려주거나 응답 없이 멈춤. 브링업 로그에 `this command can only be used in manual mode`.
- **증상 B**: 아무것도 닿지 않았는데 `get_tool_force()` Z가 약 -12N(그리퍼 무게), `get_current_posx()` Z가 정상(약 86mm)보다 약 208mm 높은 약 294mm로 나옴(플랜지 기준). 힘제어가 위로 올라가고 "Z ≤ 80mm" 조건이 절대 만족되지 않음.
- **원인**:
  - `set_tool`, `set_tcp`는 **수동 모드에서만** 받는다. 브링업이 로봇을 자동 모드로 바꾸므로 그냥 부르면 거부되거나 응답이 없다.
  - 브링업을 다시 켠 직후 툴/TCP가 적용되지 않은 채 실행된 경우가 4번 있었다(17:14, 17:25, 17:47, 18:05). 16:44 실행처럼 브링업을 다시 켜도 정상인 경우가 있어, 정확히 어떤 조건에서 풀리는지는 확인하지 못했다. 그래서 스크립트에서 매번 다시 설정하기로 했다.
  - 이름이 펜던트 등록 이름과 글자 하나까지 같아야 한다(`"Tool Weight_1"` ✗ → `"Tool Weight"` ✓).
- **해결**: 스크립트 시작 시 수동 모드 → 설정 → 자동 모드. 실패하면 로봇을 움직이지 않고 종료한다(18:23, 18:24 실행에서 이 안전장치가 동작함).

```python
set_robot_mode(ROBOT_MODE_MANUAL)
rt = set_tool("Tool Weight")
rc = set_tcp("GripperDA_v1")
set_robot_mode(ROBOT_MODE_AUTONOMOUS)
if rt != 0 or rc != 0:
    return
```

- **이름 확인 방법** (펜던트에서 고른 뒤):

```bash
ros2 service call /dsr01/dsr_controller2/tool/get_current_tool dsr_msgs2/srv/GetCurrentTool
```

- **정상 판정**: 접촉 전 Fz ±2N 이내, 4번 기어 위 Z 약 86mm. 18:36 성공 실행은 Fz 1.2N, Z 86.4mm였다.

---

## 7. 첫 명령이 유실되어 스크립트가 멈춤

- **증상**: 시작 로그 뒤 아무 출력 없이 멈춤. Ctrl+C 하면 첫 로봇 명령의 `spin_until_future_complete`에서 멈춰 있음. 브링업과 컨트롤러는 정상(`get_robot_state` 서비스 응답 있음).
- **원인**: 라이브러리는 import 직후 바로 명령을 보낼 수 있는데, 노드와 컨트롤러가 서로를 발견하기 전에 보낸 요청은 버려지고 응답을 계속 기다린다. 파일 맨 위의 `time.sleep`은 노드 생성 전이라 효과가 없다.
- **해결**: import 직후, 첫 명령 전에 서비스가 보일 때까지 기다린다.

```python
from dsr_msgs2.srv import SetSingularityHandling
probe = node.create_client(SetSingularityHandling, "dsr_controller2/motion/set_singularity_handling")
while not probe.wait_for_service(timeout_sec=1.0):
    node.get_logger().info("컨트롤러 서비스 대기 중...")
time.sleep(1.0)
```

---

## 8. 목표 힘을 낮추자 `DR_FC_MOD_REL`의 영향이 드러남 (추정)

- **배경**: 디버깅 중 안전을 위해 목표 힘을 -10N → -5N, 판정 범위를 7~12N → 3~8N으로 낮췄다.
- **증상**: 목표 -5N인데 측정값이 약 -3.9N에서 더 커지지 않음(17:25 실행).
- **원인 (추정)**: REL 모드는 힘제어를 켠 시점의 측정값을 기준으로 한 상대 힘이라, 시작 시점 센서값만큼 실제 목표가 어긋난다. 원본처럼 -10N에 7~12N이면 여유가 있어 드러나지 않지만, 값을 줄이니 판정 경계에 걸렸다. 이 실행은 툴/TCP 설정도 빠져 있어 원인을 하나로 확정하지는 못했다.
- **해결**: `mod=DR_FC_MOD_ABS` (절대 기준).

---

## 9. 힘제어 중 `amovel`이 함께 걸려 흔들기 거부 → 힘이 2N 안팎에서 정체

- **증상**: 힘이 3N 근처까지 오르다가 흔들기 호출 직후 2N 안팎으로 떨어지고 Z가 멈춤. 15초 시간 초과. 브링업 로그: `state[TASK_MOTION_w_FORCE_CONTROL] rejected event[eMove_Periodic]` (모두 10회).
- **원인**: 원본 DRL은 힘제어를 켠 뒤 Z 방향 `amovel` -20mm를 함께 보낸다. 컨트롤러 상태가 "이동 중 + 힘제어"(`TASK_MOTION_w_FORCE_CONTROL`)인 동안에는 `move_periodic`을 받지 않는다(알람으로 확인). 힘제어가 Z를 잡고 있어 `amovel`이 끝나지 않고 이 상태가 계속된 것으로 보인다. 디버깅 중 이 `amovel`을 50mm/s로 낮춘 뒤에도 같은 거부가 계속됐다.
  - 첫 실행(16:16, 원본과 같은 1000mm/s)에서도 같은 알람이 있었다. 이때는 4번 문제로 흔들기가 곧바로 호출된 경우라, 펜던트에서 이 구간이 어떻게 처리되는지는 확인하지 못했다.
- **해결**: `amovel`과 그 뒤 `mwait()`를 지우고 힘제어만으로 닿을 때까지 내려간다. 18:33 실행부터 거부가 없어지고 흔들기 1회 만에 삽입됐다.

---

## 10. `move_periodic` 파라미터 규칙 위반으로 즉시 반환

- **증상**: 흔들기가 움직이지 않고 바로 반환되고, 반복문이 흔들기를 1초에 약 30번 다시 호출. Python 반환값은 0(성공)이라 알아채기 어려움. 브링업 로그: `Acc.Time(0.500) should be smaller than 1/2 of total Time(1.000)` (모두 341줄).
- **원인**: `atime < (period × repeat) / 2` 이어야 한다. 테스트로 `atime=0.5, repeat=1`로 줄이면서 위반했다.
- **해결**: 최종 코드는 `atime=0.3, repeat=2` (0.3 < 1.0)이고, 흔들기 사이에 2.5초 간격을 둔다. 원본 값 `atime=1.0, period=1, repeat=3`도 규칙(1.0 < 1.5)은 지키지만 Python으로 실행해 보지는 않았다.
- **교훈**: Python 반환값 0은 "서비스 호출 성공"이지 "컨트롤러가 모션을 받아들임"이 아니다. 모션 거부는 **브링업 터미널의 `OnLogAlarm`** 에만 나온다.

---

## 11. 힘제어 해제 누락으로 홈 복귀 거부

- **증상**: 삽입까지 되는데 마지막 50mm 상승과 홈 `movej`가 안 됨(18:33 실행). 브링업 로그: `state[TASK_FORCE_CONTROL] rejected event[eMoveJ]`. 노드 로그에는 그래도 "작업 완료"가 찍힘.
- **원인**: 코드를 고치는 중에 `release_force()`, `release_compliance_ctrl()` 두 줄이 빠져 `finally`에만 남았다. 힘제어가 켜진 채로 위치 이동을 보내면 컨트롤러가 받지 않는다.
- **해결**: 그리퍼 열기 → `release_force` → `release_compliance_ctrl` → 상승 → 홈. `finally`의 해제는 Ctrl+C로 멈췄을 때를 대비해 그대로 둔다. 이렇게 고친 18:36 실행에서 전체 동작이 끝까지 성공했다.

---

## 12. 빌드 방식 때문에 수정이 반영되지 않음

- **증상**: 소스를 고쳤는데 실행 결과가 그대로. 새로 넣은 로그 줄이 안 나옴.
- **원인**: `colcon build`를 `--symlink-install` 없이 하면 install 폴더에 그 시점 **복사본**이 들어가고, 이후 소스 수정이 반영되지 않는다.
- **해결**: `colcon build --symlink-install --packages-select <패키지명>`. 확인: `readlink -f build/<패키지명>/<패키지명>/my_gear.py`가 src 경로를 가리켜야 한다.

---

## 13. 코드와 무관했던 것

- 브링업 시작 로그의 `pymodbus` 에러: OnRobot 모드버스 그리퍼 노드의 에러. 이 실습은 디지털 출력으로 그리퍼를 다뤄서 무관했다.
- 브링업 `Timeout → Disconnected → Network is unreachable`: 로봇과의 이더넷 연결이 끊긴 것. 케이블과 네트워크 설정을 확인한다.
- 스크립트를 실행하는 동안 브링업 터미널은 **계속 떠 있어야** 한다(끄고 실행하면 첫 명령에서 멈춤).

---

## 진단 방법 정리

| 보고 싶은 것 | 방법 |
|---|---|
| 툴 무게 보정, TCP | 힘제어 전 `get_tool_force()`, `get_current_posx()` 로그. Fz ±2N, Z가 티칭 높이 근처면 정상 |
| 설정 함수 성공 여부 | 반환값 로그 (0 성공, -1 실패). `set_tool`, `task_compliance_ctrl`, `set_desired_force` 등 |
| 컨트롤러가 모션을 거부했는지 | **브링업 터미널**의 `[callback OnLogAlarm]` 아래 `param :` 줄 |
| 힘제어 동작 | 반복문에서 0.1초마다 `Fz`, `Z` 로그. 로그 간격이 벌어지면 동기 모션(흔들기) 실행 중 |
| 컨트롤러 생존 | `ros2 service call /dsr01/dsr_controller2/system/get_robot_state dsr_msgs2/srv/GetRobotState` (1 = STANDBY) |
| 반복문 종료 사유 | `while ... else:`로 시간 초과와 조건 만족을 구분해 로그 |

---

## 최종 힘제어 구간 구조 (18:36 성공 버전)

```python
task_compliance_ctrl()
set_stiffnessx([3000.0, 3000.0, 3000.0, 200.0, 200.0, 200.0], time=0.0)
set_desired_force([0.0, 0.0, -5.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)
wait(0.5)                                   # amovel 없음, 힘제어만으로 하강

t0, last_periodic = time.time(), 0.0
while time.time() - t0 < 15.0:
    fz = get_tool_force()[2]
    if inserted():                          # Z <= 80mm
        break
    if force_in_range(fz) and time.time() - last_periodic > 2.5:    # |Fz| 3~8N
        move_periodic(amp=[0, 0, 0, 0, 0, 10], period=[0, 0, 0, 0, 0, 1], atime=0.3, repeat=2, ref=DR_TOOL)
        last_periodic = time.time()
        continue
    wait(0.1)
else:
    node.get_logger().info("15초 시간 초과")

set_digital_output(1, OFF); set_digital_output(2, ON); wait(2)    # 그리퍼만 열기
release_force(time=0.0)
release_compliance_ctrl()
movel(posx(0, 0, 50, 0, 0, 0), ...)
movej(home, ...)
```

18:36 실행에서는 힘제어를 켠 뒤 약 1.4초 만에 |Fz| 3.0N에 도달해 흔들기 1회를 했고, Z가 85.6mm → 79.9mm로 내려가 약 3.9초 만에 삽입으로 판정했다.
