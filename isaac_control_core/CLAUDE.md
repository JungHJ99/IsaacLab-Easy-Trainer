# isaac_control_core - Project Guide

MoveIt/cuRobo 등 모션 플래너에 독립적인 공통 추상화 레이어.
로봇 설정, 모션 컨트롤러 인터페이스, 태스크, 스킬을 제공한다.

## Architecture

```
isaac_control_core/
  core/
    robot.py               # RobotConfig, GripperConfig (ABC/dataclass)
    motion_controller.py   # MotionController (ABC - 모든 플래너 공통 인터페이스)
    task.py                # BaseTask (validate -> execute -> evaluate)
  robots/
    piper.py               # PiperConfig (6DOF + gripper)
    piper_v100.py          # PiperV100Config (ee_link=link6, gripper 0~0.10)
    franka.py              # FrankaConfig (7DOF + 2-finger)
  tasks/
    pick_and_place.py      # PickAndPlaceTask
    stack_cube.py          # StackCubeTask
    motor_in_box.py        # MotorInBoxTask
  utils/
    skills.py              # RobotSkills (모션 프리미티브 라이브러리)
```

## 사용하는 패키지

이 패키지는 symlink + Docker volume mount로 양쪽 워크스페이스에 공유됨:
- `moveit/ws/src/isaac_control_core` -> symlink -> `isaac_control_core/`
- `curobo/ws/src/isaac_control_core` -> symlink -> `isaac_control_core/`

Docker에서는 `docker_start.sh`가 별도 volume으로 마운트.

## MotionController ABC

모든 모션 플래너가 구현해야 하는 인터페이스:

```python
class MotionController(Node, ABC):
    # 공통 구현
    wait_for_ready()              # _wait_for_planner() + joint_states + objects
    get_current_ee_orientation()  # TF 조회
    _get_arm_positions()          # 현재 arm 관절 위치

    # 서브클래스 구현 필수
    _wait_for_planner()           # 플래너별 서버 대기
    move_to_joint(dict) -> bool
    move_to_pose(x,y,z, ox,oy,oz,ow) -> bool
    move_linear(x,y,z, ox,oy,oz,ow) -> bool
    set_gripper(width) -> bool
```

구현체:
- `MoveItController` (isaac_robot_control) - MoveGroup + Cartesian
- `CuroboController` (curobo_control) - cuRobo MotionGen

### object_orientations property

- `MotionController`는 마커에서 수신한 오브젝트 orientation (quaternion xyzw)을 저장
- `object_orientations` property로 딕셔너리 형태로 조회 가능
- `ObjectManager`가 마커 발행 시 실제 오브젝트 orientation 포함
  - `inv_base_tf * world_tf`로 계산한 상대 orientation (hardcoded `w=1.0` 아님)

## RobotSkills

MotionController를 감싸는 모션 프리미티브. 어떤 플래너든 동일하게 동작:
- `pick(x,y,z)` → approach → descend → close_gripper → lift
- `place(x,y,z)` → approach → descend → open_gripper → retreat
- `place_on_object(held, target)` → EE-오브젝트 오프셋 보정 place

### RobotSkills 상세

#### `pick` / `place` orientation 전달
- `descend`, `lift`, `retreat` 는 `pick`/`place` 로부터 `**ori` (grasp_orientation) 를 받아 사용
- TF orientation drift 방지: 각 단계에서 TF를 새로 읽지 않고 처음 계산한 orientation을 그대로 유지

#### `orientation_weight` 파라미터 제거
- `pick`, `place`, `place_on_object` 에서 `orientation_weight` 파라미터가 **제거됨**
- cuRobo는 `rotation_threshold`로 목표 도달을 판정하므로 별도 weight 불필요

#### `_hold()` 메서드
- `time.sleep()` 대신 사용하는 대기 메서드
- 컨트롤러에 `hold_position()` 메서드가 있으면 해당 메서드 호출 (현재 위치 유지)
- 없으면 `time.sleep()` fallback
- 모션 사이 대기 시간에 물리 시뮬레이션이 로봇을 흔들리게 방치하지 않음

#### `place_on_object(held, target)`
- TF를 사용하여 **EE→held object** 오프셋을 계산
- 이 오프셋을 target object 위치에 적용하여 정확한 place 위치 결정
- **중요**: `ee_link`가 `ee_frame`과 일치해야 오프셋이 정확함
  - Piper + cuRobo의 경우 ee_link="gripper_base" 사용

## Tasks

BaseTask 서브클래스. `run()` = validate → execute → evaluate:
- `PickAndPlaceTask` - pick → place, 평가: XY<8cm, Z>0
- `StackCubeTask` - pick → place_on_object, 평가: XY<5cm, Z>0
- `MotorInBoxTask` - pick → place_on_object, 평가: XY<8cm, Z<3cm (박스 안)

### Tasks 상세

#### 오브젝트 위치 읽기 전 sync_state
- `MotorInBoxTask`, `PickAndPlaceTask` 에서 오브젝트 위치를 읽기 전 `controller.sync_state(30)` 호출
- stale 데이터 방지: 시뮬레이션에서 물체 위치가 업데이트되기 전에 읽으면 오래된 위치를 사용하게 됨
- `sync_state`는 단순 `time.sleep` — background executor가 콜백을 처리하므로 명시적 spin 불필요

#### PickAndPlaceTask — align_orientation 파라미터

`align_orientation=True` 시 마커 orientation에서 오브젝트의 yaw를 읽어 그리퍼 방향 정렬:

- **Yaw 추출 방법**: 오브젝트 로컬 X축을 XY 평면에 투영 → `atan2`로 yaw 계산
- **Yaw 정규화**: `[-90°, 90°]` 범위로 normalize (평행 그리퍼 대칭성 반영)
- **적용 방식**: `R.from_euler('YZ', [pi, -obj_yaw])` (top-down 그리퍼 Z 뒤집힘 보정을 위해 부호 반전)
- `pick()` 메서드에 `ox, oy, oz, ow` orientation override 파라미터 추가

## Domain Randomization

### `add_camera` — position_delta / orientation_delta / focal_length
- `position_range` / `orientation_range` → **`position_delta` / `orientation_delta`** 로 변경
  - delta 방식: 기준 위치(position)로부터 ±offset 범위 내에서 랜덤 샘플링
- `focal_length` 파라미터 추가: 5=ultra-wide, 10=wide, 24=normal, 50=telephoto

### `set_lighting_randomization()`
- 조명 도메인 랜덤화 메서드
- 파라미터: `intensity_range` (밝기 범위), `color_temp_range` (색온도 범위), `additional_lights` (추가 광원)
- 매 에피소드마다 호출하여 조명 조건 다양화

### `set_background_randomization()`
- DomeLight 색상 (grayscale) 랜덤화 + texture 제거
- Custom `BackgroundFloor`에 OmniPBR material 사용 (기본 ground_plane 대체)
  - `DisplayColorAttr`은 RTX renderer에서 동작하지 않으므로 OmniPBR 머티리얼 필수

## Piper 그리퍼 값 매핑

로봇별 gripper 스케일:
- `piper`: 실제 0~0.085 ↔ 시뮬레이션 0~0.035
- `piper_v100`: 실제 0~0.10 ↔ 시뮬레이션 0~0.05

`PiperConfig`의 `open_width`는 실제 로봇 스케일.
변환은 `GripperScaleBridge`(MoveIt/service_runner) 또는 `CuroboController`에서 처리.

### GripperScaleBridge (service_runner.py)

시뮬 ↔ 실제 gripper 스케일 변환을 ROS 토픽 레벨에서 처리하는 클래스:
- 구독: `/simulation/joint_states` → republish to `/joint_states` (real 스케일)
- 구독: `/simulation/joint_command` → republish to `/joint_command` (real 스케일)
- **단일 책임**: `/joint_states` 외부 발행은 GripperScaleBridge가 전담
  - `CuroboController._joint_state_cb`는 내부 `_current_joint_state` 갱신만 담당, 외부 발행 X
  - 이전에는 양쪽에서 중복 발행하여 콜백 레이턴시가 늘어남

## ROS2 Spin 아키텍처 (service_runner.py)

### 핵심 원칙: 노드별 dedicated background spin

- `service_node`와 `controller`는 **각각 별도의 `SingleThreadedExecutor`**에서 background spin
- 같은 default executor를 공유하면 `ValueError: generator already executing` 발생
- `MultiThreadedExecutor`로 두 노드를 묶는 방식도 가능하지만, 노드별 SingleThreadedExecutor가 더 안정적

```python
service_executor = SingleThreadedExecutor()
service_executor.add_node(service_node)
controller_executor = SingleThreadedExecutor()
controller_executor.add_node(controller)
threading.Thread(target=service_executor.spin, daemon=True).start()
threading.Thread(target=controller_executor.spin, daemon=True).start()
```

### `wait_for_ready()` 호출 순서
- **반드시 background spin 시작 *후*에 호출**해야 함
- 만약 background spin 전에 `spin_once(controller)`를 부르면, 노드가 임시 default executor에 묶여
  이후 background `controller_executor.spin()`이 효과 없음 (콜백이 처리되지 않아 `_current_joint_state` stale)
- entry point (`pick_and_place_service.py` 등)에서 `controller.wait_for_ready()` 호출하지 말 것
- `run_task_service` 내부에서 background spin 시작 후 호출

### `MotionController._cb_group = ReentrantCallbackGroup()`
- 모든 subscription이 ReentrantCallbackGroup에 묶여있음
- joint_state, TF, marker 콜백이 서로를 막지 않고 동시 처리 가능

### `reset_env()` 임시 노드 사용
- controller가 background spin 중이므로 `rclpy.spin_until_future_complete(controller, ...)` 호출 시 generator 충돌
- `reset_env`는 `_reset_env_client` 임시 노드를 만들어 service call 후 destroy
- 임시 노드는 background spin과 분리되어 있어 안전하게 spin_until_future_complete 가능

### controller 코드에서 명시적 spin 금지
- `CuroboController` 내부에서 `rclpy.spin_once(self, ...)` 호출하지 말 것 (background executor와 충돌)
- hold 루프는 `_publish_cmd + time.sleep`만 — 콜백 처리는 background executor가 담당
- `sync_state()`도 `time.sleep(0.1)` 기반 polling일 뿐

## MotionController 플래너별 Property 차이

### `needs_orientation_correction` property
- **MoveIt (`MoveItController`)**: `True` — MoveIt은 orientation 보정이 필요 (Cartesian planning 시)
- **cuRobo (`CuroboController`)**: `False` — cuRobo는 이미 orientation을 포함하여 planning하므로 보정 적용 시 config jump 발생

### `sync_state(n, timeout)` 메서드
- 콜백 처리 대기용 헬퍼 — 단순히 `time.sleep(0.1)` 호출
- background executor가 controller 콜백을 처리한다고 가정 (run_task_service에서 보장)
- 명시적 `spin_once` 호출하지 않음 — executor와 충돌 방지
- `n`, `timeout` 인자는 backward compat용으로 무시됨

### `gripper_length` 설정
- 플래너별 override 없이 **`RobotConfig.gripper_length` 단일 값** 사용
- 플래너마다 다른 값이 필요하면 별도 RobotConfig 서브클래스를 만들 것

## Symlink 구조

프로젝트 루트에 위치한 패키지를 양쪽 워크스페이스에 symlink로 공유:

```
IsaacLab/
  isaac_control_core/                    # 프로젝트 루트 (원본)
  moveit/ws/src/isaac_control_core/      # symlink -> ../../isaac_control_core/
  curobo/ws/src/isaac_control_core/      # symlink -> ../../isaac_control_core/

  ros_pkgs/piper_description/            # 프로젝트 루트 (원본)
  moveit/ws/src/piper_description/       # symlink -> ../../ros_pkgs/piper_description/
  curobo/ws/src/piper_description/       # symlink -> ../../ros_pkgs/piper_description/
```

- `piper_description`은 cuRobo 컨테이너에서 **COLCON_IGNORE** 파일 포함
  - cuRobo는 piper_description을 colcon build하지 않음 (URDF만 직접 참조)
- Docker `docker_start.sh`가 volume mount를 설정하여 컨테이너 내에서도 동일 구조 유지
