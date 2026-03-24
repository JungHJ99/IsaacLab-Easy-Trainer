# isaac_robot_control - Project Guide

MoveIt2 + IsaacSim 연동을 위한 ROS2 패키지.
로봇에 독립적인 추상화 레이어 위에 Franka/Piper 구현을 제공한다.

## Architecture

```
core/                          # 추상화 레이어 (로봇 무관)
  robot.py                     #   RobotConfig, GripperConfig (ABC/dataclass)
  bridge.py                    #   TrajectoryBridge (MoveIt ↔ IsaacSim 중간 브릿지)
  controller.py                #   RobotController (MoveGroup/Cartesian/Gripper 통합)
  task.py                      #   BaseTask (validate → execute → evaluate 파이프라인)

robots/                        # 로봇별 구현
  franka.py, franka_bridge.py  #   Franka Panda (7DOF + 2-finger gripper)
  piper.py,  piper_bridge.py   #   Piper (6DOF + parallel-jaw, joint8 mimic)

tasks/
  pick_and_place.py            # PickAndPlaceTask (RobotSkills 기반)

utils/
  skills.py                    # RobotSkills (재사용 가능한 모션 프리미티브)

Entry points:
  trajectory_bridge.py         # Franka bridge 실행
  trajectory_bridge_piper.py   # Piper bridge 실행
  demo_control.py              # Franka 인터랙티브 데모
  pick_and_place.py            # Franka pick-and-place
  pick_and_place_piper.py      # Piper pick-and-place
  benchmark_piper.py           # Piper N-trial 벤치마크
  pick_and_place_service_piper.py  # Piper 서비스 기반 pick-and-place (1회 실행)

launch/
  isaac_moveit.launch.py       # Franka MoveIt2 런치
  isaac_moveit_piper.launch.py # Piper MoveIt2 런치
launch_common.py               # 공통 런치 유틸 (generate_isaac_moveit_launch)

config/
  piper.srdf                   # Piper SRDF (arm chain + gripper group)
  piper_moveit_controllers.yaml
  moveit_controllers.yaml      # Franka 컨트롤러
  ompl_planning.yaml           # OMPL 설정
  *.rviz                       # RViz 설정
```

## Core Abstractions

### RobotConfig (robot.py)
로봇 설정 인터페이스. 새 로봇 추가 시 이것만 구현하면 된다.

필수 속성:
- `name`, `arm_joint_names`, `home_joints` - 로봇/관절 정보
- `arm_controller_topic`, `gripper` (GripperConfig) - 컨트롤러 토픽
- `base_frame`, `ee_frame` - TF 프레임
- `move_group_name` - MoveIt planning group 이름
- `gripper_length` - EE frame에서 그리퍼 접촉점까지 Z 오프셋

선택 속성:
- `grasp_orientation` → (x, y, z, w) 쿼터니언. None이면 자유 자세.

### TrajectoryBridge (bridge.py)
MoveIt FollowJointTrajectory 액션 → IsaacSim joint_command 토픽 변환.

- `/simulation/joint_states` 구독, `/simulation/joint_command` 발행
- 100Hz 보간으로 trajectory 실행
- **전체 관절 합성 publish**: arm/gripper 컨트롤러가 따로 호출되더라도,
  `_publish_command()`에서 `_commanded_positions`(마지막 명령값)과 합쳐서
  항상 전체 관절을 하나의 메시지로 publish. 실물 로봇 드라이버 호환용.
- `mimic_joints` 프로퍼티: 패시브 조인트 자동 동기화 (예: Piper joint8 = -1 * joint7).
  mimic joint는 `_current_positions`에서 제외하고 `_append_mimic()`에서
  source joint 기반으로 계산하여 추가.

### RobotController (controller.py)
모든 모션 명령을 담당하는 통합 컨트롤러.

주요 메서드:
- `move_to_joint(dict)` - OMPL 관절 공간 플래닝
- `move_to_pose(x,y,z, ox,oy,oz,ow)` - Cartesian 우선, MoveGroup 폴백
- `move_linear(x,y,z, ox,oy,oz,ow)` - Cartesian 직선만 사용
- `set_gripper(width)` - 그리퍼 너비 제어
- `get_current_ee_orientation()` - TF에서 현재 EE 자세 조회

중요 파라미터:
- `min_fraction = 0.7` - Cartesian 경로 최소 달성률
- `velocity_scaling = 0.3` - Cartesian 속도 스케일링
- `orientation_tolerance = 0.35 rad` - MoveGroup 폴백 시 orientation 허용 오차

오브젝트 추적:
- `object_names` 파라미터로 Marker 토픽 자동 구독
- `object_positions` dict에 실시간 위치 저장
- 토픽: `/simulation/object_markers/{name}`

### BaseTask (task.py)
작업 추상 클래스. `run()` 호출 시 validate → execute → evaluate 순서 실행.

### RobotSkills (utils/skills.py)
RobotController를 감싸는 재사용 가능한 모션 프리미티브 라이브러리.
Task에서 직접 컨트롤러를 호출하지 않고, 이 클래스를 통해 모션을 수행한다.

기본 스킬:
- `go_home()`, `go_to_joints(dict)` - 관절 이동
- `open_gripper()`, `close_gripper()` - 그리퍼 제어
- `approach(x,y,z, height, **ori)` - 자유 공간 이동 + orientation 보정
- `descend(x,y,z, height)` - 직선 하강
- `lift(x,y,z, height)`, `retreat(x,y,z, height)` - 직선 상승

복합 스킬:
- `pick(x,y,z)` - approach → descend → close_gripper → lift
- `place(x,y,z)` - approach → descend → open_gripper → retreat
- `insert(x,y,z, velocity_scaling)` - approach → 느린 직선 하강 (peg-in-hole용)

유틸:
- `get_current_arm_joints()` - 현재 arm 관절 위치 조회
- `grasp_ori_kwargs` - RobotConfig의 grasp_orientation을 **kwargs dict로 변환

## Robot-Specific Details

### Piper (6-DOF)
- **Gripper**: joint7 (구동) + joint8 (mimic, -1배). URDF에서 mimic 태그 제거됨.
  bridge `_append_mimic()` + IsaacSim relay node 두 계층에서 joint8 자동 동기화.
- **Grasp orientation**: `(0, 1, 0, 0)` = Y축 180도 회전 = top-down
- **gripper_length**: -0.01m (tcp가 gripper_base보다 약간 안쪽)
- **Home**: 모든 관절 0
- **SRDF**: arm = chain(base_link → gripper_base), gripper = joint7만

### Franka Panda (7-DOF)
- **Gripper**: panda_finger_joint1 + panda_finger_joint2 (독립 제어)
- **gripper_length**: 0.103m
- **Home**: 사전 정의된 관절 각도

## Pick-and-Place Motion Sequence

PickAndPlaceTask는 RobotSkills의 복합 스킬을 사용하여 시퀀스를 실행한다:

```
1. 초기 관절 저장 (get_current_arm_joints)
2. skills.pick(cx, cy, cz)   → approach → descend → close_gripper → lift
3. skills.place(px, py, pz)  → approach → descend → open_gripper → retreat
4. 초기 관절 복귀 (go_to_joints)
```

높이 오프셋 (오브젝트 z + offset + gripper_length):
- APPROACH: 0.15m, GRASP: 0.01m, LIFT: 0.10m, PLACE: 0.08m

평가 (evaluate): pick_object가 place_target 위에 있고 XY 거리 < 8cm이면 성공.

## ROS2 Topics & Namespaces

IsaacSim 측 토픽은 `/simulation/` 네임스페이스:
- `/simulation/joint_states` - 관절 상태 (IsaacSim → Bridge)
- `/simulation/joint_command` - 관절 명령 (Bridge/외부 → relay node)
- `/simulation/joint_command_internal` - mimic 추가된 관절 명령 (relay → IsaacSim ArticulationController)
- `/simulation/tf`, `/simulation/tf_static` - TF 변환
- `/simulation/object_markers/{name}` - 오브젝트 위치 마커

MoveIt 측:
- `/move_action` - MoveGroup 액션
- `/compute_cartesian_path` - Cartesian 경로 계산
- `/{arm,gripper}_controller/follow_joint_trajectory` - 각 컨트롤러 액션

환경 리셋:
- `/simulation/reset_env` (std_srvs/Trigger) - IsaacSim 환경 리셋

서비스:
- `/pick_and_place` (std_srvs/Trigger) - pick-and-place 1회 실행 (환경 리셋 포함)

## Launch & Execution

```bash
# Docker MoveIt2 컨테이너 내부
ros2 launch isaac_robot_control isaac_moveit_piper.launch.py
ros2 run isaac_robot_control pick_and_place_piper
ros2 run isaac_robot_control benchmark_piper --ros-args -p n_trials:=10

# 서비스 기반 pick-and-place (노드 상주, 서비스 콜마다 1회 실행)
ros2 run isaac_robot_control pick_and_place_service_piper
ros2 service call /pick_and_place std_srvs/srv/Trigger

# 파라미터 지정
ros2 run isaac_robot_control pick_and_place_piper \
    --ros-args -p pick_object:=BlueCube -p place_target:=GreenPlate
```

## Known Constraints & Design Decisions

- **6-DOF orientation**: Piper는 6자유도라 위치(3DOF) + orientation(2DOF) 제약 시
  자유도가 1개뿐. orientation_tolerance를 0.35rad 이상으로 유지해야 OMPL goal sampling 성공.
  너무 타이트하면 "Unable to sample any valid states for goal tree" 에러 발생.

- **Cartesian 0% from Home**: Home에서 먼 위치로의 Cartesian 직선은 거의 항상 실패 (0%).
  `move_to_pose`가 자동으로 MoveGroup으로 폴백한다.

- **Mimic joint 처리 (2계층)**: IsaacSim URDF importer가 mimic 태그에서 physics constraint를
  생성하면 joint가 멈추는 버그 존재. URDF에서 mimic 태그를 제거하고 소프트웨어적으로 동기화.
  - **Bridge 레벨** (`_append_mimic`): MoveIt trajectory 명령 시 bridge가 joint8을 자동 추가.
    mimic joint는 `_current_positions`에서 제외, source joint 기반 계산값만 사용.
  - **IsaacSim relay 레벨**: `/simulation/joint_command` → relay node (mimic 추가) →
    `/simulation/joint_command_internal` → OmniGraph ArticulationController.
    MoveIt 경유하지 않는 외부 퍼블리셔도 mimic joint가 자동 적용됨.
  - 두 계층 모두 동일한 결과를 보장: joint7만 보내도 joint8이 -1배로 동기화.

- **allowed_start_tolerance**: 0.05로 설정 (기본 0.01은 IsaacSim 물리 시뮬레이션의
  미세한 관절 편차 때문에 자주 실패).

- **Trajectory 속도**: Cartesian 이동 시 velocity/acceleration scaling 0.3 기본값.
  물체를 잡고 들어올릴 때 너무 빠르면 물체가 튀어나감.

- **서비스 노드 executor 분리**: `pick_and_place_service_piper`는 서비스 서버 노드
  (별도 스레드 spin)와 컨트롤러 노드(메인 스레드 spin_until_future_complete)를 분리.
  동일 노드에서 spin + spin_until_future_complete를 사용하면 executor 충돌으로
  두 번째 서비스 콜부터 동작하지 않는다. threading.Event로 스레드 간 동기화.
