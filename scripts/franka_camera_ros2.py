"""
Franka Panda + ROS2 Camera Demo

Isaac Sim 헤드리스 환경에서 Franka Panda 로봇을 sinusoidal 관절 동작으로 제어하고
카메라 데이터를 ROS2 토픽으로 퍼블리쉬하는 통합 예제.

Usage (컨테이너 내부):
    /workspace/isaaclab/_isaac_sim/python.sh scripts/franka_camera_ros2.py

또는 프로젝트 루트에서:
    ./run_franka_ros2.sh

퍼블리쉬 토픽:
    /joint_states          - sensor_msgs/JointState  (로봇 관절 상태)
    /camera_rgb            - sensor_msgs/Image        (RGB 이미지)
    /camera_depth          - sensor_msgs/Image        (깊이 이미지)
    /camera_camera_info    - sensor_msgs/CameraInfo   (카메라 내부 파라미터)
    /camera_pointcloud     - sensor_msgs/PointCloud2  (포인트 클라우드)
    /tf                    - tf2_msgs/TFMessage        (좌표 변환)
    /clock                 - rosgraph_msgs/Clock       (시뮬레이션 시간)

확인 방법:
    ros2 topic list
    ros2 topic hz /franka/joint_states
    rviz2  (Fixed Frame: world, Add: RobotModel, Camera, PointCloud2)
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import math

import carb
import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.prims import is_prim_path_valid, set_targets
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
from pxr import UsdGeom, Gf

# DLAA 모드 설정 - GUI의 "auto"와 동일 효과
# DLAA = 네이티브 해상도 DLSS (내부 렌더 해상도 = 출력 해상도, 노이즈 최소)
_s = carb.settings.get_settings()
_s.set("/rtx/post/dlss/enabled", True)
_s.set("/rtx/post/aa/op", 4)          # 4 = DLAA (다운스케일 없음)
_s.set("/rtx/post/dlss/execMode", 3)  # 3 = Quality

# ROS2 브리지 익스텐션 활성화
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
simulation_app.update()


# ---------------------------------------------------------------------------
# ROS2 OmniGraph 퍼블리셔 설정 함수들
# ---------------------------------------------------------------------------

def setup_clock_and_joint_state_publisher(robot_prim_path: str):
    """Clock + 로봇 Joint State를 ROS2로 퍼블리쉬하는 OmniGraph 생성."""
    graph_path = "/RobotStateGraph"

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", "/clock"),
                ("PublishJointState.inputs:topicName", "/franka/joint_states"),
            ],
        },
    )

    # Joint State 퍼블리셔가 Franka 프림을 바라보도록 연결
    stage = omni.usd.get_context().get_stage()
    set_targets(
        stage.GetPrimAtPath(graph_path + "/PublishJointState"),
        "inputs:targetPrim",
        [robot_prim_path],
    )


def setup_robot_tf_publisher(robot_prim_path: str):
    """로봇 링크들의 TF를 ROS2로 퍼블리쉬하는 OmniGraph 생성."""
    graph_path = "/RobotTFGraph"

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishTF.inputs:topicName", "/tf"),
            ],
        },
    )

    stage = omni.usd.get_context().get_stage()
    set_targets(
        stage.GetPrimAtPath(graph_path + "/PublishTF"),
        "inputs:targetPrims",
        [robot_prim_path],
    )


def publish_camera_info(camera: Camera, freq: int):
    """카메라 내부 파라미터(CameraInfo)를 ROS2로 퍼블리쉬."""
    from isaacsim.ros2.bridge import read_camera_info

    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_camera_info"
    frame_id = camera.prim_path.split("/")[-1]

    writer = rep.writers.get("ROS2PublishCameraInfo")
    camera_info = read_camera_info(render_product_path=render_product)
    writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=1,
        topicName=topic_name,
        width=camera_info["width"],
        height=camera_info["height"],
        projectionType=camera_info["projectionType"],
        k=camera_info["k"].reshape([1, 9]),
        r=camera_info["r"].reshape([1, 9]),
        p=camera_info["p"].reshape([1, 12]),
        physicalDistortionModel=camera_info["physicalDistortionModel"],
        physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatch" + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_rgb(camera: Camera, freq: int):
    """RGB 이미지를 ROS2로 퍼블리쉬."""
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_rgb"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(frameId=frame_id, nodeNamespace="", queueSize=1, topicName=topic_name)
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_depth(camera: Camera, freq: int):
    """깊이 이미지를 ROS2로 퍼블리쉬."""
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_depth"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(frameId=frame_id, nodeNamespace="", queueSize=1, topicName=topic_name)
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_pointcloud(camera: Camera, freq: int):
    """포인트 클라우드를 ROS2로 퍼블리쉬."""
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_pointcloud"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )
    writer = rep.writers.get(rv + "ROS2PublishPointCloud")
    writer.initialize(frameId=frame_id, nodeNamespace="", queueSize=1, topicName=topic_name)
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_camera_tf(camera: Camera):
    """카메라 좌표계 TF를 ROS2로 퍼블리쉬."""
    camera_prim = camera.prim_path
    camera_frame_id = camera_prim.split("/")[-1]
    graph_path = "/CameraTFGraph"

    if not is_prim_path_valid(graph_path):
        og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("RosPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                    ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                ],
            },
        )

    og.Controller.edit(
        graph_path,
        {
            og.Controller.Keys.CREATE_NODES: [
                ("PublishTF_" + camera_frame_id, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                (
                    "PublishRawTF_" + camera_frame_id + "_optical",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishTF_" + camera_frame_id + ".inputs:topicName", "/tf"),
                ("PublishRawTF_" + camera_frame_id + "_optical.inputs:topicName", "/tf"),
                (
                    "PublishRawTF_" + camera_frame_id + "_optical.inputs:parentFrameId",
                    camera_frame_id,
                ),
                (
                    "PublishRawTF_" + camera_frame_id + "_optical.inputs:childFrameId",
                    camera_frame_id + "_optical",
                ),
                (
                    "PublishRawTF_" + camera_frame_id + "_optical.inputs:rotation",
                    [0.5, -0.5, 0.5, 0.5],
                ),
            ],
            og.Controller.Keys.CONNECT: [
                (
                    graph_path + "/OnTick.outputs:tick",
                    "PublishTF_" + camera_frame_id + ".inputs:execIn",
                ),
                (
                    graph_path + "/OnTick.outputs:tick",
                    "PublishRawTF_" + camera_frame_id + "_optical.inputs:execIn",
                ),
                (
                    graph_path + "/IsaacClock.outputs:simulationTime",
                    "PublishTF_" + camera_frame_id + ".inputs:timeStamp",
                ),
                (
                    graph_path + "/IsaacClock.outputs:simulationTime",
                    "PublishRawTF_" + camera_frame_id + "_optical.inputs:timeStamp",
                ),
            ],
        },
    )

    stage = omni.usd.get_context().get_stage()
    set_targets(
        stage.GetPrimAtPath(graph_path + "/PublishTF_" + camera_frame_id),
        "inputs:targetPrims",
        [camera_prim],
    )


# ---------------------------------------------------------------------------
# Franka 관절 목표 계산 (sinusoidal 웨이포인트 보간)
# ---------------------------------------------------------------------------

# Franka Panda 홈 포지션 (관절 순서: joint1~7, finger1, finger2)
FRANKA_HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04])

# 방문할 웨이포인트들 (arm 7개 관절만, 단위: rad)
WAYPOINTS = np.array([
    [ 0.0,  -0.785,  0.0, -2.356,  0.0,  1.571,  0.785],   # 홈
    [ 0.5,  -0.5,    0.3, -2.0,    0.2,  1.8,    1.2  ],    # 오른쪽 뻗기
    [-0.5,  -0.5,   -0.3, -2.0,   -0.2,  1.8,    0.3  ],    # 왼쪽 뻗기
    [ 0.0,  -1.2,    0.0, -2.8,    0.0,  2.2,    0.785],    # 위로 뻗기
    [ 0.0,  -0.785,  0.0, -2.356,  0.0,  1.571,  0.785],    # 홈 복귀
])

# 관절 제한 (소프트 리밋, rad) - Franka Panda 스펙
JOINT_LIMITS_LOW  = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_LIMITS_HIGH = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])


def get_joint_targets(t: float) -> np.ndarray:
    """
    시간 t(초)에 따라 웨이포인트를 순환하며 부드럽게 보간된 9-DOF 관절 목표를 반환.
    각 웨이포인트 체류 시간: 3초, 전환 시간: 2초
    """
    cycle = 5.0  # 웨이포인트당 사이클 길이 (초)
    n = len(WAYPOINTS)
    total = t % (cycle * n)
    idx = int(total / cycle)
    frac = (total - idx * cycle) / cycle

    wp_cur = WAYPOINTS[idx]
    wp_nxt = WAYPOINTS[(idx + 1) % n]

    # smoothstep 보간
    s = frac * frac * (3 - 2 * frac)
    arm_targets = wp_cur + s * (wp_nxt - wp_cur)
    arm_targets = np.clip(arm_targets, JOINT_LIMITS_LOW, JOINT_LIMITS_HIGH)

    # 그리퍼: 천천히 열고 닫기
    gripper_val = 0.04 + 0.035 * math.sin(math.pi * frac)
    return np.append(arm_targets, [gripper_val, gripper_val])


# ---------------------------------------------------------------------------
# 메인
# ---------------------------------------------------------------------------

def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_ground_plane()

    # 조명
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path="/World/DomeLight",
        prim_type="DomeLight",
        attributes={"inputs:intensity": 1500.0},
    )

    # 테이블 (간단한 박스로 대체)
    stage = omni.usd.get_context().get_stage()
    table = UsdGeom.Cube.Define(stage, "/World/Table")
    table.GetSizeAttr().Set(1.0)
    table.AddScaleOp().Set(Gf.Vec3d(0.8, 0.6, 0.05))
    table.AddTranslateOp().Set(Gf.Vec3d(0.4, 0.0, 0.4))
    table.CreateDisplayColorAttr([(0.55, 0.4, 0.25)])

    # 작업 대상 오브젝트 (큐브)
    target_cube = UsdGeom.Cube.Define(stage, "/World/TargetCube")
    target_cube.GetSizeAttr().Set(0.06)
    target_cube.AddTranslateOp().Set(Gf.Vec3d(0.55, 0.0, 0.455))
    target_cube.CreateDisplayColorAttr([(1.0, 0.3, 0.2)])

    # Franka Panda 스폰 (USD 자동 로드)
    robot_prim_path = "/World/Franka"
    franka = Franka(prim_path=robot_prim_path, name="franka")

    # 카메라: 로봇 정면 비스듬히 위에서 내려다보는 위치
    camera = Camera(
        prim_path="/World/Camera",
        position=np.array([2.0, 0.0, 1.0]),
        frequency=30,
        resolution=(640, 480),
        orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 40.0, 180.0]), degrees=True),
    )

    # 씬에 추가 후 리셋
    world.scene.add(franka)
    world.reset()
    camera.initialize()

    # 렌더링 파이프라인 워밍업
    print("[INFO] 렌더링 워밍업 중...")
    for _ in range(15):
        world.step(render=True)

    # ROS2 퍼블리셔 설정
    print("[INFO] ROS2 퍼블리셔 설정 중...")
    setup_clock_and_joint_state_publisher(robot_prim_path)
    setup_robot_tf_publisher(robot_prim_path)

    publish_freq = 30
    publish_camera_tf(camera)
    publish_camera_info(camera, publish_freq)
    publish_rgb(camera, publish_freq)
    publish_depth(camera, publish_freq)
    publish_pointcloud(camera, publish_freq)

    # 시뮬레이션 정보 출력
    num_dof = franka.num_dof
    joint_names = franka.dof_names
    print("\n" + "=" * 65)
    print("  Franka Panda + ROS2 Camera Demo 실행 중!")
    print("=" * 65)
    print(f"\n  로봇 DOF 수: {num_dof}")
    print(f"  관절 이름: {joint_names}")
    print("\n  퍼블리쉬 토픽:")
    print("    /joint_states          - sensor_msgs/JointState")
    print("    /camera_rgb            - sensor_msgs/Image (640x480 RGB)")
    print("    /camera_depth          - sensor_msgs/Image (640x480 Depth)")
    print("    /camera_camera_info    - sensor_msgs/CameraInfo")
    print("    /camera_pointcloud     - sensor_msgs/PointCloud2")
    print("    /tf                    - tf2_msgs/TFMessage")
    print("    /clock                 - rosgraph_msgs/Clock")
    print("\n  확인 명령어 (다른 터미널에서):")
    print("    ros2 topic list")
    print("    ros2 topic hz /joint_states")
    print("    ros2 topic echo /joint_states --once")
    print("=" * 65 + "\n")

    # 시뮬레이션 루프
    sim_dt = 1.0 / 60.0
    sim_time = 0.0

    while simulation_app.is_running():
        joint_targets = get_joint_targets(sim_time)
        franka.set_joint_positions(joint_targets)
        world.step(render=True)
        sim_time += sim_dt

    simulation_app.close()


if __name__ == "__main__":
    main()
