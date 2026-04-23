"""Move-and-Look 서비스 설정.

컨트롤러 생성 후 run_task_service()를 호출하는 헬퍼.
MoveIt/cuRobo 양쪽에서 동일하게 사용.

사용법 (entry point에서):
    from isaac_control_core.services.move_and_look_service import run
    from isaac_robot_control.core import MoveItController  # 또는 CuroboController
    run(MoveItController)
"""

import rclpy
from isaac_control_core.robots import PiperV100Config
from isaac_control_core.tasks.move_and_look import MoveAndLookTask
from isaac_control_core.services.service_runner import run_task_service


def run(controller_class, args=None, **controller_kwargs):
    """Move-and-Look 서비스 실행.

    Args:
        controller_class: MotionController 구현 클래스 (MoveItController 또는 CuroboController).
        args: ROS2 args.
        **controller_kwargs: controller_class에 전달할 추가 인자.
    """
    rclpy.init(args=args)

    tmp_node = rclpy.create_node("_move_and_look_params")
    tmp_node.declare_parameter("target_object", "AlienDoll")
    tmp_node.declare_parameter("height", 0.30)
    tmp_node.declare_parameter("tilt_deg", 30.0)
    target_object = tmp_node.get_parameter("target_object").get_parameter_value().string_value
    height = tmp_node.get_parameter("height").get_parameter_value().double_value
    tilt_deg = tmp_node.get_parameter("tilt_deg").get_parameter_value().double_value
    tmp_node.destroy_node()

    robot = PiperV100Config()
    controller = controller_class(
        robot_config=robot,
        object_names=[target_object],
        node_name="move_and_look_service_piper",
        **controller_kwargs,
    )
    controller.wait_for_ready()

    run_task_service(
        controller=controller,
        task_class=MoveAndLookTask,
        service_name="/move_and_look",
        service_label="Move-and-Look",
        pick_object=target_object,
        place_target=target_object,
        task_kwargs={"target_object": target_object, "height": height, "tilt_deg": tilt_deg},
        timeout=60.0,
    )
