"""Fairino FR5 로봇 설정.

6-DOF 협동 로봇 (5kg payload).
그리퍼 별도 — Robotiq 2F-85 등 외부 그리퍼 부착 필요.
그리퍼 없이 사용 시 ee_frame = wrist3_link.
"""

from isaac_control_core.core.robot import RobotConfig, GripperConfig


class Fairino5Config(RobotConfig):
    """Fairino FR5 6-DOF 로봇 설정 (그리퍼 없음)."""

    @property
    def name(self) -> str:
        return "fairino5"

    @property
    def arm_joint_names(self) -> list[str]:
        return ["j1", "j2", "j3", "j4", "j5", "j6"]

    @property
    def home_joints(self) -> dict[str, float]:
        return {
            "j1": 0.0,
            "j2": 0.0,
            "j3": 0.0,
            "j4": 0.0,
            "j5": 0.0,
            "j6": 0.0,
        }

    @property
    def arm_controller_topic(self) -> str:
        return "/arm_controller/follow_joint_trajectory"

    @property
    def gripper(self) -> GripperConfig | None:
        # 그리퍼 없음 — Robotiq 2F-85 부착 시 서브클래스에서 override
        return None

    @property
    def base_frame(self) -> str:
        return "base_link"

    @property
    def ee_frame(self) -> str:
        return "wrist3_link"

    @property
    def move_group_name(self) -> str:
        return "arm"

    @property
    def gripper_length(self) -> float:
        return 0.0  # 그리퍼 없음

    @property
    def grasp_orientation(self) -> tuple[float, float, float, float]:
        """Top-down 그래스프 orientation (x, y, z, w)."""
        return (0.0, 1.0, 0.0, 0.0)


class Fairino5Robotiq85Config(Fairino5Config):
    """Fairino FR5 + Robotiq 2F-85 그리퍼 설정.

    Robotiq 2F-85:
    - 개구 범위: 0 ~ 0.085m (85mm)
    - 단일 actuator (finger_joint) → mimic으로 나머지 동기화
    """

    @property
    def name(self) -> str:
        return "fairino5_robotiq85"

    @property
    def gripper(self) -> GripperConfig:
        return GripperConfig(
            joint_names=["robotiq_85_left_knuckle_joint"],
            open_width=0.804,   # revolute joint: 0~0.804 rad
            close_width=0.0,
            controller_topic="/gripper_controller/follow_joint_trajectory",
        )

    @property
    def ee_frame(self) -> str:
        # Robotiq 부착 시 ee는 gripper tip
        return "robotiq_85_base_link"

    @property
    def gripper_length(self) -> float:
        return -0.05  # Robotiq 2F-85 핑거 길이 보정
