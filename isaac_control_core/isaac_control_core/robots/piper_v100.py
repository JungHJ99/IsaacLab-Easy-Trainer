"""Piper V100 로봇 설정 (넓은 그리퍼 버전).

그리퍼 개구: 0~0.05m (시뮬레이션), 실물 0~0.10m.
gripper_base 링크 없음 — joint7/8이 link6에 직접 연결.
ee_frame = tcp (URDF에 추가한 fixed link, link6 +Z 0.135 = grasp center).
base 링크: arm_base.
"""

from isaac_control_core.core.robot import RobotConfig, GripperConfig


class PiperV100Config(RobotConfig):
    """Piper V100 6-DOF 로봇 설정 (넓은 그리퍼)."""

    @property
    def name(self) -> str:
        return "piper_v100"

    @property
    def arm_joint_names(self) -> list[str]:
        return [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6",
        ]

    @property
    def home_joints(self) -> dict[str, float]:
        return {
            "joint1": 0.0,
            "joint2": 0.0,
            "joint3": 0.0,
            "joint4": 0.0,
            "joint5": 0.0,
            "joint6": 0.0,
        }

    @property
    def arm_controller_topic(self) -> str:
        return "/arm_controller/follow_joint_trajectory"

    @property
    def gripper(self) -> GripperConfig:
        return GripperConfig(
            joint_names=["joint7"],
            open_width=0.10,    # 실물 100mm
            close_width=0.0,
            controller_topic="/gripper_controller/follow_joint_trajectory",
        )

    @property
    def base_frame(self) -> str:
        return "arm_base"

    @property
    def ee_frame(self) -> str:
        # tcp = link6 + fixed offset (0, 0, 0.135) = 두 finger의 grasp center.
        # URDF, cuRobo yml, RobotConfig 모두 이 frame을 ee로 사용 → frame mismatch 없음.
        return "tcp"

    @property
    def move_group_name(self) -> str:
        return "arm"

    @property
    def gripper_length(self) -> float:
        # ee_frame이 'tcp' (= grasp center)이므로 추가 offset 불필요.
        # descend_z = cube_z + grasp_offset + 0 → tcp가 cube center에 정확히 도달.
        return 0.0

    @property
    def grasp_orientation(self) -> tuple[float, float, float, float]:
        """Top-down 그래스프 orientation (x, y, z, w)."""
        return (0.0, 1.0, 0.0, 0.0)
