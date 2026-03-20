"""Piper용 TrajectoryBridge 구현."""

from isaac_robot_control.core.bridge import TrajectoryBridge


class PiperTrajectoryBridge(TrajectoryBridge):
    """Piper arm + gripper controller 브릿지."""

    @property
    def controller_topics(self) -> list[str]:
        return [
            "/arm_controller/follow_joint_trajectory",
            "/gripper_controller/follow_joint_trajectory",
        ]

    @property
    def mimic_joints(self) -> dict[str, tuple[str, float, float]]:
        """joint8은 joint7의 mimic (multiplier=-1, offset=0)."""
        return {
            "joint8": ("joint7", -1.0, 0.0),
        }
