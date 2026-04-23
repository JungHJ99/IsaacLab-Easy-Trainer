"""Move-and-Look Task — wrist 카메라로 물체를 내려다보는 자세로 이동."""

from __future__ import annotations

from typing import TYPE_CHECKING

from isaac_control_core.core.task import BaseTask
from isaac_control_core.utils.skills import RobotSkills

if TYPE_CHECKING:
    from isaac_control_core.core.controller import RobotController


class MoveAndLookTask(BaseTask):
    """Move-and-Look Task.

    Args:
        target_object: 바라볼 오브젝트 이름.
        height: 오브젝트 위로 올릴 높이 (m).
        tilt_deg: 수직에서 기울이는 각도. 0이면 top-down.
    """

    def __init__(
        self,
        controller: RobotController,
        target_object: str,
        height: float = 0.30,
        tilt_deg: float = 30.0,
        **_kwargs,
    ):
        super().__init__(controller)
        self._target_object = target_object
        self._height = height
        self._tilt_deg = tilt_deg

    @property
    def target_object(self) -> str:
        return self._target_object

    def validate(self) -> bool:
        return self._validate_positions(self._target_object)

    def execute(self) -> bool:
        if not self.validate():
            return False

        self._refresh_positions()
        skills = RobotSkills(self._controller)

        tx, ty, tz = self._controller.object_positions[self._target_object]

        self.logger.info("=" * 50)
        self.logger.info("  Move-and-Look 시작")
        self.logger.info(f"  {self._target_object}: ({tx:.3f}, {ty:.3f}, {tz:.3f})")
        self.logger.info(f"  height={self._height:.2f}m, tilt={self._tilt_deg:.0f}°")
        self.logger.info("=" * 50)

        if not skills.look_at_object(tx, ty, tz,
                                     height=self._height,
                                     tilt_deg=self._tilt_deg):
            self.logger.error("Move-and-Look 실패!")
            return False
        return True
