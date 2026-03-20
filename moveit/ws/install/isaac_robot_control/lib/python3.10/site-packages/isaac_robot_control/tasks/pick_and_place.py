"""Pick-and-Place Task 구현.

오브젝트를 잡아서 목표 위치에 놓는 작업.
pick_object, place_target을 인자로 받아 범용적으로 사용 가능.
"""

from __future__ import annotations

import time
import math
import rclpy
from typing import TYPE_CHECKING

from isaac_robot_control.core.task import BaseTask

if TYPE_CHECKING:
    from isaac_robot_control.core.controller import RobotController


# 높이 오프셋 (오브젝트 z 좌표에 더해짐, ee_frame 기준)
APPROACH_OFFSET = 0.15
GRASP_OFFSET = 0.01
LIFT_OFFSET = 0.10
PLACE_OFFSET = 0.08


class PickAndPlaceTask(BaseTask):
    """Pick-and-Place Task.

    Args:
        controller: 로봇 컨트롤러.
        pick_object: 잡을 오브젝트 이름 (Marker ns).
        place_target: 놓을 목표 오브젝트 이름 (Marker ns).
    """

    def __init__(
        self,
        controller: RobotController,
        pick_object: str,
        place_target: str,
    ):
        super().__init__(controller)
        self._pick_object = pick_object
        self._place_target = place_target

    @property
    def pick_object(self) -> str:
        return self._pick_object

    @property
    def place_target(self) -> str:
        return self._place_target

    def validate(self) -> bool:
        """pick_object와 place_target의 위치가 수신되었는지 확인."""
        positions = self._controller.object_positions
        if self._pick_object not in positions:
            self.logger.error(f"오브젝트 '{self._pick_object}' 위치를 찾을 수 없습니다.")
            return False
        if self._place_target not in positions:
            self.logger.error(f"오브젝트 '{self._place_target}' 위치를 찾을 수 없습니다.")
            return False
        return True

    def execute(self) -> bool:
        """Pick-and-Place 전체 시퀀스 실행."""
        if not self.validate():
            return False

        ctrl = self._controller
        robot = ctrl.robot_config
        gripper_len = robot.gripper_length

        pick_pos = ctrl.object_positions[self._pick_object]
        place_pos = ctrl.object_positions[self._place_target]

        cx, cy, cz = pick_pos
        px, py, pz = place_pos

        approach_h = APPROACH_OFFSET + gripper_len
        grasp_h = GRASP_OFFSET + gripper_len
        lift_h = LIFT_OFFSET + gripper_len
        place_h = PLACE_OFFSET + gripper_len

        # 그래스프 orientation (top-down) — dict로 변환하여 **kwargs로 전달
        grasp_ori = robot.grasp_orientation  # (x, y, z, w) or None
        ori_kwargs = {}
        if grasp_ori:
            ori_kwargs = dict(ox=grasp_ori[0], oy=grasp_ori[1],
                              oz=grasp_ori[2], ow=grasp_ori[3])

        self.logger.info("=" * 50)
        self.logger.info("  Pick-and-Place 시작")
        self.logger.info(f"  {self._pick_object}:    ({cx:.3f}, {cy:.3f}, {cz:.3f})")
        self.logger.info(f"  {self._place_target}: ({px:.3f}, {py:.3f}, {pz:.3f})")
        self.logger.info(f"  계획 높이 ({robot.ee_frame} z):")
        self.logger.info(f"    approach: {cz + approach_h:.3f}")
        self.logger.info(f"    grasp:    {cz + grasp_h:.3f}")
        self.logger.info(f"    lift:     {cz + lift_h:.3f}")
        if ori_kwargs:
            self.logger.info(f"  grasp orientation: {grasp_ori}")
        self.logger.info("=" * 50)

        # 현재 arm 관절 위치 저장 (마지막에 복귀용)
        js = ctrl._current_joint_state
        arm_names = set(robot.arm_joint_names)
        initial_joints = {
            name: js.position[i]
            for i, name in enumerate(js.name)
            if name in arm_names
        }
        self.logger.info(f"  초기 관절: {initial_joints}")

        # Step 1: Pre-grasp — 자유 공간 이동 (orientation 제약 포함)
        self.logger.info(f"\n[Step 1/6] Pre-grasp: {self._pick_object} 위쪽 접근")
        if not ctrl.move_to_pose(cx, cy, cz + approach_h, **ori_kwargs):
            self.logger.error("Pre-grasp 실패!")
            return False

        # Step 3.5: Orientation 보정 — MoveGroup 도착 자세가 부정확할 수 있으므로
        #           같은 위치에서 정확한 orientation으로 Cartesian 보정
        if ori_kwargs:
            self.logger.info("  Orientation 보정 (Cartesian)")
            ctrl.move_linear(cx, cy, cz + approach_h, **ori_kwargs)

        # Step 2: Grasp 하강 — Cartesian 직선 (현재 자세 유지)
        self.logger.info(f"\n[Step 2/6] Grasp: {self._pick_object}로 직선 하강")
        if not ctrl.move_linear(cx, cy, cz + grasp_h):
            self.logger.error("Grasp 접근 실패!")
            return False

        # Step 3: 그리퍼 닫기
        self.logger.info("\n[Step 3/6] 그리퍼 닫기")
        ctrl.set_gripper(robot.gripper.close_width)
        time.sleep(0.5)

        # Step 4: Lift — Cartesian 직선 (현재 자세 유지)
        self.logger.info("\n[Step 4/6] 직선 들어올리기")
        if not ctrl.move_linear(cx, cy, cz + lift_h):
            self.logger.error("Lift 실패!")
            return False

        # Step 5: 목표 위치로 이동 + 하강
        self.logger.info(f"\n[Step 5/6] {self._place_target} 위로 이동")
        if not ctrl.move_to_pose(px, py, pz + approach_h, **ori_kwargs):
            self.logger.error("접시 접근 실패!")
            return False

        self.logger.info("  목표 위로 직선 하강")
        if not ctrl.move_linear(px, py, pz + place_h):
            self.logger.error("Place 실패!")
            return False

        # Step 6: 그리퍼 열기 + 후퇴 + 초기 위치 복귀
        self.logger.info("\n[Step 6/6] 그리퍼 열기 + 후퇴")
        ctrl.set_gripper(robot.gripper.open_width)
        time.sleep(0.5)

        ctrl.move_linear(px, py, pz + lift_h)

        # 초기 위치 복귀
        self.logger.info("\n[완료] 초기 위치로 복귀")
        ctrl.move_to_joint(initial_joints)

        self.logger.info("\n" + "=" * 50)
        self.logger.info("  Pick-and-Place 모션 완료!")
        self.logger.info("=" * 50)
        return True

    def evaluate(self) -> bool:
        """pick_object가 place_target 위에 있는지 판정.

        XY 거리가 place_target 크기 이내이고,
        pick_object Z가 place_target Z보다 위에 있으면 성공.
        """
        ctrl = self._controller

        # 최신 위치 수신을 위해 잠시 spin
        for _ in range(10):
            rclpy.spin_once(ctrl, timeout_sec=0.1)

        positions = ctrl.object_positions
        if self._pick_object not in positions or self._place_target not in positions:
            self.logger.error("evaluate: 오브젝트 위치를 읽을 수 없습니다.")
            return False

        pick_pos = positions[self._pick_object]
        place_pos = positions[self._place_target]

        dx = pick_pos[0] - place_pos[0]
        dy = pick_pos[1] - place_pos[1]
        xy_dist = math.sqrt(dx * dx + dy * dy)
        z_diff = pick_pos[2] - place_pos[2]

        self.logger.info(f"[판정] {self._pick_object} 위치: "
                         f"({pick_pos[0]:.3f}, {pick_pos[1]:.3f}, {pick_pos[2]:.3f})")
        self.logger.info(f"[판정] {self._place_target} 위치: "
                         f"({place_pos[0]:.3f}, {place_pos[1]:.3f}, {place_pos[2]:.3f})")
        self.logger.info(f"[판정] XY 거리: {xy_dist:.3f}m, Z 차이: {z_diff:.3f}m")

        # place_target 반경 이내 + pick_object가 위에 있어야 성공
        xy_threshold = 0.08  # 8cm 이내
        success = xy_dist < xy_threshold and z_diff > 0
        return success
