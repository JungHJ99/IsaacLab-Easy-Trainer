"""로봇 모션 스킬 라이브러리.

pick-and-place, peg-in-hole 등 다양한 태스크에서 재사용할 수 있는
모션 프리미티브를 제공합니다.
"""

from __future__ import annotations

import time
import rclpy
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaac_robot_control.core.controller import RobotController


class RobotSkills:
    """RobotController를 감싸는 모션 스킬 모음.

    Args:
        controller: 로봇 컨트롤러 인스턴스.
    """

    def __init__(self, controller: RobotController):
        self._ctrl = controller
        self._robot = controller.robot_config
        self._logger = controller.get_logger()

    @property
    def gripper_length(self) -> float:
        return self._robot.gripper_length

    @property
    def grasp_ori_kwargs(self) -> dict:
        """grasp_orientation을 **kwargs용 dict로 반환. None이면 빈 dict."""
        ori = self._robot.grasp_orientation
        if ori:
            return dict(ox=ori[0], oy=ori[1], oz=ori[2], ow=ori[3])
        return {}

    # ── 상태 조회 ──────────────────────────────────────────────

    def get_current_ee_position(self) -> tuple[float, float, float] | None:
        """TF에서 현재 EE 프레임의 position (x, y, z)을 조회 (base_frame 기준)."""
        try:
            t = self._ctrl._tf_buffer.lookup_transform(
                self._robot.base_frame,
                self._robot.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            p = t.transform.translation
            return (p.x, p.y, p.z)
        except Exception as e:
            self._logger.warn(f"EE position 조회 실패: {e}")
            return None

    def get_object_position(self, name: str) -> tuple[float, float, float] | None:
        """오브젝트 마커 위치를 최신으로 갱신 후 반환."""
        for _ in range(10):
            rclpy.spin_once(self._ctrl, timeout_sec=0.1)
        pos = self._ctrl.object_positions.get(name)
        if pos is None:
            self._logger.error(f"오브젝트 '{name}' 위치를 찾을 수 없습니다.")
        return pos

    def get_current_arm_joints(self) -> dict[str, float]:
        """현재 arm 관절 위치를 {name: position} dict로 반환."""
        js = self._ctrl._current_joint_state
        arm_names = set(self._robot.arm_joint_names)
        return {
            name: js.position[i]
            for i, name in enumerate(js.name)
            if name in arm_names
        }

    def go_to_joints(self, joint_positions: dict) -> bool:
        """관절 목표 위치로 이동."""
        self._logger.info(f"[skill] go_to_joints")
        return self._ctrl.move_to_joint(joint_positions)

    def go_home(self) -> bool:
        """홈 포지션으로 이동."""
        self._logger.info("[skill] go_home")
        return self._ctrl.move_to_joint(self._robot.home_joints)

    # ── 그리퍼 ─────────────────────────────────────────────────

    def open_gripper(self, wait: float = 0.5) -> bool:
        """그리퍼 열기."""
        self._logger.info("[skill] open_gripper")
        result = self._ctrl.set_gripper(self._robot.gripper.open_width)
        if wait > 0:
            time.sleep(wait)
        return result

    def close_gripper(self, wait: float = 0.5) -> bool:
        """그리퍼 닫기."""
        self._logger.info("[skill] close_gripper")
        result = self._ctrl.set_gripper(self._robot.gripper.close_width)
        if wait > 0:
            time.sleep(wait)
        return result

    # ── 이동 프리미티브 ────────────────────────────────────────

    def approach(self, x: float, y: float, z: float,
                 height: float = 0.15, **ori_kwargs) -> bool:
        """오브젝트 위쪽으로 자유 공간 이동 + orientation 보정.

        Args:
            x, y, z: 오브젝트 위치.
            height: 오브젝트 z에 더할 접근 높이 (gripper_length 포함).
            **ori_kwargs: orientation (ox, oy, oz, ow). 생략 시 현재 자세 유지.
        """
        target_z = z + height
        self._logger.info(f"[skill] approach → ({x:.3f}, {y:.3f}, {target_z:.3f})")

        if not self._ctrl.move_to_pose(x, y, target_z, **ori_kwargs):
            return False

        # orientation 보정 (MoveGroup 폴백 시 부정확할 수 있음)
        if ori_kwargs:
            self._logger.info("[skill]   orientation 보정 (Cartesian)")
            self._ctrl.move_linear(x, y, target_z, **ori_kwargs)

        return True

    def descend(self, x: float, y: float, z: float,
                height: float = 0.01) -> bool:
        """목표 높이로 직선 하강 (현재 자세 유지).

        Args:
            x, y, z: 오브젝트 위치.
            height: 오브젝트 z에 더할 높이 (gripper_length 포함).
        """
        target_z = z + height
        self._logger.info(f"[skill] descend → z={target_z:.3f}")
        return self._ctrl.move_linear(x, y, target_z)

    def lift(self, x: float, y: float, z: float,
             height: float = 0.10) -> bool:
        """직선 상승 (현재 자세 유지).

        Args:
            x, y, z: 오브젝트 위치.
            height: 오브젝트 z에 더할 높이 (gripper_length 포함).
        """
        target_z = z + height
        self._logger.info(f"[skill] lift → z={target_z:.3f}")
        return self._ctrl.move_linear(x, y, target_z)

    def retreat(self, x: float, y: float, z: float,
                height: float = 0.10) -> bool:
        """후퇴 상승 (lift와 동일, 의미적 구분용)."""
        target_z = z + height
        self._logger.info(f"[skill] retreat → z={target_z:.3f}")
        return self._ctrl.move_linear(x, y, target_z)

    # ── 복합 스킬 ──────────────────────────────────────────────

    def pick(self, x: float, y: float, z: float,
             approach_offset: float = 0.15,
             grasp_offset: float = 0.01,
             lift_offset: float = 0.10) -> bool:
        """오브젝트를 잡아서 들어올리기.

        approach → descend → close_gripper → lift 시퀀스.

        Args:
            x, y, z: 오브젝트 위치.
            approach_offset, grasp_offset, lift_offset: z 오프셋 (gripper_length 별도 추가).
        """
        gl = self.gripper_length
        ori = self.grasp_ori_kwargs

        if not self.approach(x, y, z, height=approach_offset + gl, **ori):
            return False
        if not self.descend(x, y, z, height=grasp_offset + gl):
            return False
        self.close_gripper()
        if not self.lift(x, y, z, height=lift_offset + gl):
            return False
        return True

    def place(self, x: float, y: float, z: float,
              approach_offset: float = 0.15,
              place_offset: float = 0.08,
              lift_offset: float = 0.10) -> bool:
        """오브젝트를 목표 위치에 놓기.

        approach → descend → open_gripper → retreat 시퀀스.

        Args:
            x, y, z: 목표 위치.
            approach_offset, place_offset, lift_offset: z 오프셋 (gripper_length 별도 추가).
        """
        gl = self.gripper_length
        ori = self.grasp_ori_kwargs

        if not self.approach(x, y, z, height=approach_offset + gl, **ori):
            return False
        if not self.descend(x, y, z, height=place_offset + gl):
            return False
        self.open_gripper()
        self.retreat(x, y, z, height=lift_offset + gl)
        return True

    def place_on_object(self, held_object: str, target_object: str,
                        stack_offset: float = 0.0,
                        approach_offset: float = 0.15,
                        place_offset: float = 0.01,
                        lift_offset: float = 0.10) -> bool:
        """홀드 중인 오브젝트를 target 오브젝트 위에 정확히 배치.

        EE와 홀드된 오브젝트의 상대 오프셋을 계산하여,
        그립 위치에 관계없이 오브젝트가 정확히 target 위에 놓이도록 보정한다.

        Args:
            held_object: 현재 잡고 있는 오브젝트 이름.
            target_object: 놓을 대상 오브젝트 이름.
            stack_offset: target 오브젝트 z 위에 추가할 높이.
            approach_offset, place_offset, lift_offset: z 오프셋 (gripper_length 별도 추가).
        """
        gl = self.gripper_length
        ori = self.grasp_ori_kwargs

        # 1) 현재 EE 위치와 홀드된 오브젝트 위치로 상대 오프셋 계산
        ee_pos = self.get_current_ee_position()
        held_pos = self.get_object_position(held_object)
        target_pos = self.get_object_position(target_object)

        if ee_pos is None or held_pos is None or target_pos is None:
            return False

        # offset = held_object - ee (base frame 기준)
        off_x = held_pos[0] - ee_pos[0]
        off_y = held_pos[1] - ee_pos[1]
        off_z = held_pos[2] - ee_pos[2]

        self._logger.info(f"[skill] place_on_object: EE-to-object offset = "
                          f"({off_x:.4f}, {off_y:.4f}, {off_z:.4f})")

        # 2) 목표: held_object가 target 위에 오도록 EE 목표 계산
        #    goal_ee = target_pos + (0, 0, stack_offset) - offset
        tx = target_pos[0] - off_x
        ty = target_pos[1] - off_y
        tz = target_pos[2] + stack_offset  # target 위 높이 (EE 기준은 아래에서 보정)

        self._logger.info(f"[skill] place_on_object: target=({target_pos[0]:.3f}, "
                          f"{target_pos[1]:.3f}, {target_pos[2]:.3f}), "
                          f"EE goal=({tx:.3f}, {ty:.3f}, ...)")

        # 3) approach → descend → open_gripper → retreat (보정된 XY 사용)
        if not self.approach(tx, ty, tz, height=approach_offset + gl - off_z, **ori):
            return False
        if not self.descend(tx, ty, tz, height=place_offset + gl - off_z):
            return False
        self.open_gripper()
        self.retreat(tx, ty, tz, height=lift_offset + gl - off_z)
        return True

    def insert(self, x: float, y: float, z: float,
               approach_offset: float = 0.15,
               insert_offset: float = 0.0,
               velocity_scaling: float = 0.1) -> bool:
        """Peg-in-hole 삽입 스킬.

        approach → 느린 직선 하강으로 삽입.

        Args:
            x, y, z: 홀 위치.
            approach_offset: 접근 높이.
            insert_offset: 삽입 깊이 (0이면 홀 표면까지, 음수면 더 깊이).
            velocity_scaling: 삽입 시 속도 스케일 (기본 0.1 = 느리게).
        """
        gl = self.gripper_length
        ori = self.grasp_ori_kwargs

        if not self.approach(x, y, z, height=approach_offset + gl, **ori):
            return False

        target_z = z + insert_offset + gl
        self._logger.info(f"[skill] insert → z={target_z:.3f} (slow)")
        return self._ctrl.move_linear(
            x, y, target_z,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=velocity_scaling,
        )
