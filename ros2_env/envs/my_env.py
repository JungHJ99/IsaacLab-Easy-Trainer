"""
환경 편집기로 생성된 환경.

Usage:
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/my_env.py
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/my_env.py --headless
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ros2_env.base_env import BaseEnv

PIPER_PKG = Path(__file__).resolve().parents[2] / "ros_pkgs/piper_description"
PIPER_URDF_V100 = str(PIPER_PKG / "urdf/piper_description_v100.urdf")
PIPER_URDF_STD = str(PIPER_PKG / "urdf/piper_description.urdf")
USD_DIR = Path(__file__).resolve().parents[2] / "custom_usds/usd"


class MyEnv2(BaseEnv):

    def __init__(self, headless=False, use_v100=True):
        self._use_v100 = use_v100
        super().__init__(headless=headless)

    def setup_scene(self):
        self.add_table(center=[0.365, 0.0, 0.012], scale=[0.9, 1.4, 0.04],
                       color=(0.03, 0.03, 0.03), spawn_range=[[0.43, 0.18], [0.18, -0.18]])

        urdf = PIPER_URDF_V100 if self._use_v100 else PIPER_URDF_STD
        self.add_robot_from_urdf(
            name="piper",
            urdf_path=urdf,
            prim_path="/World/Piper",
            position=[0.0, 0.0, 0.032],
            color=(0.01, 0.01, 0.01),
        )
        self.add_mimic_joint("joint8", "joint7", multiplier=-1.0)

        # ── 오브젝트 ──
        # Sektion Cabinet: Isaac Sim 번들 articulated 서랍장 (drawer_top_joint / drawer_bottom_joint)
        # 에셋 경로는 설치된 Isaac 버전에 따라 다르므로 런타임 헬퍼로 자동 해석.
        try:
            from isaacsim.storage.native import get_assets_root_path
        except ImportError:
            from omni.isaac.core.utils.nucleus import get_assets_root_path
        assets_root = get_assets_root_path()
        sektion_cabinet_usd = f"{assets_root}/Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd"
        print(f"[INFO] Sektion Cabinet USD: {sektion_cabinet_usd}")

        # articulated USD는 wrap_rigidbody=None 자동 감지로 래핑 스킵됨.
        # uniform scale은 articulation 물리에 지장 없음 (PhysX가 조인트/관성 자동 보정).
        self.add_object(
            "Cabinet",
            usd_path=sektion_cabinet_usd,
            position=(1.1, 0.0, 0.0),
            orientation=(0, 0, 180),
            scale=0.5,
        )

        # ── 카메라 ──
        self.add_camera(name='side_cam', position=[0.42129395649021645, -1.6332931339129597, 1.3261807891504858], orientation=[51.059105, 1.2660791, 3.953346], position_delta=[0.03, 0.03, 0.03], orientation_delta=[1, 1, 1])
        self.add_camera(name='top_cam', position=[0.8889916444797262, 0.08290099598718931, 2.0569620163434847], orientation=[16.736338, 2.5316212, 89.90968], position_delta=[0.03, 0.03, 0.03], orientation_delta=[1, 1, 1])
        self.add_camera(name='wrist_cam', position=[0.012212855875360258, 0.1438242841130981, 0.07214914261067958], orientation=[128.89171, 0.80611223, 176.2427], parent_prim='/piper_camera/link6', focal_length=13.0)

        # ── 조명 ──
        self.add_light(type='dome', name='DomeLight', intensity=1650, color_temp=6000, intensity_delta=350, color_temp_delta=500)
        self.add_light(type='dome', name='DomeLight2', intensity=1500.0, color_temp=6000.0)

        self.set_background_randomization(brightness_range=[0.0, 0.3])


if __name__ == "__main__":
    headless = "--headless" in sys.argv
    use_v100 = "--standard" not in sys.argv
    MyEnv2(headless=headless, use_v100=use_v100).run()
