"""
환경 편집기로 생성된 환경.

Usage:
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/new_env.py
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/new_env.py --headless
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ros2_env.base_env import BaseEnv

PIPER_PKG = Path(__file__).resolve().parents[2] / "ros_pkgs/piper_description"
PIPER_URDF_V100 = str(PIPER_PKG / "urdf/piper_description_v100.urdf")
PIPER_URDF_STD = str(PIPER_PKG / "urdf/piper_description.urdf")
USD_DIR = Path(__file__).resolve().parents[2] / "custom_usds/usd"



class NewEnv(BaseEnv):

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
        self.add_object(name='RedCube', size=0.05, color=(0.800000011920929, 0.0, 0.0), shape='cube', scale=1.0, position=(0.3, 0.0, 0.052), orientation=(0.0, 0.0, 0.0), position_delta=(0.1, 0.3499999940395355, 0.0), orientation_delta=(0.0, 0.0, 45.0), friction=3.0, mass=0.02)
        self.add_object(name='WhitePlate', size=0.20000000298023224, color=(0.8, 0.8, 0.8), shape='cylinder', height=0.0010000000474974513, scale=1.0, position=(0.3, -1.4776690022699768e-06, 0.05699978023767471), position_delta=(0.1, 0.3499999940395355, 0.0), friction=3.0, mass=0.02)

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
    NewEnv(headless=headless, use_v100=use_v100).run()
