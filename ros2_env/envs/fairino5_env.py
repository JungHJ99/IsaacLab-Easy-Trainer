"""
Fairino FR5 + Robotiq 2F-85 + 테이블 + 오브젝트 환경.

Usage:
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/fairino5_env.py
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/fairino5_env.py --headless
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ros2_env.base_env import BaseEnv

# URDF/메쉬 경로
FAIRINO_PKG = Path(__file__).resolve().parents[2] / "ros_pkgs/fairino_description"
FAIRINO_URDF = str(FAIRINO_PKG / "urdf/fairino5_robotiq85.urdf")

# Custom USD 경로
USD_DIR = Path(__file__).resolve().parents[2] / "custom_usds/usd"


class Fairino5Env(BaseEnv):

    def setup_scene(self):
        self.add_table(
            center=[0.4, 0.0, 0.012],
            scale=[0.9, 1.4, 0.04],
            color=(0.03, 0.03, 0.03),
            spawn_range=[[0.55, 0.25], [0.25, -0.25]],
        )

        # 테이블 표면 z = 0.012 + 0.04/2 = 0.032
        self.add_robot_from_urdf(
            name="fairino5",
            urdf_path=FAIRINO_URDF,
            position=[0.0, 0.0, 0.032],
            color=(0.05, 0.05, 0.05),
        )

        # Robotiq 2F-85 mimic joints
        self.add_mimic_joint("robotiq_85_right_knuckle_joint",
                             "robotiq_85_left_knuckle_joint", multiplier=1.0)
        self.add_mimic_joint("robotiq_85_left_inner_knuckle_joint",
                             "robotiq_85_left_knuckle_joint", multiplier=1.0)
        self.add_mimic_joint("robotiq_85_right_inner_knuckle_joint",
                             "robotiq_85_left_knuckle_joint", multiplier=1.0)
        self.add_mimic_joint("robotiq_85_left_finger_tip_joint",
                             "robotiq_85_left_knuckle_joint", multiplier=-1.0)
        self.add_mimic_joint("robotiq_85_right_finger_tip_joint",
                             "robotiq_85_left_knuckle_joint", multiplier=-1.0)

        # 오브젝트
        self.add_object("WhitePlate", size=0.22, color=(0.95, 0.95, 0.95),
                        shape="cylinder", height=0.01,
                        position_range=[[0.25, 0.05, 0.052], [0.55, 0.25, 0.052]])

        self.add_object("RedCube", size=0.05, color=(0.8, 0.0, 0.0), shape="cube",
                        position_range=[[0.25, -0.25, 0.052], [0.55, 0, 0.052]],
                        orientation_range=[[0, 0, -45], [0, 0, 45]])
        self.add_object("BlueCube", size=0.05, color=(0.0, 0.03, 0.1), shape="cube",
                        position_range=[[0.25, -0.25, 0.052], [0.55, 0, 0.052]],
                        orientation_range=[[0, 0, -45], [0, 0, 45]])
        self.add_object("GreenCube", size=0.05, color=(0.0, 0.03, 0.0), shape="cube",
                        position_range=[[0.25, -0.25, 0.052], [0.55, 0, 0.052]],
                        orientation_range=[[0, 0, -45], [0, 0, 45]])

        # 카메라 (외부 시점)
        self.add_camera(
            position=[0.4, -1.5, 1.3],
            orientation=[50, 0, 5],
            position_delta=[0.05, 0.05, 0.05],
            orientation_delta=[2, 2, 2],
            name="side_cam",
        )

        self.add_camera(
            position=[0.4, 0.0, 2.0],
            orientation=[0, 0, 90],
            position_delta=[0.05, 0.05, 0.05],
            orientation_delta=[2, 2, 2],
            name="top_cam",
        )

        # 손목 카메라 (wrist3_link에 부착)
        wrist_parent = f"{self._robot_prim_path}/wrist3_link"
        self.add_camera(
            position=[0.0, 0.0, 0.15],
            orientation=[180, 0, 0],
            parent_prim=wrist_parent,
            name="wrist_cam",
            focal_length=13.0,
        )

        # 조명 (dome + 추가 distant 2개)
        self.add_light(type="dome", name="DomeLight",
                       intensity=1650, intensity_delta=850,
                       color_temp=6000, color_temp_delta=500)
        self.add_light(type="distant", name="DistantLight_0",
                       intensity=500, intensity_delta=300,
                       orientation=[0.0, 0.0, 0.0],
                       orientation_delta=[60, 60, 180])
        self.add_light(type="distant", name="DistantLight_1",
                       intensity=500, intensity_delta=300,
                       orientation=[0.0, 0.0, 180.0],
                       orientation_delta=[60, 60, 180])
        self.set_background_randomization(brightness_range=[0.0, 0.3])


if __name__ == "__main__":
    headless = "--headless" in sys.argv
    Fairino5Env(headless=headless).run()
