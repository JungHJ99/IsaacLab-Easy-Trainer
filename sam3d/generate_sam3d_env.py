import os
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
from isaacsim.simulation_app import SimulationApp

# 1. Simulation App 실행 (다른 모듈 임포트 전에 필수)
simulation_app = SimulationApp({"headless": True})

# Isaac Sim 관련 임포트
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.prims import XFormPrim
from isaacsim.sensors.camera import Camera
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.world import World
from pxr import UsdPhysics, Gf, Sdf

# --- [설정] 파일 경로 ---
ENV_NAME = "real2sim"  # 필요시 변경
base_dir = f"/workspace/sam-3d-objects/outputs/{ENV_NAME}"
mesh_dir = os.path.join(base_dir, "meshes")
transform_dir = os.path.join(base_dir, "transforms")
output_usd_path = f"/workspace/isaaclab/custom_usds/{ENV_NAME}.usd"

# --- [핵심] 좌표계 변환 행렬 정의 ---
# GLB(Y-up) 모델을 Isaac Sim(Z-up) 좌표계로 바로잡는 회전 행렬입니다.
# X축 고정, Y -> Z, Z -> -Y (Standard Right-Handed Rotation)
T_yup_to_zup = np.array([
    [1,  0,  0,  0],
    [0,  0, -1,  0],
    [0,  1,  0,  0],
    [0,  0,  0,  1]
])

def main():
    # 1. World & Ground 생성
    world = World()
    world.scene.add_default_ground_plane()

    # 2. Scene Root 생성 (모든 물체의 부모)
    scene_root_path = "/World/Scene_Root"
    prim_utils.create_prim(scene_root_path, "Xform")
    scene_root = XFormPrim(scene_root_path)

    
    camera_path = f"{scene_root_path}/Camera"
    prim_utils.create_prim(camera_path, "Camera")
    camera_prim = get_current_stage().GetPrimAtPath(camera_path)

    target_euler_deg = [-180, 0, 180] # Z, Y, X 순서로 가정
    r = R.from_euler('zyx', target_euler_deg, degrees=True)
    q_xyzw = r.as_quat() # Scipy 결과: (x, y, z, w)

    # 2. PXR의 Gf.Quatd 형식으로 변환 (w가 스칼라 파트)
    isaac_quat = Gf.Quatd(q_xyzw[3], Gf.Vec3d(q_xyzw[0], q_xyzw[1], q_xyzw[2]))

    xform_order_attr = camera_prim.GetAttribute("xformOpOrder")
    if not xform_order_attr.Get():
        xform_order_attr.Set(["xformOp:translate", "xformOp:orient", "xformOp:scale"])

    orient_attr = camera_prim.GetAttribute("xformOp:orient")
    if not orient_attr:
        orient_attr = camera_prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd)
    orient_attr.Set(isaac_quat)

    translate_attr = camera_prim.GetAttribute("xformOp:translate")
    if not translate_attr:
        translate_attr = camera_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3)
    translate_attr.Set(Gf.Vec3d(0,0,-4))

    # 4. 파일 목록 로드
    if not os.path.exists(mesh_dir):
        print(f"Error: Mesh directory not found: {mesh_dir}")
        simulation_app.close()
        return

    mesh_files = sorted([f for f in os.listdir(mesh_dir) if f.endswith('.glb')])
    print(f"Processing {len(mesh_files)} meshes...")

    for i, mesh_filename in enumerate(mesh_files):
        # 인덱스 파싱 (파일명이 0.glb, 1.glb 형태라고 가정)
        try:
            base_name = os.path.splitext(mesh_filename)[0]
            file_index = int(base_name.split('_')[-1]) # ex: mesh_0.glb -> 0
        except (ValueError, IndexError):
            file_index = i

        json_filepath = os.path.join(transform_dir, f'{file_index}.json')
        if not os.path.exists(json_filepath):
            print(f"Skipping {mesh_filename}: No matching transform JSON found.")
            continue

        # --- [핵심 로직 시작] ---
        
        # A. JSON 로드 (Row-Major) -> Column-Major로 변환
        # PyTorch3D/JSON 저장 방식: Row-Major (v * M)
        # Isaac Sim/OpenGL 방식: Column-Major (M * v)
        # 따라서 불러온 후 반드시 .T (Transpose)를 해야 올바른 변환 행렬이 됩니다.
        with open(json_filepath, 'r') as f:
            T_json_row = np.array(json.load(f))
            T_world = T_json_row.T 

        # B. 최종 Transform 계산
        # 논리: "물체(GLB)를 Z-up으로 세운 뒤(T_yup_to_zup), 월드 위치로 이동(T_world)"
        # 수식(Col-Major): T_final = T_world * T_yup_to_zup
        T_local = T_world @ T_yup_to_zup

        # C. 행렬 분해 (Decompose)
        # 1) Translation
        translation = T_local[:3, 3]

        # 2) Scale (각 컬럼 벡터의 길이)
        scale_x = np.linalg.norm(T_local[:3, 0])
        scale_y = np.linalg.norm(T_local[:3, 1])
        scale_z = np.linalg.norm(T_local[:3, 2])
        scale = np.array([scale_x, scale_y, scale_z])

        # 3) Rotation Matrix 추출
        if np.isclose(scale, 0).any():
            rot_mat = np.eye(3)
        else:
            # Scale 성분을 제거하여 순수 회전만 남김
            # 주의: np.array로 묶으면 행(row)으로 쌓이므로 다시 .T를 해줘야 원래 컬럼 형태 유지
            rot_mat = np.array([
                T_local[:3, 0] / scale_x,
                T_local[:3, 1] / scale_y,
                T_local[:3, 2] / scale_z
            ]).T

        # D. Quaternion 변환 (Scipy -> Isaac)
        r = R.from_matrix(rot_mat)
        quat_xyzw = r.as_quat() # Scipy는 (x, y, z, w)
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]) # Isaac은 (w, x, y, z)

        # --- [Prim 생성 및 배치] ---
        prim_path = f"{scene_root_path}/Object_{file_index}"
        mesh_abs_path = os.path.abspath(os.path.join(mesh_dir, mesh_filename))
        
        # Mesh import
        prim_utils.create_prim(prim_path=prim_path, usd_path=mesh_abs_path)
        
        # Transform 적용
        xform = XFormPrim(prim_path)
        xform.set_local_pose(translation=translation, orientation=quat_wxyz)
        xform.set_local_scale(scale)
        
        # Physics 적용 (충돌체 등)
        UsdPhysics.CollisionAPI.Apply(xform.prim)
        UsdPhysics.RigidBodyAPI.Apply(xform.prim)
        
        print(f"[{file_index}] Success: {mesh_filename}")

    # 5. (옵션) Scene Root 전체 회전 보정
    # 만약 전체 씬이 여전히 누워있다면 여기서 Scene_Root를 90도 돌립니다.
    # 일반적으로 개별 객체에서 T_yup_to_zup을 했으면 여긴 0도여야 맞지만, 
    # SAM-3D 결과가 전체적으로 누워있다면 90도로 두세요.
    
    # 예: 전체 씬이 X축으로 90도 돌아야 바닥에 선다면:
    global_rot_deg = 90  
    r_global = R.from_euler('x', global_rot_deg, degrees=True)
    q_g_xyzw = r_global.as_quat()
    q_g_wxyz = np.array([q_g_xyzw[3], q_g_xyzw[0], q_g_xyzw[1], q_g_xyzw[2]])
    
    scene_root.set_world_pose(position=np.array([0,0,0]), orientation=q_g_wxyz)
    print(f"Applied Global Rotation: {global_rot_deg} deg")

    # 6. USD 저장
    stage = get_current_stage()
    os.makedirs(os.path.dirname(output_usd_path), exist_ok=True)
    stage.Export(output_usd_path)
    print(f"Done! Saved to: {output_usd_path}")
    
    simulation_app.close()

if __name__ == "__main__":
    main()