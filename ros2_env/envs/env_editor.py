"""
환경 편집기. 시뮬레이션과 함께 UI 패널을 띄워 런타임으로 오브젝트/카메라/조명을
추가하고 파라미터를 조절한 뒤 BaseEnv 서브클래스 파이썬 파일로 저장한다.

Usage:
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/env_editor.py
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/env_editor.py --standard
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from ros2_env.base_env import BaseEnv


PIPER_PKG = Path(__file__).resolve().parents[2] / "ros_pkgs/piper_description"
PIPER_URDF_V100 = str(PIPER_PKG / "urdf/piper_description_v100.urdf")
PIPER_URDF_STD = str(PIPER_PKG / "urdf/piper_description.urdf")
USD_DIR = Path(__file__).resolve().parents[2] / "custom_usds/usd"
DEFAULT_SAVE_DIR = str(Path(__file__).resolve().parent)


# pi0_env.py 기본값과 일치
DEFAULT_TABLE = dict(
    center=[0.365, 0.0, 0.012],
    scale=[0.9, 1.4, 0.04],
    color=(0.03, 0.03, 0.03),
    spawn_range=[[0.43, 0.18], [0.18, -0.18]],
)
DEFAULT_ROBOT_POSITION = [0.0, 0.0, 0.032]
DEFAULT_ROBOT_COLOR = (0.01, 0.01, 0.01)


class EnvEditor(BaseEnv):
    """편집 모드 환경 — 로봇+테이블 고정 배치, UI로 오브젝트/카메라/조명 추가."""

    def __init__(self, headless: bool = False, use_v100: bool = True):
        super().__init__(headless=headless)
        self._use_v100 = use_v100
        self._object_specs: list[dict] = []
        self._camera_specs: list[dict] = []
        self._light_specs: list[dict] = []
        self._panel = None

    # ── 씬 구성 ──────────────────────────────────────────

    def setup_scene(self):
        self.add_table(**DEFAULT_TABLE)

        urdf = PIPER_URDF_V100 if self._use_v100 else PIPER_URDF_STD
        self.add_robot_from_urdf(
            name="piper",
            urdf_path=urdf,
            prim_path="/World/Piper",
            position=DEFAULT_ROBOT_POSITION,
            color=DEFAULT_ROBOT_COLOR,
        )
        self.add_mimic_joint("joint8", "joint7", multiplier=-1.0)

        # 기본 카메라 / 조명 (UI에서 수정/삭제 가능)
        self._add_default_cameras()
        self._add_default_lights()

        # UI 캔버스 범위
        x_range = (0.10, 0.65)
        y_range = (-0.45, 0.45)

        from ros2_env.env_editor_ui import EditorPanel, LauncherPanel
        self._panel = EditorPanel(
            env=self,
            spawn_x_range=x_range,
            spawn_y_range=y_range,
            default_save_dir=DEFAULT_SAVE_DIR,
        )

        # 런처 창 — New Env / Load Env 선택
        self._launcher = LauncherPanel(env=self, envs_dir=DEFAULT_SAVE_DIR)

    def _add_default_cameras(self):
        """side / top / wrist 기본 카메라를 editor spec에 등록 (삭제 가능)."""
        wrist_link = "link6" if self._use_v100 else "gripper_base"
        wrist_parent = f"{self._robot_prim_path}/{wrist_link}"

        defaults = [
            dict(
                name="side_cam",
                position=[0.42129395649021645, -1.6332931339129597, 1.3261807891504858],
                orientation=[51.059105, 1.2660791, 3.953346],
                position_delta=[0.03, 0.03, 0.03],
                orientation_delta=[1, 1, 1],
            ),
            dict(
                name="top_cam",
                position=[0.8889916444797262, 0.08290099598718931, 2.0569620163434847],
                orientation=[16.736338, 2.5316212, 89.90968],
                position_delta=[0.03, 0.03, 0.03],
                orientation_delta=[1, 1, 1],
            ),
            dict(
                name="wrist_cam",
                position=[0.012212855875360258, 0.1438242841130981, 0.07214914261067958],
                orientation=[128.89171, 0.80611223, 176.2427],
                parent_prim=wrist_parent,
                focal_length=13.0,
            ),
        ]
        for kwargs in defaults:
            try:
                self.add_editor_camera(kwargs)
            except Exception as e:
                print(f"[EDITOR][WARN] default camera '{kwargs['name']}' 실패: {e}")

    def _add_default_lights(self):
        """기본 DomeLight 하나 — UI에서 수정/삭제 가능."""
        try:
            self.add_editor_light(dict(
                type="dome", name="DomeLight",
                intensity=1650, intensity_delta=350,
                color_temp=6000, color_temp_delta=500,
            ))
        except Exception as e:
            print(f"[EDITOR][WARN] default light 실패: {e}")

    # ── Load / Clear ─────────────────────────────────────

    def clear_editor_state(self):
        """모든 오브젝트/카메라/조명 제거. 테이블과 로봇은 유지."""
        for spec in list(self._object_specs):
            try:
                self.remove_editor_object(spec["name"])
            except Exception as e:
                print(f"[EDITOR][WARN] remove object 실패: {e}")
        for spec in list(self._camera_specs):
            try:
                self.remove_editor_camera(spec["name"])
            except Exception as e:
                print(f"[EDITOR][WARN] remove camera 실패: {e}")
        for spec in list(self._light_specs):
            try:
                self.remove_editor_light(spec["name"])
            except Exception as e:
                print(f"[EDITOR][WARN] remove light 실패: {e}")

    def load_env_file(self, path: str) -> str:
        """envs/*.py 파일을 파싱해 editor 상태로 로드.

        Returns:
            str: 파싱된 클래스 이름 (Save 섹션 기본값 업데이트용).
        """
        specs = _parse_env_file(path)
        self.clear_editor_state()
        print(f"[EDITOR] 로드: objects={len(specs['objects'])}, "
              f"cameras={len(specs['cameras'])}, lights={len(specs['lights'])}")
        for kwargs in specs["objects"]:
            try:
                self.add_editor_object(kwargs)
            except Exception as e:
                print(f"[EDITOR][WARN] load object '{kwargs.get('name')}' 실패: {e}")
        for kwargs in specs["cameras"]:
            try:
                self.add_editor_camera(kwargs)
            except Exception as e:
                print(f"[EDITOR][WARN] load camera '{kwargs.get('name')}' 실패: {e}")
        for kwargs in specs["lights"]:
            try:
                self.add_editor_light(kwargs)
            except Exception as e:
                print(f"[EDITOR][WARN] load light '{kwargs.get('name')}' 실패: {e}")
        classname = specs.get("classname", "MyEnv")
        if self._panel is not None:
            self._panel.refresh_all_lists()
            self._panel.set_save_defaults(
                classname=classname, filename=Path(path).name,
            )
        return classname

    # ── UI 콜백용 API: 오브젝트 ──────────────────────────

    def editor_object_specs(self) -> list[dict]:
        return list(self._object_specs)

    def add_editor_object(self, kwargs: dict):
        """UI에서 호출: ObjectManager에 오브젝트 추가 + 위치 설정 + spec 저장.

        position_delta / orientation_delta / scale_delta가 지정되면
        position ± delta 범위로 랜덤 배치. position_range / orientation_range는
        레거시 포맷이지만 호환을 위해 계속 받는다 (delta로 변환).
        """
        if self._obj_manager is None:
            raise RuntimeError("add_table()이 먼저 실행되어야 합니다.")

        for spec in self._object_specs:
            if spec.get("name") == kwargs.get("name"):
                raise ValueError(f"이름 중복: {kwargs.get('name')}")

        merged = dict(kwargs)

        # 레거시 position_range → position + position_delta 변환
        pr_legacy = merged.pop("position_range", None)
        if pr_legacy is not None and "position" not in merged:
            mid = [0.5 * (pr_legacy[0][i] + pr_legacy[1][i]) for i in range(3)]
            delta = [0.5 * abs(pr_legacy[1][i] - pr_legacy[0][i]) for i in range(3)]
            merged["position"] = tuple(float(v) for v in mid)
            if any(d > 1e-9 for d in delta):
                merged["position_delta"] = tuple(float(d) for d in delta)
        orr_legacy = merged.pop("orientation_range", None)
        if orr_legacy is not None and "orientation" not in merged:
            mid = [0.5 * (orr_legacy[0][i] + orr_legacy[1][i]) for i in range(3)]
            delta = [0.5 * abs(orr_legacy[1][i] - orr_legacy[0][i]) for i in range(3)]
            merged["orientation"] = tuple(float(v) for v in mid)
            if any(d > 1e-9 for d in delta):
                merged["orientation_delta"] = tuple(float(d) for d in delta)

        position = merged.get("position")
        orientation = merged.get("orientation")
        position_delta = merged.get("position_delta")
        orientation_delta = merged.get("orientation_delta")
        scale_delta = merged.get("scale_delta")
        scale = float(merged.get("scale") or 1.0)

        # delta → range 계산 (ObjectManager 내부에서 사용)
        position_range = None
        if position is not None and position_delta is not None:
            position_range = [
                [float(position[i]) - float(position_delta[i]) for i in range(3)],
                [float(position[i]) + float(position_delta[i]) for i in range(3)],
            ]
        orientation_range = None
        if orientation_delta is not None:
            base_ori = orientation if orientation is not None else (0.0, 0.0, 0.0)
            orientation_range = [
                [float(base_ori[i]) - float(orientation_delta[i]) for i in range(3)],
                [float(base_ori[i]) + float(orientation_delta[i]) for i in range(3)],
            ]
        scale_range = None
        if scale_delta is not None:
            sd = float(scale_delta)
            if sd > 1e-9:
                scale_range = (max(1e-3, scale - sd), scale + sd)

        # `_` 프리픽스 및 delta 키는 editor 메타데이터 — ObjectManager.add_object에
        # 넘기기 전에 제거 (converted range만 전달).
        inner_kwargs = {k: v for k, v in merged.items()
                        if not k.startswith("_") and k not in {
                            "position_delta", "orientation_delta", "scale_delta"
                        }}
        inner_kwargs["position_range"] = position_range
        inner_kwargs["orientation_range"] = orientation_range
        inner_kwargs["scale_range"] = scale_range

        obj = self._obj_manager.add_object(**inner_kwargs)

        # 초기 위치: position 우선, 없으면 range에서 샘플, 최후에는 테이블 중앙 기본값.
        if position is not None:
            cx, cy, cz = float(position[0]), float(position[1]), float(position[2])
        elif position_range is not None:
            cx = float(np.random.uniform(position_range[0][0], position_range[1][0]))
            cy = float(np.random.uniform(position_range[0][1], position_range[1][1]))
            cz = float(np.random.uniform(position_range[0][2], position_range[1][2]))
        else:
            cx, cy = 0.3, 0.0
            cz = self._obj_manager._surface_z + obj.half_height + 0.05

        obj.set_position(np.array([cx, cy, cz]))

        if orientation is not None:
            obj.set_orientation(tuple(float(o) for o in orientation))
        elif orientation_range is not None:
            mid = tuple((orientation_range[0][i] + orientation_range[1][i]) / 2.0
                        for i in range(3))
            obj.set_orientation(mid)

        self._reset_rigid_body_velocity(obj.prim_path)
        self._register_marker_pub(obj)

        # 저장되는 spec은 delta 포맷 (range는 유지하지 않음).
        save_spec = dict(kwargs)
        save_spec.pop("position_range", None)
        save_spec.pop("orientation_range", None)
        if "position" not in save_spec and position is not None:
            save_spec["position"] = tuple(float(v) for v in position)
        if "orientation" not in save_spec and orientation is not None:
            save_spec["orientation"] = tuple(float(v) for v in orientation)
        if position_delta is not None:
            save_spec["position_delta"] = tuple(float(v) for v in position_delta)
        if orientation_delta is not None:
            save_spec["orientation_delta"] = tuple(float(v) for v in orientation_delta)
        if scale_delta is not None:
            save_spec["scale_delta"] = float(scale_delta)

        self._object_specs.append(save_spec)
        print(f"[EDITOR] 오브젝트 추가: {save_spec.get('name')} "
              f"pos={position} pos_delta={position_delta}")

    def remove_editor_object(self, name: str) -> bool:
        if self._obj_manager is None:
            return False

        idx = next((i for i, s in enumerate(self._object_specs) if s.get("name") == name), -1)
        if idx < 0:
            return False

        obj_idx = next((i for i, o in enumerate(self._obj_manager.objects) if o.name == name), -1)
        if obj_idx >= 0:
            obj = self._obj_manager.objects.pop(obj_idx)
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            stage.RemovePrim(obj.prim_path)

        pubs = self._obj_manager._marker_pubs
        if name in pubs:
            try:
                self._obj_manager._ros_node.destroy_publisher(pubs[name])
            except Exception:
                pass
            del pubs[name]

        self._object_specs.pop(idx)
        return True

    def update_editor_object(self, name: str, **changes):
        """오브젝트의 임의 필드를 실시간 업데이트.

        지원 필드:
            position, orientation            (XYZ live 반영)
            position_delta, orientation_delta (±delta, spec + range 동기화)
            scale_delta                      (±delta, randomize 시 scale 샘플)
            size, height, scale              (geom attribute 직접 수정)
            color                            (DisplayColor / USD shader)
            mass, friction                   (RigidBody/PhysicsMaterial)
        """
        if self._obj_manager is None:
            raise RuntimeError("ObjectManager가 초기화되지 않았습니다.")

        spec = next((s for s in self._object_specs if s.get("name") == name), None)
        obj = next((o for o in self._obj_manager.objects if o.name == name), None)
        if spec is None or obj is None:
            raise ValueError(f"오브젝트를 찾을 수 없음: {name}")

        position = changes.pop("position", None)
        orientation = changes.pop("orientation", None)
        position_delta = changes.pop("position_delta", None)
        orientation_delta = changes.pop("orientation_delta", None)
        scale_delta = changes.pop("scale_delta", None)
        size = changes.pop("size", None)
        height = changes.pop("height", None)
        scale = changes.pop("scale", None)
        color = changes.pop("color", None)
        mass = changes.pop("mass", None)
        friction = changes.pop("friction", None)
        if changes:
            print(f"[EDITOR][WARN] update_editor_object 무시된 키: {list(changes)}")

        if position is not None:
            pos_t = tuple(float(p) for p in position)
            obj.set_position(np.array(pos_t))
            obj.fixed_position = pos_t
            self._reset_rigid_body_velocity(obj.prim_path)
            spec["position"] = pos_t

        if orientation is not None:
            ori_t = tuple(float(o) for o in orientation)
            obj.set_orientation(ori_t)
            obj.fixed_orientation = ori_t
            self._reset_rigid_body_velocity(obj.prim_path)
            spec["orientation"] = ori_t

        if position_delta is not None:
            pd = tuple(float(v) for v in position_delta)
            spec["position_delta"] = pd
            base = spec.get("position") or obj.fixed_position or (0.0, 0.0, 0.0)
            if any(v > 1e-9 for v in pd):
                obj.position_range = [
                    [float(base[i]) - pd[i] for i in range(3)],
                    [float(base[i]) + pd[i] for i in range(3)],
                ]
            else:
                obj.position_range = None

        if orientation_delta is not None:
            od = tuple(float(v) for v in orientation_delta)
            spec["orientation_delta"] = od
            base = spec.get("orientation") or obj.fixed_orientation or (0.0, 0.0, 0.0)
            if any(v > 1e-9 for v in od):
                obj.orientation_range = [
                    [float(base[i]) - od[i] for i in range(3)],
                    [float(base[i]) + od[i] for i in range(3)],
                ]
            else:
                obj.orientation_range = None

        if scale_delta is not None:
            sd = float(scale_delta)
            spec["scale_delta"] = sd
            base_scale = float(spec.get("scale") or 1.0)
            if sd > 1e-9:
                obj.scale_range = (max(1e-3, base_scale - sd), base_scale + sd)
            else:
                obj.scale_range = None

        if size is not None:
            obj.set_size(float(size))
            spec["size"] = float(size)

        if height is not None:
            obj.set_height(float(height))
            spec["height"] = float(height)

        if scale is not None:
            s = float(scale)
            obj.set_scale(s)
            spec["scale"] = s
            # scale 바뀌면 scale_range도 base 재계산
            if spec.get("scale_delta"):
                sd = float(spec["scale_delta"])
                obj.scale_range = (max(1e-3, s - sd), s + sd)

        if color is not None:
            c = tuple(float(v) for v in color)
            obj.set_color(c)
            spec["color"] = c

        if mass is not None:
            m = float(mass)
            obj.set_mass(m)
            spec["mass"] = m

        if friction is not None:
            f = float(friction)
            obj.set_friction(f)
            spec["friction"] = f

        # position 혹은 orientation이 바뀌면 delta-based range도 base 이동
        if position is not None and spec.get("position_delta"):
            pd = spec["position_delta"]
            obj.position_range = [
                [float(position[i]) - float(pd[i]) for i in range(3)],
                [float(position[i]) + float(pd[i]) for i in range(3)],
            ]
        if orientation is not None and spec.get("orientation_delta"):
            od = spec["orientation_delta"]
            obj.orientation_range = [
                [float(orientation[i]) - float(od[i]) for i in range(3)],
                [float(orientation[i]) + float(od[i]) for i in range(3)],
            ]

    def recreate_editor_object(self, old_name: str, new_kwargs: dict) -> str:
        """오브젝트를 destroy+recreate. shape/usd_path/name 등 비(非)라이브 필드 변경용.

        new_kwargs에서 지정하지 않은 필드는 기존 spec을 유지한다.
        position/orientation은 기존 sim 상태를 우선하여 보존한다.
        이름이 바뀌는 경우 반환값은 새 이름.
        """
        spec = next((s for s in self._object_specs if s.get("name") == old_name), None)
        if spec is None:
            raise ValueError(f"오브젝트를 찾을 수 없음: {old_name}")

        # 현재 sim 위치/회전을 보존 (spec의 fixed값보다 최신 상태)
        obj = next((o for o in self._obj_manager.objects if o.name == old_name), None)
        cur_pos: tuple | None = None
        cur_ori: tuple | None = None
        if obj is not None:
            p = obj.get_position()
            cur_pos = (float(p[0]), float(p[1]), float(p[2]))
            cur_ori = obj.fixed_orientation

        merged = dict(spec)
        merged.update(new_kwargs)

        # position은 new_kwargs > 현재 sim 상태 > spec.position 순으로 해결
        if "position" not in new_kwargs and cur_pos is not None:
            merged["position"] = cur_pos
        if "orientation" not in new_kwargs and cur_ori is not None:
            merged["orientation"] = cur_ori

        self.remove_editor_object(old_name)
        self.add_editor_object(merged)
        return merged.get("name", old_name)

    def _reset_rigid_body_velocity(self, prim_path: str):
        import omni.usd
        from pxr import UsdPhysics, Gf
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rb = UsdPhysics.RigidBodyAPI(prim)
            try:
                rb.CreateVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
                rb.CreateAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
            except Exception as e:
                print(f"[EDITOR][WARN] velocity reset failed: {e}")

    def _register_marker_pub(self, obj):
        om = self._obj_manager
        if om._ros_node is None:
            if self._robot_prim_path is None:
                return
            om.setup_marker_publisher(self._robot_prim_path)
            return
        from visualization_msgs.msg import Marker
        topic = f"/simulation/object_markers/{obj.name}"
        om._marker_pubs[obj.name] = om._ros_node.create_publisher(Marker, topic, 10)

    # ── UI 콜백용 API: 카메라 ────────────────────────────

    def editor_camera_specs(self) -> list[dict]:
        return list(self._camera_specs)

    def add_editor_camera(self, kwargs: dict):
        """UI에서 호출: add_camera 실행 + spec 저장. name 중복 체크."""
        name = kwargs.get("name")
        if not name:
            raise ValueError("카메라 이름이 필요합니다.")
        for spec in self._camera_specs:
            if spec.get("name") == name:
                raise ValueError(f"이름 중복: {name}")

        self.add_camera(**kwargs)
        self._camera_specs.append(dict(kwargs))
        print(f"[EDITOR] 카메라 추가: {name}")

    def remove_editor_camera(self, name: str) -> bool:
        """카메라 삭제: USD prim + self._cameras + spec."""
        idx = next((i for i, s in enumerate(self._camera_specs) if s.get("name") == name), -1)
        if idx < 0:
            return False

        cam_idx = next((i for i, c in enumerate(self._cameras) if c.get("prim_path", "").endswith("/" + name)), -1)
        if cam_idx >= 0:
            cam_info = self._cameras.pop(cam_idx)
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            stage.RemovePrim(cam_info["prim_path"])

        self._camera_specs.pop(idx)
        return True

    def update_editor_camera(self, name: str, **changes):
        """카메라 라이브 업데이트.

        지원 필드:
            position, orientation, focal_length                     (sim 즉시 반영)
            position_delta, orientation_delta                       (spec만 갱신)
        """
        import omni.usd
        from pxr import Gf, UsdGeom
        stage = omni.usd.get_context().get_stage()

        spec = next((s for s in self._camera_specs if s.get("name") == name), None)
        cam_info = next((c for c in self._cameras
                         if c.get("prim_path", "").endswith("/" + name)), None)
        if spec is None or cam_info is None:
            raise ValueError(f"카메라를 찾을 수 없음: {name}")

        prim = stage.GetPrimAtPath(cam_info["prim_path"])
        if not prim.IsValid():
            raise RuntimeError(f"prim invalid: {cam_info['prim_path']}")
        xform = UsdGeom.Xformable(prim)

        position = changes.pop("position", None)
        orientation = changes.pop("orientation", None)
        focal_length = changes.pop("focal_length", None)
        position_delta = changes.pop("position_delta", None)
        orientation_delta = changes.pop("orientation_delta", None)
        if changes:
            print(f"[EDITOR][WARN] update_editor_camera 무시된 키: {list(changes)}")

        if position is not None:
            for op in xform.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:translate":
                    op.Set(Gf.Vec3d(*position))
                    break
            spec["position"] = list(position)
            if cam_info.get("base_position") is not None:
                cam_info["base_position"] = list(position)

        if orientation is not None:
            found = False
            for op in xform.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:rotateXYZ":
                    op.Set(Gf.Vec3f(*orientation))
                    found = True
                    break
            if not found:
                xform.AddRotateXYZOp().Set(Gf.Vec3f(*orientation))
            spec["orientation"] = list(orientation)
            if cam_info.get("base_orientation") is not None:
                cam_info["base_orientation"] = list(orientation)

        if focal_length is not None:
            cam_prim = UsdGeom.Camera(prim)
            cam_prim.CreateFocalLengthAttr().Set(float(focal_length))
            spec["focal_length"] = float(focal_length)

        if position_delta is not None:
            spec["position_delta"] = [float(v) for v in position_delta]

        if orientation_delta is not None:
            spec["orientation_delta"] = [float(v) for v in orientation_delta]

    def recreate_editor_camera(self, old_name: str, new_kwargs: dict) -> str:
        """카메라 destroy+recreate. name/parent_prim/freq/publish_* 등 비라이브 필드 변경용."""
        spec = next((s for s in self._camera_specs if s.get("name") == old_name), None)
        if spec is None:
            raise ValueError(f"카메라를 찾을 수 없음: {old_name}")
        merged = dict(spec)
        merged.update(new_kwargs)
        self.remove_editor_camera(old_name)
        self.add_editor_camera(merged)
        return merged.get("name", old_name)

    # ── UI 콜백용 API: 조명 ──────────────────────────────

    def editor_light_specs(self) -> list[dict]:
        return list(self._light_specs)

    def add_editor_light(self, kwargs: dict):
        name = kwargs.get("name")
        if not name:
            raise ValueError("조명 이름이 필요합니다.")
        for spec in self._light_specs:
            if spec.get("name") == name:
                raise ValueError(f"이름 중복: {name}")

        self.add_light(**kwargs)
        self._light_specs.append(dict(kwargs))
        print(f"[EDITOR] 조명 추가: {name}")

    def remove_editor_light(self, name: str) -> bool:
        idx = next((i for i, s in enumerate(self._light_specs) if s.get("name") == name), -1)
        if idx < 0:
            return False

        light_idx = next((i for i, l in enumerate(self._lights) if l.get("name") == name), -1)
        if light_idx >= 0:
            light = self._lights.pop(light_idx)
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            stage.RemovePrim(light["prim_path"])

        self._light_specs.pop(idx)
        return True

    def update_editor_light(self, name: str, **changes):
        """조명 라이브 업데이트.

        지원 필드:
            intensity, color_temp, orientation                  (sim 즉시 반영)
            intensity_delta, color_temp_delta, orientation_delta (spec만 갱신)
        """
        import omni.usd
        from pxr import Gf, UsdGeom
        stage = omni.usd.get_context().get_stage()

        spec = next((s for s in self._light_specs if s.get("name") == name), None)
        light = next((l for l in self._lights if l.get("name") == name), None)
        if spec is None or light is None:
            raise ValueError(f"조명을 찾을 수 없음: {name}")

        prim = stage.GetPrimAtPath(light["prim_path"])
        if not prim.IsValid():
            raise RuntimeError(f"prim invalid: {light['prim_path']}")

        intensity = changes.pop("intensity", None)
        color_temp = changes.pop("color_temp", None)
        orientation = changes.pop("orientation", None)
        intensity_delta = changes.pop("intensity_delta", None)
        color_temp_delta = changes.pop("color_temp_delta", None)
        orientation_delta = changes.pop("orientation_delta", None)
        if changes:
            print(f"[EDITOR][WARN] update_editor_light 무시된 키: {list(changes)}")

        if intensity is not None:
            prim.GetAttribute("inputs:intensity").Set(float(intensity))
            spec["intensity"] = float(intensity)
            light["base_intensity"] = float(intensity)

        if color_temp is not None:
            prim.GetAttribute("inputs:enableColorTemperature").Set(True)
            prim.GetAttribute("inputs:colorTemperature").Set(float(color_temp))
            spec["color_temp"] = float(color_temp)
            light["base_color_temp"] = float(color_temp)

        if orientation is not None and light["type"] == "distant":
            xform = UsdGeom.Xformable(prim)
            found = False
            for op in xform.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:rotateXYZ":
                    op.Set(Gf.Vec3f(*orientation))
                    found = True
                    break
            if not found:
                xform.AddRotateXYZOp().Set(Gf.Vec3f(*orientation))
            spec["orientation"] = list(orientation)
            light["base_orientation"] = list(orientation)

        if intensity_delta is not None:
            spec["intensity_delta"] = float(intensity_delta)

        if color_temp_delta is not None:
            spec["color_temp_delta"] = float(color_temp_delta)

        if orientation_delta is not None:
            spec["orientation_delta"] = [float(v) for v in orientation_delta]

    def recreate_editor_light(self, old_name: str, new_kwargs: dict) -> str:
        """조명 destroy+recreate. name/type 등 비라이브 필드 변경용."""
        spec = next((s for s in self._light_specs if s.get("name") == old_name), None)
        if spec is None:
            raise ValueError(f"조명을 찾을 수 없음: {old_name}")
        merged = dict(spec)
        merged.update(new_kwargs)
        self.remove_editor_light(old_name)
        self.add_editor_light(merged)
        return merged.get("name", old_name)

    # ── 템플릿 파일 생성 ──────────────────────────────────

    def save_env_template(self, filepath: str, classname: str) -> str:
        fname = Path(filepath).name
        lines = _render_template(classname, self._object_specs,
                                 self._camera_specs, self._light_specs, fname)
        Path(filepath).write_text(lines, encoding="utf-8")
        print(f"[EDITOR] 저장 완료: {filepath}")
        return filepath


# ─────────────────────────────────────────────────────────
# 템플릿 렌더링
# ─────────────────────────────────────────────────────────

def _parse_env_file(path: str) -> dict:
    """envs/*.py 파일에서 add_object/add_camera/add_light 호출을 추출.

    모듈 네임스페이스에서 상수 (PIPER_URDF_V100, USD_DIR 등)를 먼저 실행해
    `str(USD_DIR / "foo.usd")` 같은 표현식도 평가할 수 있게 한다.
    """
    import ast

    source = Path(path).read_text(encoding="utf-8")
    tree = ast.parse(source)

    # 모듈 레벨 네임스페이스에 상수 실행
    module_ns: dict = {
        "__file__": path,
        "__name__": "__env_parser__",
        "Path": Path,
        "str": str,
        "sys": sys,
    }
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            continue
        if isinstance(node, ast.If):
            # `if __name__ == "__main__":` 스킵
            test = node.test
            if (isinstance(test, ast.Compare) and isinstance(test.left, ast.Name)
                    and test.left.id == "__name__"):
                continue
        try:
            code = compile(ast.Module(body=[node], type_ignores=[]), path, "exec")
            exec(code, module_ns)
        except Exception:
            pass  # best-effort (import 실패 등은 무시)

    # setup_scene을 가진 클래스 찾기
    target_cls = None
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == "setup_scene":
                    target_cls = node
                    break
        if target_cls is not None:
            break
    if target_cls is None:
        raise ValueError(f"{path}에 setup_scene 메서드를 가진 클래스가 없습니다.")

    setup = next(item for item in target_cls.body
                 if isinstance(item, ast.FunctionDef) and item.name == "setup_scene")

    method_map = {"add_object": "objects", "add_camera": "cameras", "add_light": "lights"}
    specs: dict = {"objects": [], "cameras": [], "lights": [], "classname": target_cls.name}

    for node in ast.walk(setup):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not (isinstance(func, ast.Attribute) and isinstance(func.value, ast.Name)
                and func.value.id == "self" and func.attr in method_map):
            continue

        kwargs = {}
        nucleus_relpath: str | None = None
        failed = False
        for kw in node.keywords:
            if kw.arg is None:
                continue
            # f"{ASSETS_ROOT}/rel/path" 패턴 감지 (Nucleus 에셋)
            if kw.arg == "usd_path":
                rel = _extract_nucleus_relpath(kw.value)
                if rel is not None:
                    nucleus_relpath = rel
            try:
                value = ast.literal_eval(kw.value)
            except (ValueError, SyntaxError):
                try:
                    expr = ast.Expression(kw.value)
                    value = eval(compile(expr, path, "eval"), module_ns)
                except Exception as e:
                    print(f"[PARSE][WARN] {func.attr} kwarg '{kw.arg}' 평가 실패: {e}")
                    failed = True
                    break
            kwargs[kw.arg] = value

        if not failed:
            if nucleus_relpath is not None:
                kwargs["_nucleus_relpath"] = nucleus_relpath
            specs[method_map[func.attr]].append(kwargs)

    return specs


def _extract_nucleus_relpath(expr) -> str | None:
    """f"{ASSETS_ROOT}/rel/path" 형태의 JoinedStr에서 'rel/path' 추출. 아니면 None."""
    import ast

    if not isinstance(expr, ast.JoinedStr):
        return None
    values = expr.values
    if len(values) < 2:
        return None
    first, second = values[0], values[1]
    # generator 템플릿은 ASSETS_ROOT, my_env.py 등 수기 작성 파일은 assets_root 사용.
    if not (isinstance(first, ast.FormattedValue)
            and isinstance(first.value, ast.Name)
            and first.value.id in ("ASSETS_ROOT", "assets_root")):
        return None
    if not (isinstance(second, ast.Constant) and isinstance(second.value, str)):
        return None
    # 나머지 세그먼트(상수)가 있다면 이어붙임
    tail = second.value
    for v in values[2:]:
        if isinstance(v, ast.Constant) and isinstance(v.value, str):
            tail += v.value
        else:
            return None  # 동적 조각이 더 있으면 안전하게 포기
    return tail.lstrip("/")


def _py_repr(value):
    if isinstance(value, float):
        return f"{value!r}"
    if isinstance(value, tuple):
        return "(" + ", ".join(_py_repr(v) for v in value) + ")"
    if isinstance(value, list):
        return "[" + ", ".join(_py_repr(v) for v in value) + "]"
    if isinstance(value, dict):
        return "{" + ", ".join(f"{k!r}: {_py_repr(v)}" for k, v in value.items()) + "}"
    return repr(value)


def _render_call(method: str, spec: dict, key_order: list[str]) -> str:
    parts = []
    for k in key_order:
        if k in spec and spec[k] is not None:
            parts.append(f"{k}={_py_repr(spec[k])}")
    return f"        self.{method}(" + ", ".join(parts) + ")"


def _render_add_object_call(spec: dict) -> str:
    key_order = [
        "name", "size", "color", "shape", "height", "usd_path", "scale",
        "box_size", "position", "orientation",
        "position_delta", "orientation_delta", "scale_delta",
        "wrap_rigidbody", "deformable",
        "collision_approximation", "friction", "mass",
    ]
    nucleus_rel = spec.get("_nucleus_relpath")
    # 의미 없는 필드 스킵: orientation=(0,0,0)에 delta도 없으면 굳이 저장 안함
    # (transform op 트리거되어 GUI 드래그가 어려워지는 것을 방지)
    ori = spec.get("orientation")
    ori_delta = spec.get("orientation_delta")
    skip_ori = (ori is not None
                and all(abs(float(v)) < 1e-9 for v in ori)
                and (ori_delta is None or all(abs(float(v)) < 1e-9 for v in ori_delta)))
    parts = []
    for k in key_order:
        if k not in spec or spec[k] is None:
            continue
        if k == "orientation" and skip_ori:
            continue
        if k == "usd_path" and nucleus_rel:
            parts.append(f'usd_path=f"{{ASSETS_ROOT}}/{nucleus_rel}"')
        else:
            parts.append(f"{k}={_py_repr(spec[k])}")
    return f"        self.add_object(" + ", ".join(parts) + ")"


def _render_add_camera_call(spec: dict) -> str:
    key_order = [
        "name", "position", "orientation", "resolution", "freq",
        "publish_rgb", "publish_depth", "publish_pointcloud",
        "parent_prim", "near_plane", "focal_length",
        "position_delta", "orientation_delta",
    ]
    return _render_call("add_camera", spec, key_order)


def _render_add_light_call(spec: dict) -> str:
    key_order = [
        "type", "name", "intensity", "color_temp", "orientation",
        "intensity_delta", "color_temp_delta", "orientation_delta",
    ]
    return _render_call("add_light", spec, key_order)


def _render_template(classname: str, object_specs: list[dict],
                     camera_specs: list[dict], light_specs: list[dict],
                     filename: str = "my_env.py") -> str:
    table = DEFAULT_TABLE
    add_object_calls = "\n".join(_render_add_object_call(s) for s in object_specs)
    if not add_object_calls:
        add_object_calls = "        # (오브젝트 없음)"

    add_camera_calls = "\n".join(_render_add_camera_call(s) for s in camera_specs)
    if not add_camera_calls:
        add_camera_calls = "        # (카메라 없음)"

    add_light_calls = "\n".join(_render_add_light_call(s) for s in light_specs)
    if not add_light_calls:
        add_light_calls = "        # (조명 없음 — BaseEnv가 기본 DomeLight 자동 추가)"

    # Nucleus 에셋이 하나라도 있으면 모듈 레벨에서 ASSETS_ROOT 해석.
    has_nucleus = any(s.get("_nucleus_relpath") for s in object_specs)
    nucleus_block = ""
    if has_nucleus:
        nucleus_block = (
            "\n"
            "try:\n"
            "    from isaacsim.storage.native import get_assets_root_path as _get_assets_root\n"
            "except ImportError:\n"
            "    from omni.isaac.core.utils.nucleus import get_assets_root_path as _get_assets_root\n"
            "ASSETS_ROOT = _get_assets_root()\n"
        )

    template = f'''"""
환경 편집기로 생성된 환경.

Usage:
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/{filename}
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/{filename} --headless
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ros2_env.base_env import BaseEnv

PIPER_PKG = Path(__file__).resolve().parents[2] / "ros_pkgs/piper_description"
PIPER_URDF_V100 = str(PIPER_PKG / "urdf/piper_description_v100.urdf")
PIPER_URDF_STD = str(PIPER_PKG / "urdf/piper_description.urdf")
USD_DIR = Path(__file__).resolve().parents[2] / "custom_usds/usd"
{nucleus_block}


class {classname}(BaseEnv):

    def __init__(self, headless=False, use_v100=True):
        self._use_v100 = use_v100
        super().__init__(headless=headless)

    def setup_scene(self):
        self.add_table(center={_py_repr(table["center"])}, scale={_py_repr(table["scale"])},
                       color={_py_repr(table["color"])}, spawn_range={_py_repr(table["spawn_range"])})

        urdf = PIPER_URDF_V100 if self._use_v100 else PIPER_URDF_STD
        self.add_robot_from_urdf(
            name="piper",
            urdf_path=urdf,
            prim_path="/World/Piper",
            position={_py_repr(DEFAULT_ROBOT_POSITION)},
            color={_py_repr(DEFAULT_ROBOT_COLOR)},
        )
        self.add_mimic_joint("joint8", "joint7", multiplier=-1.0)

        # ── 오브젝트 ──
{add_object_calls}

        # ── 카메라 ──
{add_camera_calls}

        # ── 조명 ──
{add_light_calls}

        self.set_background_randomization(brightness_range=[0.0, 0.3])


if __name__ == "__main__":
    headless = "--headless" in sys.argv
    use_v100 = "--standard" not in sys.argv
    {classname}(headless=headless, use_v100=use_v100).run()
'''
    return template


if __name__ == "__main__":
    headless = "--headless" in sys.argv
    use_v100 = "--standard" not in sys.argv
    EnvEditor(headless=headless, use_v100=use_v100).run()
