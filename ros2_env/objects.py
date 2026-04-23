"""
테이블 위 오브젝트 생성, 랜덤 배치, ROS2 퍼블리쉬.

isaacsim import는 함수/메서드 내부에서 수행.
"""

import numpy as np


class TargetObject:
    """씬 오브젝트. 프리미티브 / USD 에셋 / 장식용 box 모두 지원.

    물리 처리(wrap_rigidbody):
        None  = 자동 감지. box_size 또는 articulated USD면 False, 그 외 True.
        True  = 최상위 prim에 RigidBodyAPI + CollisionAPI + PhysicsMaterial 부착.
                단일 강체로 취급 (기본 조작 대상 오브젝트).
        False = USD 내부 물리 API를 그대로 유지. articulated 에셋(서랍장/노트북 등)
                또는 순수 장식용에 사용.

    배치:
        position  지정 → 해당 위치 고정, randomize 대상에서 제외.
        position_range 지정 → 해당 범위 내 랜덤 배치.
        둘 다 None → 테이블 기본 spawn 범위 내 랜덤.

    Args:
        usd_path: USD 에셋 경로. 지정 시 shape 무시.
        box_size: (x, y, z) 직육면체 크기. 지정 시 usd_path 무시.
        position: (x, y, z) 고정 월드 좌표.
        orientation: (rx, ry, rz) 도 단위 고정 회전.
        position_range: [[x_min, y_min, z], [x_max, y_max, z]]. None이면 테이블 기본 범위.
        orientation_range: [[rx_min, ry_min, rz_min], [rx_max, ry_max, rz_max]] 도 단위.
        wrap_rigidbody: 최상위 RigidBody 래핑 여부. None이면 자동 감지.
        scale: USD 에셋 스케일 (기본 1.0).
    """

    def __init__(self, name: str, stage, size: float | None = None,
                 color: tuple | None = None, shape: str = "cube",
                 height: float | None = None,
                 usd_path: str | None = None, scale: float = 1.0,
                 box_size: tuple | None = None,
                 position: tuple | None = None,
                 orientation: tuple | None = None,
                 position_range: list | None = None,
                 orientation_range: list | None = None,
                 wrap_rigidbody: bool | None = None,
                 deformable: float | None = None,
                 collision_approximation: str = "convexHull",
                 friction: float = 1.0,
                 mass: float | None = None):
        from pxr import Usd, UsdGeom, UsdPhysics, Gf

        # 기본 색 (USD는 원본 유지하므로 None)
        if color is None and usd_path is None:
            color = (0.5, 0.5, 0.5) if box_size is not None else (1.0, 0.3, 0.2)

        self.name = name
        self.prim_path = f"/World/{name}"
        self.color = color
        self._stage = stage
        self.position_range = position_range
        self.orientation_range = orientation_range
        self.scale_range: tuple[float, float] | None = None
        self.fixed_position = tuple(position) if position is not None else None
        self.fixed_orientation = tuple(orientation) if orientation is not None else None
        self._scale = scale
        self._box_size = box_size

        # ── 지오메트리 생성 ────────────────────────────────────
        if box_size is not None:
            # 장식용 직육면체 (scale로 크기 조절)
            geom = UsdGeom.Cube.Define(stage, self.prim_path)
            geom.GetSizeAttr().Set(1.0)
            if color is not None:
                geom.CreateDisplayColorAttr([color])
            self.half_height = box_size[2] / 2.0
            self._geom_kind = "box"
            if size is None:
                size = float(max(box_size))  # 마커/충돌 거리용 근사

        elif usd_path is not None:
            # Xform 타입 필수 — untyped prim에는 RigidBodyAPI를 적용할 수 없다.
            prim = stage.DefinePrim(self.prim_path, "Xform")
            prim.GetReferences().AddReference(usd_path)
            if size is None:
                size = self._compute_usd_size(prim, scale)
                print(f"[INFO] {name}: USD size 자동 계산 → {size:.4f}m")
            self.half_height = size / 2.0
            self._geom_kind = "usd"
            self._usd_path = usd_path

        else:
            # 프리미티브 cube/sphere/cylinder
            if size is None:
                size = 0.06
            if shape == "cube":
                geom = UsdGeom.Cube.Define(stage, self.prim_path)
                geom.GetSizeAttr().Set(size)
                self.half_height = size / 2.0
            elif shape == "sphere":
                geom = UsdGeom.Sphere.Define(stage, self.prim_path)
                geom.GetRadiusAttr().Set(size / 2.0)
                self.half_height = size / 2.0
            elif shape == "cylinder":
                actual_height = height if height is not None else size
                geom = UsdGeom.Cylinder.Define(stage, self.prim_path)
                geom.GetRadiusAttr().Set(size / 2.0)
                geom.GetHeightAttr().Set(actual_height)
                self.half_height = actual_height / 2.0
            if color is not None:
                geom.CreateDisplayColorAttr([color])
            self._geom_kind = "prim"

        self.size = size

        prim = stage.GetPrimAtPath(self.prim_path)
        self._xform = UsdGeom.Xformable(prim)

        # ── wrap_rigidbody 자동 감지 ───────────────────────────
        if wrap_rigidbody is None:
            if self._geom_kind == "box":
                wrap_rigidbody = False
            elif self._geom_kind == "usd":
                # stage composition + 레이어 직접 검사 양쪽 시도.
                # instanceable USD는 stage traversal에서 master 내부가 안 보이므로
                # Sdf.Layer로 USD를 직접 열어 articulation을 확인한다.
                has_art = self._has_articulation(prim) or \
                          self._usd_has_articulation(usd_path)
                wrap_rigidbody = not has_art
                if has_art:
                    print(f"[INFO] {self.name}: articulated USD 감지 → RigidBody 래핑 스킵")
            else:
                wrap_rigidbody = True
        self._wrap_rigidbody = wrap_rigidbody

        # ── xform op 설정 ──────────────────────────────────────
        # box_size(장식용)는 축별 스케일이 필요하므로 단일 transform 매트릭스 op 사용.
        # 그 외(rigid body 오브젝트)는 translate + orient + scale 분리 op 사용 —
        # PhysX가 시뮬레이션 결과를 xformOp:translate / xformOp:orient에 써야 하고,
        # 단일 transform matrix op에는 기록할 수 없기 때문 (RigidBody가 kinematic처럼 멈춤).
        self._xform.ClearXformOpOrder()

        if self._geom_kind == "box":
            self._use_transform_op = True
            init_ori = orientation if orientation is not None else (0.0, 0.0, 0.0)
            init_pos = position if position is not None else (0.0, 0.0, 0.0)
            mat = self._build_transform_matrix(init_ori, 1.0, init_pos,
                                               box_size=box_size)
            self._xform.AddTransformOp().Set(mat)
        else:
            self._use_transform_op = False
            init_pos = position if position is not None else (0.0, 0.0, 0.0)
            self._xform.AddTranslateOp().Set(Gf.Vec3d(*init_pos))

            if orientation is not None:
                init_ori = orientation
            elif orientation_range is not None:
                ori_min, ori_max = orientation_range
                init_ori = tuple((ori_min[i] + ori_max[i]) / 2.0 for i in range(3))
            else:
                init_ori = (0.0, 0.0, 0.0)
            self._xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(
                self._euler_to_quatf(init_ori)
            )

            if self._geom_kind == "usd" or scale != 1.0:
                self._xform.AddScaleOp().Set(Gf.Vec3d(scale, scale, scale))

        # ── USD 에셋 처리: 색상 + 메쉬 CollisionAPI ────────────
        if self._geom_kind == "usd":
            if color is not None:
                self._apply_color_to_usd(stage, prim, color)

            # 래핑 시에만 메쉬에 CollisionAPI 적용 (articulated USD는 내부에 이미 있음)
            if wrap_rigidbody:
                if prim.IsA(UsdGeom.Mesh):
                    UsdPhysics.CollisionAPI.Apply(prim)
                    mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
                    mc.CreateApproximationAttr(collision_approximation)
                else:
                    for desc in Usd.PrimRange(prim):
                        if desc.IsA(UsdGeom.Mesh):
                            UsdPhysics.CollisionAPI.Apply(desc)
                            mc = UsdPhysics.MeshCollisionAPI.Apply(desc)
                            mc.CreateApproximationAttr(collision_approximation)

        # ── 물리 래핑 ──────────────────────────────────────────
        if not wrap_rigidbody:
            # USD 내부 물리 유지 / 장식용 box: 추가 물리 없음
            self._position = np.array([0.0, 0.0, 0.0])
            return

        if deformable is not None:
            self._apply_deformable(stage, prim, deformable)
        else:
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.CollisionAPI.Apply(prim)

        # PhysicsMaterial (마찰 / 복원계수)
        material_path = f"{self.prim_path}/PhysicsMaterial"
        UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(material_path))
        mat_prim = stage.GetPrimAtPath(material_path)
        mat_api = UsdPhysics.MaterialAPI(mat_prim)
        mat_api.CreateStaticFrictionAttr(friction)
        mat_api.CreateDynamicFrictionAttr(friction)
        mat_api.CreateRestitutionAttr(0.0)

        if mass is not None:
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr(mass)

        from pxr import UsdShade
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            UsdShade.Material(mat_prim),
            UsdShade.Tokens.weakerThanDescendants,
            "physics",
        )

        self._position = np.array([0.0, 0.0, 0.0])

    @staticmethod
    def _compute_usd_size(prim, scale: float) -> float:
        """USD 에셋의 월드 bbox에서 최대 extent를 추출해 scale 적용.

        instance proxy + default purpose로 bbox 계산. 실패 또는 빈 bbox면 0.1m fallback.
        articulated USD는 초기 상태(닫힌 서랍 등) 기준으로 계산.
        """
        from pxr import Usd, UsdGeom
        try:
            cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                useExtentsHint=True,
            )
            bbox = cache.ComputeWorldBound(prim)
            rng = bbox.ComputeAlignedRange()
            if rng.IsEmpty():
                return 0.1
            v_min = rng.GetMin()
            v_max = rng.GetMax()
            max_extent = max(v_max[0] - v_min[0],
                             v_max[1] - v_min[1],
                             v_max[2] - v_min[2])
            if max_extent <= 0:
                return 0.1
            return float(max_extent * scale)
        except Exception as e:
            print(f"[WARN] USD size 자동 계산 실패: {e}")
            return 0.1

    @staticmethod
    def _has_articulation(prim) -> bool:
        """스테이지 composition 상태에서 ArticulationRootAPI 존재 확인."""
        from pxr import Usd, UsdPhysics
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return True
        # 일반 traversal
        for desc in Usd.PrimRange(prim):
            if desc.HasAPI(UsdPhysics.ArticulationRootAPI):
                return True
        # instance proxy 포함 traversal (instanceable USD 대응)
        try:
            predicate = Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)
            for desc in Usd.PrimRange(prim, predicate):
                if desc.HasAPI(UsdPhysics.ArticulationRootAPI):
                    return True
        except Exception:
            pass
        return False

    @staticmethod
    def _usd_has_articulation(usd_path: str) -> bool:
        """USD 파일을 Sdf.Layer로 직접 열어 ArticulationRootAPI 존재 확인.

        스테이지 composition/인스턴싱에 의존하지 않으므로
        instanceable 에셋(예: sektion_cabinet_instanceable.usd)에서도 신뢰 가능.
        """
        from pxr import Sdf
        try:
            layer = Sdf.Layer.FindOrOpen(usd_path)
            if layer is None:
                return False

            def _walk(spec) -> bool:
                try:
                    api_op = spec.GetInfo("apiSchemas")
                    items = []
                    if api_op is not None:
                        if hasattr(api_op, "GetAddedOrExplicitItems"):
                            items = list(api_op.GetAddedOrExplicitItems())
                        elif hasattr(api_op, "explicitItems"):
                            items = list(api_op.explicitItems)
                        elif hasattr(api_op, "__iter__"):
                            items = list(api_op)
                    if any("ArticulationRootAPI" in str(n) for n in items):
                        return True
                except Exception:
                    pass
                for child in spec.nameChildren:
                    if _walk(child):
                        return True
                return False

            for root in layer.rootPrims:
                if _walk(root):
                    return True
            return False
        except Exception as e:
            print(f"[WARN] USD 레이어 검사 실패 ({usd_path}): {e}")
            return False

    def _apply_deformable(self, stage, prim, deformable):
        """Deformable body (RigidBody와 양립 불가)."""
        from omni.physx.scripts import deformableUtils
        from pxr import Usd, UsdGeom, UsdShade
        import math

        mesh_prim = prim
        mesh_path = self.prim_path
        if not prim.IsA(UsdGeom.Mesh):
            for desc in Usd.PrimRange(prim):
                if desc.IsA(UsdGeom.Mesh):
                    mesh_prim = desc
                    mesh_path = str(desc.GetPath())
                    break

        young_modulus = 100.0 * math.pow(1000.0, deformable)

        deformableUtils.add_physx_deformable_body(
            stage, mesh_path, collision_simplification=True,
            simulation_hexahedral_resolution=2, self_collision=False,
        )

        deform_mat_path = f"{self.prim_path}/DeformableMaterial"
        deformableUtils.add_deformable_body_material(
            stage, deform_mat_path,
            youngs_modulus=young_modulus, poissons_ratio=0.45,
            damping_scale=0.1, dynamic_friction=1.0,
        )
        mat = UsdShade.Material(stage.GetPrimAtPath(deform_mat_path))
        UsdShade.MaterialBindingAPI.Apply(mesh_prim).Bind(
            mat, UsdShade.Tokens.weakerThanDescendants, "physics",
        )

    def _apply_color_to_usd(self, stage, prim, color):
        """USD 오브젝트에 단색 OmniPBR 머티리얼을 생성하여 바인딩."""
        from pxr import UsdShade, Sdf, Gf, Usd, UsdGeom

        mat_path = f"{self.prim_path}/Looks/ColorMaterial"
        shader_path = f"{mat_path}/Shader"

        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, shader_path)
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*color)
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        for desc in Usd.PrimRange(prim):
            if desc.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI.Apply(desc).Bind(mat)

        print(f"[INFO] USD 오브젝트 색상 적용: {self.name} → {color}")

    @staticmethod
    def _euler_to_rotation(orientation):
        """(rx, ry, rz) degrees → 3x3 rotation matrix rows."""
        import math
        rx, ry, rz = [math.radians(a) for a in orientation]
        cx, sx = math.cos(rx), math.sin(rx)
        cy, sy = math.cos(ry), math.sin(ry)
        cz, sz = math.cos(rz), math.sin(rz)
        return (
            (cy*cz, -cy*sz, sy),
            (sx*sy*cz+cx*sz, -sx*sy*sz+cx*cz, -sx*cy),
            (-cx*sy*cz+sx*sz, cx*sy*sz+sx*cz, cx*cy),
        )

    @staticmethod
    def _euler_to_quatf(orientation):
        """(rx, ry, rz) degrees → Gf.Quatf (Rx * Ry * Rz, same convention as _euler_to_rotation)."""
        from pxr import Gf
        import math
        hrx, hry, hrz = [math.radians(a) / 2.0 for a in orientation]
        cx, sx = math.cos(hrx), math.sin(hrx)
        cy, sy = math.cos(hry), math.sin(hry)
        cz, sz = math.cos(hrz), math.sin(hrz)
        w  = cx*cy*cz - sx*sy*sz
        qx = sx*cy*cz + cx*sy*sz
        qy = cx*sy*cz - sx*cy*sz
        qz = cx*cy*sz + sx*sy*cz
        return Gf.Quatf(float(w), Gf.Vec3f(float(qx), float(qy), float(qz)))

    @staticmethod
    def _build_transform_matrix(orientation, scale, position=(0, 0, 0),
                                box_size: tuple | None = None):
        """orientation(degrees) + scale(또는 box_size) + position → Gf.Matrix4d.

        USD는 row-vector convention (p' = p * M)이므로 rotation을 전치하여 저장.
        box_size가 있으면 축별 스케일(sx, sy, sz)을 각 basis 벡터에 곱함.
        """
        from pxr import Gf
        rot = TargetObject._euler_to_rotation(orientation)
        r = rot
        if box_size is not None:
            sx, sy, sz = box_size[0] / 2.0, box_size[1] / 2.0, box_size[2] / 2.0
            return Gf.Matrix4d(
                sx*r[0][0], sx*r[1][0], sx*r[2][0], 0,
                sy*r[0][1], sy*r[1][1], sy*r[2][1], 0,
                sz*r[0][2], sz*r[1][2], sz*r[2][2], 0,
                position[0], position[1], position[2], 1,
            )
        s = scale
        return Gf.Matrix4d(
            s*r[0][0], s*r[1][0], s*r[2][0], 0,
            s*r[0][1], s*r[1][1], s*r[2][1], 0,
            s*r[0][2], s*r[1][2], s*r[2][2], 0,
            position[0], position[1], position[2], 1,
        )

    def set_position(self, position: np.ndarray):
        from pxr import Gf, UsdGeom
        self._position = np.asarray(position).copy()
        prim = self._stage.GetPrimAtPath(self.prim_path)
        xform = UsdGeom.Xformable(prim)
        for op in xform.GetOrderedXformOps():
            name = op.GetOpName()
            if name == "xformOp:translate":
                op.Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
                return
            if name == "xformOp:transform":
                # transform op 사용 중: 기존 행렬의 translation만 교체
                mat = op.Get()
                new_mat = Gf.Matrix4d(mat)
                new_mat.SetTranslateOnly(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
                op.Set(new_mat)
                return

    def set_orientation(self, orientation: tuple):
        """오브젝트의 orientation을 갱신.

        rigid body 오브젝트: xformOp:orient op에 quaternion 직접 write.
        box_size 장식용 (transform matrix op): 매트릭스 회전 부분만 교체.
        """
        from pxr import UsdGeom
        prim = self._stage.GetPrimAtPath(self.prim_path)
        xform = UsdGeom.Xformable(prim)

        for op in xform.GetOrderedXformOps():
            name = op.GetOpName()
            if name == "xformOp:orient":
                op.Set(self._euler_to_quatf(orientation))
                return
            if name == "xformOp:transform":
                old_mat = op.Get()
                old_trans = old_mat.ExtractTranslation()
                effective_scale = self._scale if self._geom_kind != "box" else 1.0
                mat = self._build_transform_matrix(
                    orientation, effective_scale,
                    (old_trans[0], old_trans[1], old_trans[2]),
                    box_size=self._box_size,
                )
                op.Set(mat)
                return

    # ── Live setter API (env editor용) ─────────────────────

    def set_color(self, color: tuple):
        """오브젝트 색상 live update. primitive는 DisplayColor, USD는 shader diffuse."""
        from pxr import UsdGeom, UsdShade, Sdf, Gf, Usd
        self.color = color
        prim = self._stage.GetPrimAtPath(self.prim_path)
        if self._geom_kind == "usd":
            # 기존 ColorMaterial shader의 diffuse 덮어쓰기. 없으면 새로 생성.
            shader_path = f"{self.prim_path}/Looks/ColorMaterial/Shader"
            shader_prim = self._stage.GetPrimAtPath(shader_path)
            if shader_prim.IsValid():
                shader = UsdShade.Shader(shader_prim)
                diffuse = shader.GetInput("diffuseColor")
                if diffuse:
                    diffuse.Set(Gf.Vec3f(*color))
                else:
                    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                        Gf.Vec3f(*color))
                return
            # shader 없으면 새로 적용
            self._apply_color_to_usd(self._stage, prim, color)
            return
        # primitive / box: GPrim DisplayColor
        gprim = UsdGeom.Gprim(prim)
        if gprim:
            attr = gprim.GetDisplayColorAttr()
            if not attr:
                attr = gprim.CreateDisplayColorAttr()
            attr.Set([Gf.Vec3f(*color)])

    def set_mass(self, mass: float):
        """RigidBody 질량 live update."""
        from pxr import UsdPhysics
        prim = self._stage.GetPrimAtPath(self.prim_path)
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(float(mass))

    def set_friction(self, friction: float):
        """PhysicsMaterial의 static/dynamic friction live update."""
        from pxr import UsdPhysics
        material_path = f"{self.prim_path}/PhysicsMaterial"
        mat_prim = self._stage.GetPrimAtPath(material_path)
        if not mat_prim.IsValid():
            return
        mat_api = UsdPhysics.MaterialAPI(mat_prim)
        mat_api.CreateStaticFrictionAttr(float(friction))
        mat_api.CreateDynamicFrictionAttr(float(friction))

    def set_size(self, size: float):
        """primitive geom의 size live update. USD는 내부 value만 갱신."""
        from pxr import UsdGeom
        size = float(size)
        self.size = size
        prim = self._stage.GetPrimAtPath(self.prim_path)
        if self._geom_kind == "prim":
            # cube/sphere/cylinder 판별은 geom API 호출로 처리
            cube = UsdGeom.Cube(prim)
            if cube:
                cube.GetSizeAttr().Set(size)
                self.half_height = size / 2.0
                return
            sphere = UsdGeom.Sphere(prim)
            if sphere:
                sphere.GetRadiusAttr().Set(size / 2.0)
                self.half_height = size / 2.0
                return
            cyl = UsdGeom.Cylinder(prim)
            if cyl:
                cyl.GetRadiusAttr().Set(size / 2.0)
                # height는 별도 설정. size 변경 시에도 half_height는 유지 (height 필드가 결정)
                return
        # USD / box는 size 변경으로 기하가 바뀌지 않음 (scale 사용)
        if self._geom_kind == "usd":
            self.half_height = size / 2.0

    def set_height(self, height: float):
        """cylinder 전용 live height update."""
        from pxr import UsdGeom
        prim = self._stage.GetPrimAtPath(self.prim_path)
        cyl = UsdGeom.Cylinder(prim)
        if cyl:
            cyl.GetHeightAttr().Set(float(height))
            self.half_height = float(height) / 2.0

    def set_scale(self, scale: float):
        """USD 오브젝트의 scale live update. scale op 직접 수정, transform op는 재계산."""
        from pxr import Gf, UsdGeom
        scale = float(scale)
        self._scale = scale
        if self._geom_kind != "usd":
            return
        prim = self._stage.GetPrimAtPath(self.prim_path)
        xform = UsdGeom.Xformable(prim)

        for op in xform.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:scale":
                op.Set(Gf.Vec3d(scale, scale, scale))
                return
            if op.GetOpName() == "xformOp:transform":
                old_mat = op.Get()
                old_trans = old_mat.ExtractTranslation()
                rot = old_mat.ExtractRotation()
                try:
                    euler = rot.Decompose(Gf.Vec3d(1, 0, 0),
                                          Gf.Vec3d(0, 1, 0),
                                          Gf.Vec3d(0, 0, 1))
                    orientation = (float(euler[0]), float(euler[1]), float(euler[2]))
                except Exception:
                    orientation = (0.0, 0.0, 0.0)
                mat = self._build_transform_matrix(
                    orientation, scale,
                    (old_trans[0], old_trans[1], old_trans[2]),
                    box_size=self._box_size,
                )
                op.Set(mat)
                return
        # scale op 없음 → 추가
        xform.AddScaleOp().Set(Gf.Vec3d(scale, scale, scale))

    def get_position(self) -> np.ndarray:
        from pxr import UsdGeom
        prim = self._stage.GetPrimAtPath(self.prim_path)
        xform = UsdGeom.Xformable(prim)
        transform = xform.ComputeLocalToWorldTransform(0)
        t = transform.ExtractTranslation()
        return np.array([t[0], t[1], t[2]])


class ObjectManager:
    """여러 오브젝트를 생성, 랜덤 배치, ROS2 마커 퍼블리쉬하는 매니저."""

    def __init__(self, stage, table_center: np.ndarray, table_scale: np.ndarray,
                 spawn_range: list | None = None):
        self._stage = stage
        self.objects: list[TargetObject] = []

        table_surface_z = table_center[2] + table_scale[2] / 2.0

        if spawn_range is not None:
            self._x_range = (spawn_range[0][0], spawn_range[1][0])
            self._y_range = (spawn_range[0][1], spawn_range[1][1])
        else:
            half_x = table_scale[0] / 2.0 * 0.5
            half_y = table_scale[1] / 2.0 * 0.5
            self._x_range = (table_center[0] - half_x, table_center[0] + half_x)
            self._y_range = (table_center[1] - half_y, table_center[1] + half_y)

        self._surface_z = table_surface_z

        self._ros_node = None
        self._marker_pubs = {}
        self._robot_prim_path = None

    def add_object(self, name: str, size: float | None = None,
                   color: tuple | None = None, shape: str = "cube",
                   height: float | None = None,
                   usd_path: str | None = None, scale: float = 1.0,
                   box_size: tuple | None = None,
                   position: tuple | None = None,
                   orientation: tuple | None = None,
                   position_range: list | None = None,
                   orientation_range: list | None = None,
                   scale_range: tuple[float, float] | None = None,
                   wrap_rigidbody: bool | None = None,
                   deformable: float | None = None,
                   collision_approximation: str = "convexHull",
                   friction: float = 1.0,
                   mass: float | None = None) -> TargetObject:
        obj = TargetObject(
            name, self._stage, size=size, color=color, shape=shape, height=height,
            usd_path=usd_path, scale=scale, box_size=box_size,
            position=position, orientation=orientation,
            position_range=position_range, orientation_range=orientation_range,
            wrap_rigidbody=wrap_rigidbody, deformable=deformable,
            collision_approximation=collision_approximation,
            friction=friction, mass=mass,
        )
        if scale_range is not None:
            obj.scale_range = (float(scale_range[0]), float(scale_range[1]))
        self.objects.append(obj)
        return obj

    def randomize_all(self):
        """오브젝트 위치/회전/스케일을 갱신.

        position_range가 있으면 range 우선 (fixed_position 무시).
        둘 다 없으면 테이블 기본 범위에서 샘플.
        """
        positions = []
        for obj in self.objects:
            has_pos_range = obj.position_range is not None
            has_ori_range = obj.orientation_range is not None

            # ── 위치 결정 ──────────────────────────────────
            if has_pos_range:
                x_range = (obj.position_range[0][0], obj.position_range[1][0])
                y_range = (obj.position_range[0][1], obj.position_range[1][1])
                z_range = (obj.position_range[0][2], obj.position_range[1][2])
                for _ in range(100):
                    x = np.random.uniform(*x_range)
                    y = np.random.uniform(*y_range)
                    z = np.random.uniform(*z_range)
                    pos = np.array([x, y, z])
                    too_close = False
                    for prev_pos, prev_obj in zip(positions, self.objects):
                        min_dist = (obj.size + prev_obj.size) / 2.0 + 0.02
                        if np.linalg.norm(pos[:2] - prev_pos[:2]) < min_dist:
                            too_close = True
                            break
                    if not too_close:
                        break
            elif obj.fixed_position is not None:
                pos = np.array(obj.fixed_position)
            else:
                for _ in range(100):
                    x = np.random.uniform(*self._x_range)
                    y = np.random.uniform(*self._y_range)
                    z = self._surface_z + obj.half_height + 0.05
                    pos = np.array([x, y, z])
                    too_close = False
                    for prev_pos, prev_obj in zip(positions, self.objects):
                        min_dist = (obj.size + prev_obj.size) / 2.0 + 0.02
                        if np.linalg.norm(pos[:2] - prev_pos[:2]) < min_dist:
                            too_close = True
                            break
                    if not too_close:
                        break

            # ── 회전 결정 ──────────────────────────────────
            if has_ori_range:
                ori_min = obj.orientation_range[0]
                ori_max = obj.orientation_range[1]
                ori = tuple(
                    np.random.uniform(ori_min[i], ori_max[i]) for i in range(3)
                )
                obj.set_orientation(ori)
            elif obj.fixed_orientation is not None:
                obj.set_orientation(obj.fixed_orientation)

            # ── 스케일 랜덤 ────────────────────────────────
            if obj.scale_range is not None:
                s = float(np.random.uniform(obj.scale_range[0], obj.scale_range[1]))
                obj.set_scale(s)

            obj.set_position(pos)
            positions.append(pos)

    def setup_marker_publisher(self, robot_prim_path: str):
        """오브젝트별 Marker 퍼블리셔 (로봇 base 기준 상대 좌표)."""
        import rclpy
        from visualization_msgs.msg import Marker

        self._robot_prim_path = robot_prim_path

        try:
            rclpy.init()
        except RuntimeError:
            pass

        self._ros_node = rclpy.create_node("object_pose_publisher")
        for obj in self.objects:
            topic = f"/simulation/object_markers/{obj.name}"
            self._marker_pubs[obj.name] = self._ros_node.create_publisher(Marker, topic, 10)

    def publish_markers(self):
        """오브젝트별 Marker(구체)를 로봇 base 기준 상대 좌표로 퍼블리쉬."""
        from pxr import UsdGeom, Gf
        from visualization_msgs.msg import Marker

        if not self._marker_pubs:
            return

        prim = self._stage.GetPrimAtPath(self._robot_prim_path)
        xform = UsdGeom.Xformable(prim)
        inv_base_tf = xform.ComputeLocalToWorldTransform(0).GetInverse()

        for i, obj in enumerate(self.objects):
            pos = obj.get_position()
            rel_pt = inv_base_tf.Transform(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))

            obj_prim = self._stage.GetPrimAtPath(obj.prim_path)
            obj_xform = UsdGeom.Xformable(obj_prim)
            world_tf = obj_xform.ComputeLocalToWorldTransform(0)
            rel_tf = inv_base_tf * world_tf
            rel_quat = rel_tf.ExtractRotation().GetQuaternion()
            qw = rel_quat.GetReal()
            qi = rel_quat.GetImaginary()

            marker = Marker()
            marker.header.frame_id = "panda_link0"
            marker.ns = obj.name
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = rel_pt[0]
            marker.pose.position.y = rel_pt[1]
            marker.pose.position.z = rel_pt[2]
            marker.pose.orientation.x = qi[0]
            marker.pose.orientation.y = qi[1]
            marker.pose.orientation.z = qi[2]
            marker.pose.orientation.w = qw
            marker.scale.x = obj.size
            marker.scale.y = obj.size
            marker.scale.z = obj.size
            c = obj.color if obj.color is not None else (0.5, 0.5, 0.5)
            marker.color.r = float(c[0])
            marker.color.g = float(c[1])
            marker.color.b = float(c[2])
            marker.color.a = 0.8

            self._marker_pubs[obj.name].publish(marker)

    def get_all_positions(self) -> dict:
        return {obj.name: obj.get_position() for obj in self.objects}

    def destroy(self):
        """ROS2 노드 정리."""
        if self._ros_node:
            self._ros_node.destroy_node()
            self._ros_node = None
        self._marker_pubs.clear()

    def print_positions(self):
        for obj in self.objects:
            pos = obj.get_position()
            print(f"  {obj.name}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
