"""
테이블 위 오브젝트 생성, 랜덤 배치, ROS2 퍼블리쉬.

isaacsim import는 함수/메서드 내부에서 수행.
"""

import numpy as np


class TargetObject:
    """테이블 위의 개별 작업 대상 오브젝트.

    프리미티브(cube/sphere/cylinder) 또는 USD 에셋 파일로 생성 가능.

    Args:
        usd_path: USD 에셋 파일 경로. 지정 시 shape/color 무시.
                  로컬 경로 또는 Nucleus 경로(omniverse://...) 모두 가능.
        scale: USD 에셋의 스케일 (기본 1.0). usd_path 사용 시에만 적용.
    """

    def __init__(self, name: str, stage, size: float = 0.06,
                 color: tuple = (1.0, 0.3, 0.2), shape: str = "cube",
                 height: float | None = None,
                 usd_path: str | None = None, scale: float = 1.0):
        from pxr import UsdGeom, UsdPhysics, Gf

        self.name = name
        self.prim_path = f"/World/{name}"
        self.size = size
        self.color = color
        self._stage = stage

        if usd_path is not None:
            # USD 에셋 로드
            prim = stage.DefinePrim(self.prim_path, "Xform")
            prim.GetReferences().AddReference(usd_path)
            self._xform = UsdGeom.Xformable(prim)
            self._translate_op = self._xform.AddTranslateOp()
            if scale != 1.0:
                self._xform.AddScaleOp().Set(Gf.Vec3d(scale, scale, scale))
            self.half_height = size / 2.0  # size를 높이 힌트로 사용
        else:
            # 프리미티브 생성
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

            self._xform = UsdGeom.Xformable(stage.GetPrimAtPath(self.prim_path))
            self._translate_op = self._xform.AddTranslateOp()
            geom.CreateDisplayColorAttr([color])

        prim = stage.GetPrimAtPath(self.prim_path)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.CollisionAPI.Apply(prim)

        # 높은 마찰력으로 그리핑 안정성 향상
        material_path = f"{self.prim_path}/PhysicsMaterial"
        UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(material_path))
        mat_prim = stage.GetPrimAtPath(material_path)
        mat_api = UsdPhysics.MaterialAPI(mat_prim)
        mat_api.CreateStaticFrictionAttr(1.0)
        mat_api.CreateDynamicFrictionAttr(1.0)
        mat_api.CreateRestitutionAttr(0.0)
        # 머티리얼을 콜리전에 바인딩
        from pxr import UsdShade
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            UsdShade.Material(mat_prim),
            UsdShade.Tokens.weakerThanDescendants,
            "physics",
        )

        self._position = np.array([0.0, 0.0, 0.0])

    def set_position(self, position: np.ndarray):
        from pxr import Gf
        self._position = position.copy()
        self._translate_op.Set(Gf.Vec3d(*position.tolist()))

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
            # [[x_min, y_min], [x_max, y_max]]
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

    def add_object(self, name: str, size: float = 0.06,
                   color: tuple = (1.0, 0.3, 0.2), shape: str = "cube",
                   height: float | None = None,
                   usd_path: str | None = None, scale: float = 1.0) -> TargetObject:
        obj = TargetObject(name, self._stage, size, color, shape, height, usd_path, scale)
        self.objects.append(obj)
        return obj

    def randomize_all(self):
        """모든 오브젝트를 테이블 위 랜덤 위치에 배치 (겹침 방지)."""
        positions = []
        for obj in self.objects:
            for _ in range(100):
                x = np.random.uniform(*self._x_range)
                y = np.random.uniform(*self._y_range)
                z = self._surface_z + obj.half_height + 0.002
                pos = np.array([x, y, z])

                too_close = False
                for prev_pos, prev_obj in zip(positions, self.objects):
                    min_dist = (obj.size + prev_obj.size) / 2.0 + 0.02
                    if np.linalg.norm(pos[:2] - prev_pos[:2]) < min_dist:
                        too_close = True
                        break
                if not too_close:
                    break

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

        # 로봇 base의 역변환 행렬
        prim = self._stage.GetPrimAtPath(self._robot_prim_path)
        xform = UsdGeom.Xformable(prim)
        inv_base_tf = xform.ComputeLocalToWorldTransform(0).GetInverse()

        for i, obj in enumerate(self.objects):
            pos = obj.get_position()
            rel_pt = inv_base_tf.Transform(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))

            marker = Marker()
            marker.header.frame_id = "panda_link0"
            marker.ns = obj.name
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = rel_pt[0]
            marker.pose.position.y = rel_pt[1]
            marker.pose.position.z = rel_pt[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = obj.size
            marker.scale.y = obj.size
            marker.scale.z = obj.size
            marker.color.r = float(obj.color[0])
            marker.color.g = float(obj.color[1])
            marker.color.b = float(obj.color[2])
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
