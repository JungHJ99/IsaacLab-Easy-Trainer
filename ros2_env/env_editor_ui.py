"""
Isaac Sim 내장 omni.ui 기반 환경 편집 패널.

오브젝트, 카메라, 조명을 런타임으로 추가/편집하고 BaseEnv 서브클래스 파이썬
템플릿을 저장한다. omni.ui 기본 폰트가 한글 글리프를 지원하지 않아 UI 텍스트는
모두 영어.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable


SHAPE_CHOICES = ("cube", "sphere", "cylinder", "USD")
LIGHT_TYPE_CHOICES = ("dome", "distant")


# Nucleus(assets_root) 상대 경로 프리셋. 필요시 확장.
# 첫 항목은 no-op: 사용자가 직접 경로를 입력할 때는 이 항목을 선택 유지.
NUCLEUS_PRESETS: dict[str, str | None] = {
    "(none — local / custom)": None,
    "Sektion Cabinet (articulated drawer)":
        "Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd",
    "Kitchen Set — Fridge (articulated)":
        "Isaac/Props/KitchenSet/assets/Fridge/Fridge.usd",
    "Kitchen Set — Oven (articulated)":
        "Isaac/Props/KitchenSet/assets/Oven/Oven.usd",
    "YCB Mug (rigid)":
        "Isaac/Props/YCB/Axis_Aligned/025_mug.usd",
    "YCB Cracker Box (rigid)":
        "Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd",
    "YCB Power Drill (rigid)":
        "Isaac/Props/YCB/Axis_Aligned/035_power_drill.usd",
}


def _resolve_nucleus_path(relpath: str) -> str | None:
    """assets_root + relpath로 풀 URL 생성. 실패 시 None."""
    try:
        from isaacsim.storage.native import get_assets_root_path
    except ImportError:
        try:
            from omni.isaac.core.utils.nucleus import get_assets_root_path
        except ImportError:
            return None
    root = get_assets_root_path()
    if not root:
        return None
    return f"{root.rstrip('/')}/{relpath.lstrip('/')}"


# ─────────────────────────────────────────────────────────
# DragCanvas: bird's-eye 탑뷰 드래그
# ─────────────────────────────────────────────────────────

class DragCanvas:
    """탑뷰 드래그 캔버스 (bird's-eye)."""

    def __init__(
        self,
        world_x_range: tuple[float, float],
        world_y_range: tuple[float, float],
        pixel_size: tuple[int, int] = (360, 260),
        on_change: Callable[[tuple[float, float, float, float]], None] | None = None,
    ):
        self.world_x_min, self.world_x_max = world_x_range
        self.world_y_min, self.world_y_max = world_y_range
        self.pixel_w, self.pixel_h = pixel_size
        self.on_change = on_change

        self._sel_start = None
        self._sel_end = None
        self._dragging = False
        self._overlays: list[dict] = []

        self._build()

    def _build(self):
        import omni.ui as ui
        self._frame = ui.Frame(width=self.pixel_w, height=self.pixel_h)
        with self._frame:
            self._zstack = ui.ZStack()
        try:
            self._frame.set_mouse_pressed_fn(self._on_press)
            self._frame.set_mouse_released_fn(self._on_release)
            self._frame.set_mouse_moved_fn(self._on_move)
        except AttributeError:
            self._zstack.set_mouse_pressed_fn(self._on_press)
            self._zstack.set_mouse_released_fn(self._on_release)
            self._zstack.set_mouse_moved_fn(self._on_move)
        self._render()

    def _screen_to_pixel(self, sx: float, sy: float) -> tuple[float, float]:
        fx = self._frame.screen_position_x
        fy = self._frame.screen_position_y
        return (sx - fx, sy - fy)

    def _pixel_to_world(self, px: float, py: float) -> tuple[float, float]:
        tx = max(0.0, min(1.0, px / self.pixel_w))
        ty = max(0.0, min(1.0, py / self.pixel_h))
        wy = self.world_y_max - tx * (self.world_y_max - self.world_y_min)
        wx = self.world_x_max - ty * (self.world_x_max - self.world_x_min)
        return wx, wy

    def _world_to_pixel(self, wx: float, wy: float) -> tuple[float, float]:
        tx = (self.world_y_max - wy) / max(1e-9, self.world_y_max - self.world_y_min)
        ty = (self.world_x_max - wx) / max(1e-9, self.world_x_max - self.world_x_min)
        return tx * self.pixel_w, ty * self.pixel_h

    def _on_press(self, sx, sy, button, modifier):
        if button != 0:
            return
        px, py = self._screen_to_pixel(sx, sy)
        if not (0 <= px <= self.pixel_w and 0 <= py <= self.pixel_h):
            return
        self._sel_start = (px, py)
        self._sel_end = (px, py)
        self._dragging = True
        self._render()
        rng = self.get_world_range()
        if rng and self.on_change:
            self.on_change(rng)

    def _on_move(self, sx, sy, modifier, pressed=None):
        if not self._dragging:
            return
        px, py = self._screen_to_pixel(sx, sy)
        px = max(0, min(self.pixel_w, px))
        py = max(0, min(self.pixel_h, py))
        self._sel_end = (px, py)
        self._render()
        rng = self.get_world_range()
        if rng and self.on_change:
            self.on_change(rng)

    def _on_release(self, sx, sy, button, modifier):
        if button != 0 or not self._dragging:
            return
        px, py = self._screen_to_pixel(sx, sy)
        px = max(0, min(self.pixel_w, px))
        py = max(0, min(self.pixel_h, py))
        self._sel_end = (px, py)
        self._dragging = False
        self._render()
        rng = self.get_world_range()
        if rng and self.on_change:
            self.on_change(rng)

    def get_world_range(self) -> tuple[float, float, float, float] | None:
        if self._sel_start is None or self._sel_end is None:
            return None
        wx1, wy1 = self._pixel_to_world(*self._sel_start)
        wx2, wy2 = self._pixel_to_world(*self._sel_end)
        return (min(wx1, wx2), max(wx1, wx2), min(wy1, wy2), max(wy1, wy2))

    def clear_selection(self):
        self._sel_start = None
        self._sel_end = None
        self._render()

    def set_overlays(self, overlays: list[dict]):
        self._overlays = overlays
        self._render()

    def _render(self):
        import omni.ui as ui
        self._zstack.clear()
        with self._zstack:
            ui.Rectangle(style={"background_color": 0xff1a1a1a, "border_color": 0xff555555, "border_width": 1})

            cx = self.pixel_w / 2
            cy = self.pixel_h / 2
            with ui.Placer(offset_x=0, offset_y=cy):
                ui.Rectangle(width=self.pixel_w, height=1, style={"background_color": 0xff333333})
            with ui.Placer(offset_x=cx, offset_y=0):
                ui.Rectangle(width=1, height=self.pixel_h, style={"background_color": 0xff333333})

            label_style = {"color": 0xffaaaaaa, "font_size": 10}
            with ui.Placer(offset_x=cx - 30, offset_y=2):
                ui.Label("+X (front)", style=label_style, width=70, height=14)
            with ui.Placer(offset_x=cx - 30, offset_y=self.pixel_h - 16):
                ui.Label("-X (back)", style=label_style, width=70, height=14)
            with ui.Placer(offset_x=4, offset_y=cy - 7):
                ui.Label("+Y", style=label_style, width=30, height=14)
            with ui.Placer(offset_x=self.pixel_w - 28, offset_y=cy - 7):
                ui.Label("-Y", style=label_style, width=30, height=14)

            for ov in self._overlays:
                x_min, y_min, x_max, y_max = ov["rect"]
                p1x, p1y = self._world_to_pixel(x_min, y_max)
                p2x, p2y = self._world_to_pixel(x_max, y_min)
                w = max(2, abs(p2x - p1x))
                h = max(2, abs(p2y - p1y))
                ox = min(p1x, p2x)
                oy = min(p1y, p2y)
                color = ov.get("color", 0x3300ff00)
                with ui.Placer(offset_x=ox, offset_y=oy):
                    ui.Rectangle(
                        width=w, height=h,
                        style={"background_color": color, "border_color": 0xff00ff00, "border_width": 1},
                    )

            if self._sel_start and self._sel_end:
                x1, y1 = self._sel_start
                x2, y2 = self._sel_end
                ox = min(x1, x2)
                oy = min(y1, y2)
                w = max(2, abs(x2 - x1))
                h = max(2, abs(y2 - y1))
                with ui.Placer(offset_x=ox, offset_y=oy):
                    ui.Rectangle(
                        width=w, height=h,
                        style={"background_color": 0x55ffaa00, "border_color": 0xffffaa00, "border_width": 2},
                    )


# ─────────────────────────────────────────────────────────
# 공통 helper: XYZ 행 만들기
# ─────────────────────────────────────────────────────────

def _minus_button(get_model, step: float, width: int = 16):
    """현재 HStack에 [-] 버튼 추가. get_model()이 click 시점에 model을 반환."""
    import omni.ui as ui
    ui.Button("-", width=width, height=22,
              clicked_fn=(lambda g=get_model, s=step:
                          g().set_value(g().get_value_as_float() - s)))


def _plus_button(get_model, step: float, width: int = 16):
    """현재 HStack에 [+] 버튼 추가."""
    import omni.ui as ui
    ui.Button("+", width=width, height=22,
              clicked_fn=(lambda g=get_model, s=step:
                          g().set_value(g().get_value_as_float() + s)))


def _xyz_row(label: str, default=(0.0, 0.0, 0.0), label_width=70,
             drag=False, step=0.01, spinner=False):
    """[label] [- x +] [- y +] [- z +] 행. 3개 모델 반환."""
    import omni.ui as ui
    with ui.HStack(height=22):
        ui.Label(label, width=label_width)
        models: list = []
        for i in range(3):
            if spinner:
                _minus_button(lambda i=i: models[i], step)
            if drag:
                m = ui.FloatDrag(height=22, step=step).model
            else:
                m = ui.FloatField(height=22).model
            m.set_value(default[i])
            models.append(m)
            if spinner:
                _plus_button(lambda i=i: models[i], step)
    return tuple(models)


def _get_xyz(models) -> list[float]:
    return [models[0].get_value_as_float(),
            models[1].get_value_as_float(),
            models[2].get_value_as_float()]


# ─────────────────────────────────────────────────────────
# LauncherPanel: 시작 시 New / Load 선택
# ─────────────────────────────────────────────────────────

class LauncherPanel:
    """시작 창 — New Env / Load Env 선택.

    env 콜백:
        load_env_file(path)  — 파일을 파싱해 editor 상태로 로드.
    """

    def __init__(self, env, envs_dir: str):
        import omni.ui as ui

        self._env = env
        self._envs_dir = envs_dir
        self._files = self._list_files()

        self._window = ui.Window(
            "Env Editor — Launcher",
            width=460,
            height=280,
            flags=ui.WINDOW_FLAGS_NO_COLLAPSE,
        )
        with self._window.frame:
            with ui.VStack(spacing=8, height=0):
                ui.Label("Choose how to start:",
                         style={"font_size": 14}, height=22)
                ui.Button("New Env  (defaults: side/top/wrist cam, DomeLight)",
                          height=40, clicked_fn=self._on_new,
                          style={"background_color": 0xff006600})

                ui.Separator(height=4)
                ui.Label("Or load an existing env file:",
                         style={"font_size": 12, "color": 0xffaaaaaa}, height=22)

                if self._files:
                    self._file_combo = ui.ComboBox(0, *self._files, height=22)
                    ui.Button("Load Env", height=40, clicked_fn=self._on_load,
                              style={"background_color": 0xff0066aa})
                else:
                    self._file_combo = None
                    ui.Label("  (no .py files in envs/)",
                             style={"color": 0xff888888}, height=20)

                self._status = ui.Label("", height=22)

    def _list_files(self) -> list[str]:
        skip = {"env_editor.py", "__init__.py"}
        return sorted(p.name for p in Path(self._envs_dir).glob("*.py")
                      if p.name not in skip)

    def _on_new(self):
        self._window.visible = False

    def _on_load(self):
        if self._file_combo is None or not self._files:
            return
        idx = self._file_combo.model.get_item_value_model().get_value_as_int()
        if idx < 0 or idx >= len(self._files):
            return
        filename = self._files[idx]
        path = str(Path(self._envs_dir) / filename)
        try:
            self._env.load_env_file(path)
        except Exception as e:
            self._status.text = f"Load failed: {e}"
            self._status.set_style({"color": 0xffff5555})
            return
        self._status.text = f"Loaded: {filename}"
        self._status.set_style({"color": 0xff55ff55})
        self._window.visible = False


# ─────────────────────────────────────────────────────────
# EditorPanel
# ─────────────────────────────────────────────────────────

class EditorPanel:
    """환경 편집 UI 패널 (Add-then-Edit 모델).

    초기 상태: [+Add Object] [+Add Camera] [+Add Light] 버튼 + 기존 컴포넌트 목록.
    컴포넌트 선택 혹은 Add 클릭 시 전체 필드가 live editable한 폼이 나타난다.

    env: EnvEditor 인스턴스. 콜백 API:
        editor_object_specs() / add_editor_object / remove_editor_object /
            update_editor_object(name, **changes) / recreate_editor_object(name, kwargs)
        (카메라/조명도 동일 패턴)
        save_env_template(filepath, classname)
    """

    def __init__(
        self,
        env,
        spawn_x_range: tuple[float, float],
        spawn_y_range: tuple[float, float],
        default_save_dir: str,
    ):
        import omni.ui as ui

        self._env = env
        self._save_dir = default_save_dir
        self._spawn_x_range = spawn_x_range
        self._spawn_y_range = spawn_y_range

        self._window = ui.Window(
            "Env Editor",
            width=600,
            height=900,
            flags=ui.WINDOW_FLAGS_NO_COLLAPSE,
        )

        self._object_list_container = None
        self._camera_list_container = None
        self._light_list_container = None
        self._obj_edit_frame = None
        self._cam_edit_frame = None
        self._light_edit_frame = None

        self._selected_camera_name: str | None = None
        self._selected_light_name: str | None = None
        self._selected_object_name: str | None = None
        self._suppress_live = False  # True 동안에는 value_changed 콜백에서 sim 업데이트 건너뜀

        with self._window.frame:
            with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
                with ui.VStack(spacing=6, height=0):
                    with ui.CollapsableFrame("Objects", collapsed=False):
                        with ui.VStack(spacing=4, height=0):
                            self._build_object_section()
                    with ui.CollapsableFrame("Cameras", collapsed=True):
                        with ui.VStack(spacing=4, height=0):
                            self._build_camera_section()
                    with ui.CollapsableFrame("Lights", collapsed=True):
                        with ui.VStack(spacing=4, height=0):
                            self._build_light_section()
                    with ui.CollapsableFrame("Save Env", collapsed=False):
                        with ui.VStack(spacing=4, height=0):
                            self._build_save_section()
                    self._status_label = ui.Label("Status: Ready", height=20)

    # ── Object section (Add button + list + live edit form) ──

    def _build_object_section(self):
        import omni.ui as ui
        ui.Button("+ Add Object", clicked_fn=self._on_add_object_click, height=32,
                  style={"background_color": 0xff006600})
        ui.Label("Added objects", style={"font_size": 12, "color": 0xffaaaaaa})
        self._object_list_container = ui.VStack(spacing=2, height=0)
        self._rebuild_object_list()

        ui.Separator(height=4)
        self._obj_edit_name_label = ui.Label("  (nothing selected — click + to add)",
                                             height=18, style={"color": 0xff888888})
        self._obj_edit_frame = ui.Frame(visible=False)
        with self._obj_edit_frame:
            with ui.VStack(spacing=4, height=0):
                self._build_object_edit_form()

    def _build_object_edit_form(self):
        """선택된 오브젝트의 모든 필드 live 편집 form."""
        import omni.ui as ui

        # Name (recreate on blur)
        with ui.HStack(height=22):
            ui.Label("Name:", width=70)
            self._name_model = ui.StringField(height=22).model
            self._name_model.set_value("")
        self._name_model.add_end_edit_fn(lambda _m: self._on_object_name_commit())

        # Shape (recreate on change)
        with ui.HStack(height=22):
            ui.Label("Shape:", width=70)
            self._shape_combo = ui.ComboBox(0, *SHAPE_CHOICES, height=22)
        self._shape_combo.model.get_item_value_model().add_value_changed_fn(
            lambda _m: self._on_object_shape_change())

        # USD path (recreate on blur)
        with ui.HStack(height=22):
            ui.Label("USD path:", width=70)
            self._usd_model = ui.StringField(height=22).model
            self._usd_model.set_value("")
        self._usd_model.add_end_edit_fn(lambda _m: self._on_object_usd_commit())

        # Nucleus checkbox + preset
        with ui.HStack(height=22):
            ui.Label("Nucleus:", width=70)
            self._usd_nucleus_model = ui.SimpleBoolModel()
            self._usd_nucleus_model.set_value(False)
            ui.CheckBox(model=self._usd_nucleus_model, width=16)
            ui.Label("  (path is relative to assets_root)",
                     style={"color": 0xff888888, "font_size": 10})
        self._usd_nucleus_model.add_value_changed_fn(
            lambda _m: self._on_object_usd_commit())

        with ui.HStack(height=22):
            ui.Label("Preset:", width=70)
            preset_names = list(NUCLEUS_PRESETS.keys())
            self._usd_preset_combo = ui.ComboBox(0, *preset_names, height=22)
            self._usd_preset_combo.model.get_item_value_model().add_value_changed_fn(
                lambda _m: self._on_nucleus_preset_change())

        # Size (live)
        with ui.HStack(height=22):
            ui.Label("Size:", width=70)
            self._size_model = ui.FloatField(height=22).model
            self._size_model.set_value(0.05)
            ui.Label("Height(cyl):", width=90)
            self._height_model = ui.FloatField(height=22).model
            self._height_model.set_value(0.05)
        self._size_model.add_value_changed_fn(
            lambda _m: self._on_object_field("size", self._size_model.get_value_as_float()))
        self._height_model.add_value_changed_fn(
            lambda _m: self._on_object_field("height", self._height_model.get_value_as_float()))

        # Color (live)
        with ui.HStack(height=22):
            ui.Label("Color RGB:", width=70)
            self._r_model = ui.FloatField(height=22).model
            self._g_model = ui.FloatField(height=22).model
            self._b_model = ui.FloatField(height=22).model
            self._r_model.set_value(0.8)
            self._g_model.set_value(0.2)
            self._b_model.set_value(0.2)
        for m in (self._r_model, self._g_model, self._b_model):
            m.add_value_changed_fn(lambda _m: self._on_object_color_change())

        # Scale + scale delta (live)
        with ui.HStack(height=22):
            ui.Label("Scale:", width=70)
            self._scale_model = ui.FloatField(height=22).model
            self._scale_model.set_value(1.0)
            ui.Label("±delta:", width=60)
            self._scale_delta_model = ui.FloatField(height=22).model
            self._scale_delta_model.set_value(0.0)
        self._scale_model.add_value_changed_fn(
            lambda _m: self._on_object_field("scale", self._scale_model.get_value_as_float()))
        self._scale_delta_model.add_value_changed_fn(
            lambda _m: self._on_object_field(
                "scale_delta", self._scale_delta_model.get_value_as_float()))

        # Friction + mass (live)
        with ui.HStack(height=22):
            ui.Label("Friction:", width=70)
            self._friction_model = ui.FloatField(height=22).model
            self._friction_model.set_value(3.0)
            ui.Label("Mass(kg):", width=70)
            self._mass_model = ui.FloatField(height=22).model
            self._mass_model.set_value(0.02)
        self._friction_model.add_value_changed_fn(
            lambda _m: self._on_object_field("friction", self._friction_model.get_value_as_float()))
        self._mass_model.add_value_changed_fn(
            lambda _m: self._on_object_field("mass", self._mass_model.get_value_as_float()))

        # Position XYZ (live → set_position)
        self._obj_edit_pos_models = _xyz_row("Position:", default=(0.3, 0.0, 0.052),
                                             drag=True, step=0.01, spinner=True)
        for m in self._obj_edit_pos_models:
            m.add_value_changed_fn(lambda _m: self._on_object_position_change())

        # Position delta XYZ (live → spec; ±offset for randomize)
        self._obj_pos_delta_models = _xyz_row("Pos delta:", default=(0.0, 0.0, 0.0),
                                              step=0.01)
        for m in self._obj_pos_delta_models:
            m.add_value_changed_fn(lambda _m: self._on_object_pos_delta_change())

        # Orientation XYZ (live)
        self._obj_edit_ori_models = _xyz_row("Orient (deg):", default=(0.0, 0.0, 0.0),
                                             drag=True, step=1.0, spinner=True)
        for m in self._obj_edit_ori_models:
            m.add_value_changed_fn(lambda _m: self._on_object_orientation_change())

        # Orientation delta XYZ (live → spec; ±deg for randomize)
        self._obj_ori_delta_models = _xyz_row("Ori delta:", default=(0.0, 0.0, 0.0),
                                              step=1.0)
        for m in self._obj_ori_delta_models:
            m.add_value_changed_fn(lambda _m: self._on_object_ori_delta_change())

        ui.Label("(delta=0 → fixed / no randomization)",
                 style={"font_size": 11, "color": 0xff888888})

    # ── Camera section ────────────────────────────────

    def _build_camera_section(self):
        import omni.ui as ui
        ui.Button("+ Add Camera", clicked_fn=self._on_add_camera_click, height=28,
                  style={"background_color": 0xff006600})
        ui.Label("Added cameras", style={"font_size": 12, "color": 0xffaaaaaa})
        self._camera_list_container = ui.VStack(spacing=2, height=0)

        ui.Separator(height=4)
        self._cam_edit_name_label = ui.Label("  (nothing selected)", height=18,
                                             style={"color": 0xff888888})
        self._cam_edit_frame = ui.Frame(visible=False)
        with self._cam_edit_frame:
            with ui.VStack(spacing=4, height=0):
                self._build_camera_edit_form()
        self._rebuild_camera_list()

    def _build_camera_edit_form(self):
        import omni.ui as ui

        # Name (recreate on blur)
        with ui.HStack(height=22):
            ui.Label("Name:", width=70)
            self._cam_name_model = ui.StringField(height=22).model
        self._cam_name_model.add_end_edit_fn(lambda _m: self._on_camera_name_commit())

        # Parent prim (recreate on blur)
        with ui.HStack(height=22):
            ui.Label("Parent:", width=70)
            self._cam_parent_model = ui.StringField(height=22).model
            ui.Label("(empty=World)", style={"color": 0xff888888, "font_size": 10})
        self._cam_parent_model.add_end_edit_fn(lambda _m: self._on_camera_parent_commit())

        # Position (live)
        self._cam_pos_models = _xyz_row("Position:", default=(0.42, -1.63, 1.33),
                                        drag=True, step=0.01, spinner=True)
        for m in self._cam_pos_models:
            m.add_value_changed_fn(lambda _m: self._on_camera_live_change())

        # Orientation (live)
        self._cam_ori_models = _xyz_row("Orient (deg):", default=(51.0, 1.3, 4.0),
                                        drag=True, step=1.0, spinner=True)
        for m in self._cam_ori_models:
            m.add_value_changed_fn(lambda _m: self._on_camera_live_change())

        # Focal + Freq (focal live; freq recreate)
        with ui.HStack(height=22):
            ui.Label("Focal (mm):", width=70)
            _minus_button(lambda: self._cam_focal_model, step=1.0)
            self._cam_focal_model = ui.FloatDrag(height=22, step=1.0).model
            self._cam_focal_model.set_value(24.0)
            _plus_button(lambda: self._cam_focal_model, step=1.0)
        self._cam_focal_model.add_value_changed_fn(
            lambda _m: self._on_camera_live_change())

        with ui.HStack(height=22):
            ui.Label("Freq:", width=70)
            self._cam_freq_model = ui.IntField(height=22).model
            self._cam_freq_model.set_value(30)
            ui.Label("(recreate on change)",
                     style={"color": 0xff888888, "font_size": 10})
        self._cam_freq_model.add_end_edit_fn(lambda _m: self._on_camera_freq_commit())

        # Publish flags (recreate on change)
        with ui.HStack(height=22):
            ui.Label("Publish:", width=70)
            self._cam_rgb_model = ui.SimpleBoolModel()
            self._cam_rgb_model.set_value(True)
            ui.CheckBox(model=self._cam_rgb_model, width=16)
            ui.Label("rgb", width=40)
            self._cam_depth_model = ui.SimpleBoolModel()
            self._cam_depth_model.set_value(False)
            ui.CheckBox(model=self._cam_depth_model, width=16)
            ui.Label("depth", width=50)
        for m in (self._cam_rgb_model, self._cam_depth_model):
            m.add_value_changed_fn(lambda _m: self._on_camera_publish_change())

        # Deltas (live → spec)
        self._cam_pos_delta_models = _xyz_row("Pos delta:", default=(0.03, 0.03, 0.03),
                                              step=0.01)
        for m in self._cam_pos_delta_models:
            m.add_value_changed_fn(lambda _m: self._on_camera_delta_change())
        self._cam_ori_delta_models = _xyz_row("Ori delta:", default=(1.0, 1.0, 1.0),
                                              step=0.5)
        for m in self._cam_ori_delta_models:
            m.add_value_changed_fn(lambda _m: self._on_camera_delta_change())

    # ── Light section ────────────────────────────────

    def _build_light_section(self):
        import omni.ui as ui
        ui.Button("+ Add Light", clicked_fn=self._on_add_light_click, height=28,
                  style={"background_color": 0xff006600})
        ui.Label("Added lights", style={"font_size": 12, "color": 0xffaaaaaa})
        self._light_list_container = ui.VStack(spacing=2, height=0)

        ui.Separator(height=4)
        self._light_edit_name_label = ui.Label("  (nothing selected)", height=18,
                                               style={"color": 0xff888888})
        self._light_edit_frame = ui.Frame(visible=False)
        with self._light_edit_frame:
            with ui.VStack(spacing=4, height=0):
                self._build_light_edit_form()
        self._rebuild_light_list()

    def _build_light_edit_form(self):
        import omni.ui as ui

        # Name (recreate)
        with ui.HStack(height=22):
            ui.Label("Name:", width=70)
            self._light_name_model = ui.StringField(height=22).model
        self._light_name_model.add_end_edit_fn(lambda _m: self._on_light_name_commit())

        # Type (recreate)
        with ui.HStack(height=22):
            ui.Label("Type:", width=70)
            self._light_type_combo = ui.ComboBox(0, *LIGHT_TYPE_CHOICES, height=22)
        self._light_type_combo.model.get_item_value_model().add_value_changed_fn(
            lambda _m: self._on_light_type_change())

        # Intensity + delta (live)
        with ui.HStack(height=22):
            ui.Label("Intensity:", width=70)
            _minus_button(lambda: self._light_intensity_model, step=50.0)
            self._light_intensity_model = ui.FloatDrag(height=22, step=50.0).model
            self._light_intensity_model.set_value(1500.0)
            _plus_button(lambda: self._light_intensity_model, step=50.0)
        self._light_intensity_model.add_value_changed_fn(
            lambda _m: self._on_light_live_change())
        with ui.HStack(height=22):
            ui.Label("  ±delta:", width=70)
            self._light_intensity_delta_model = ui.FloatField(height=22).model
            self._light_intensity_delta_model.set_value(300.0)
        self._light_intensity_delta_model.add_value_changed_fn(
            lambda _m: self._on_light_delta_change())

        # Color temp + delta (live)
        with ui.HStack(height=22):
            ui.Label("ColorTemp:", width=70)
            _minus_button(lambda: self._light_ct_model, step=100.0)
            self._light_ct_model = ui.FloatDrag(height=22, step=100.0).model
            self._light_ct_model.set_value(6000.0)
            _plus_button(lambda: self._light_ct_model, step=100.0)
        self._light_ct_model.add_value_changed_fn(
            lambda _m: self._on_light_live_change())
        with ui.HStack(height=22):
            ui.Label("  ±delta:", width=70)
            self._light_ct_delta_model = ui.FloatField(height=22).model
            self._light_ct_delta_model.set_value(500.0)
        self._light_ct_delta_model.add_value_changed_fn(
            lambda _m: self._on_light_delta_change())

        # Orientation (live, distant only)
        self._light_ori_models = _xyz_row("Orient (deg):", default=(0.0, 0.0, 0.0),
                                          drag=True, step=1.0, spinner=True)
        for m in self._light_ori_models:
            m.add_value_changed_fn(lambda _m: self._on_light_live_change())

        self._light_ori_delta_models = _xyz_row("Ori delta:", default=(60.0, 60.0, 180.0),
                                                step=1.0)
        for m in self._light_ori_delta_models:
            m.add_value_changed_fn(lambda _m: self._on_light_delta_change())

        ui.Label("(ColorTemp / Orient applicable only when > 0 / distant type)",
                 style={"color": 0xff888888, "font_size": 10})

    # ── 6. Save section ────────────────────────────────

    def _build_save_section(self):
        import omni.ui as ui
        with ui.HStack(height=22):
            ui.Label("Class name:", width=90)
            self._class_model = ui.StringField(height=22).model
            self._class_model.set_value("MyEnv")
        with ui.HStack(height=22):
            ui.Label("File name:", width=90)
            self._file_model = ui.StringField(height=22).model
            self._file_model.set_value("my_env.py")
        ui.Label(f"Save to: {self._save_dir}", height=18,
                 style={"color": 0xff888888, "font_size": 10})
        ui.Button("Save Env", clicked_fn=self._on_save_click, height=32,
                  style={"background_color": 0xff0066aa})

    # ── Helpers ─────────────────────────────────────────

    def _current_shape(self) -> str:
        idx = self._shape_combo.model.get_item_value_model().get_value_as_int()
        return SHAPE_CHOICES[idx]

    def _current_light_type(self) -> str:
        idx = self._light_type_combo.model.get_item_value_model().get_value_as_int()
        return LIGHT_TYPE_CHOICES[idx]

    def _unique_name(self, base: str, existing: list[str]) -> str:
        if base not in existing:
            return base
        import re
        m = re.match(r"(.*?)(\d+)$", base)
        if m:
            prefix, num = m.group(1), int(m.group(2))
        else:
            prefix, num = base, 0
        while True:
            num += 1
            candidate = f"{prefix}{num}"
            if candidate not in existing:
                return candidate

    # ── Object handlers ────────────────────────────────

    def _on_add_object_click(self):
        existing = [s.get("name") for s in self._env.editor_object_specs()]
        name = self._unique_name("NewObj", existing)
        # 기본값: 테이블 중앙쯤, red cube
        z_default = 0.052
        cx = 0.5 * (self._spawn_x_range[0] + self._spawn_x_range[1])
        cy = 0.5 * (self._spawn_y_range[0] + self._spawn_y_range[1])
        kwargs: dict = dict(
            name=name, shape="cube", size=0.05, scale=1.0, friction=3.0, mass=0.02,
            color=(0.8, 0.2, 0.2),
            position=(float(cx), float(cy), float(z_default)),
        )
        try:
            self._env.add_editor_object(kwargs)
        except Exception as e:
            self._set_status(f"Add failed: {e}", err=True)
            return
        self._set_status(f"'{name}' added.")
        self._rebuild_object_list()
        self._on_select_object(name)

    def _on_remove_object_click(self, name: str):
        try:
            ok = self._env.remove_editor_object(name)
        except Exception as e:
            self._set_status(f"Remove failed: {e}", err=True)
            return
        if ok:
            if self._selected_object_name == name:
                self._clear_object_selection()
            self._set_status(f"'{name}' removed.")
            self._rebuild_object_list()
        else:
            self._set_status(f"'{name}' not found.", err=True)

    def _on_duplicate_object(self, name: str):
        spec = next((s for s in self._env.editor_object_specs()
                     if s.get("name") == name), None)
        if spec is None:
            self._set_status(f"'{name}' not found.", err=True)
            return
        existing = [s.get("name") for s in self._env.editor_object_specs()]
        new_name = self._unique_name(name, existing)
        new_kwargs = dict(spec)
        new_kwargs["name"] = new_name
        # 시각적 구분을 위해 position을 살짝 offset
        pos = new_kwargs.get("position")
        if pos is not None:
            new_kwargs["position"] = (float(pos[0]) + 0.05,
                                      float(pos[1]) + 0.05,
                                      float(pos[2]))
        try:
            self._env.add_editor_object(new_kwargs)
        except Exception as e:
            self._set_status(f"Duplicate failed: {e}", err=True)
            return
        self._set_status(f"Duplicated: {name} → {new_name}")
        self._rebuild_object_list()
        self._on_select_object(new_name)

    def _clear_object_selection(self):
        self._selected_object_name = None
        if self._obj_edit_frame is not None:
            self._obj_edit_frame.visible = False
        if hasattr(self, "_obj_edit_name_label"):
            self._obj_edit_name_label.text = "  (nothing selected — click + to add)"
            self._obj_edit_name_label.set_style({"color": 0xff888888})

    def _on_select_object(self, name: str):
        spec = next((s for s in self._env.editor_object_specs()
                     if s.get("name") == name), None)
        if spec is None:
            return
        self._suppress_live = True
        try:
            self._selected_object_name = name
            self._obj_edit_name_label.text = f"  Editing: [{name}]"
            self._obj_edit_name_label.set_style({"color": 0xffffaa00})
            if self._obj_edit_frame is not None:
                self._obj_edit_frame.visible = True

            # Basic fields
            self._name_model.set_value(name)

            shape = spec.get("shape") or ("USD" if spec.get("usd_path") else "cube")
            if shape in SHAPE_CHOICES:
                self._shape_combo.model.get_item_value_model().set_value(
                    SHAPE_CHOICES.index(shape))

            nucleus_rel = spec.get("_nucleus_relpath")
            if nucleus_rel:
                self._usd_model.set_value(nucleus_rel)
                self._usd_nucleus_model.set_value(True)
            else:
                self._usd_model.set_value(spec.get("usd_path") or "")
                self._usd_nucleus_model.set_value(False)

            self._size_model.set_value(float(spec.get("size") or 0.05))
            self._height_model.set_value(float(spec.get("height") or 0.05))

            color = spec.get("color") or (0.8, 0.2, 0.2)
            self._r_model.set_value(float(color[0]))
            self._g_model.set_value(float(color[1]))
            self._b_model.set_value(float(color[2]))

            self._scale_model.set_value(float(spec.get("scale") or 1.0))
            self._scale_delta_model.set_value(float(spec.get("scale_delta") or 0.0))
            self._friction_model.set_value(float(spec.get("friction") or 3.0))
            self._mass_model.set_value(float(spec.get("mass") or 0.0))

            # Position + delta
            pos = spec.get("position") or [0.3, 0.0, 0.052]
            for i in range(3):
                self._obj_edit_pos_models[i].set_value(float(pos[i]))
            pos_delta = spec.get("position_delta") or [0.0, 0.0, 0.0]
            for i in range(3):
                self._obj_pos_delta_models[i].set_value(float(pos_delta[i]))

            # Orientation + delta
            ori = spec.get("orientation") or [0.0, 0.0, 0.0]
            for i in range(3):
                self._obj_edit_ori_models[i].set_value(float(ori[i]))
            ori_delta = spec.get("orientation_delta") or [0.0, 0.0, 0.0]
            for i in range(3):
                self._obj_ori_delta_models[i].set_value(float(ori_delta[i]))
        finally:
            self._suppress_live = False

    def _update_object_field(self, **changes):
        """현재 선택된 오브젝트에 update 호출 + 상태 메시지."""
        if self._suppress_live:
            return
        name = self._selected_object_name
        if not name:
            return
        try:
            self._env.update_editor_object(name, **changes)
        except Exception as e:
            self._set_status(f"Update failed: {e}", err=True)

    def _on_object_field(self, field: str, value):
        self._update_object_field(**{field: value})

    def _on_object_color_change(self):
        self._update_object_field(color=(
            self._r_model.get_value_as_float(),
            self._g_model.get_value_as_float(),
            self._b_model.get_value_as_float(),
        ))

    def _on_object_position_change(self):
        pos = _get_xyz(self._obj_edit_pos_models)
        self._update_object_field(position=pos)

    def _on_object_orientation_change(self):
        ori = _get_xyz(self._obj_edit_ori_models)
        self._update_object_field(orientation=ori)

    def _on_object_pos_delta_change(self):
        pd = _get_xyz(self._obj_pos_delta_models)
        self._update_object_field(position_delta=tuple(abs(v) for v in pd))

    def _on_object_ori_delta_change(self):
        od = _get_xyz(self._obj_ori_delta_models)
        self._update_object_field(orientation_delta=tuple(abs(v) for v in od))

    def _on_nucleus_preset_change(self):
        """Preset 선택 시 USD path에 상대 경로 자동 입력 + Nucleus 체크 + shape=USD.
        선택 중인 오브젝트가 있으면 그 자리에서 recreate."""
        idx = self._usd_preset_combo.model.get_item_value_model().get_value_as_int()
        names = list(NUCLEUS_PRESETS.keys())
        if idx < 0 or idx >= len(names):
            return
        rel = NUCLEUS_PRESETS[names[idx]]
        if rel is None:
            return
        self._suppress_live = True
        try:
            self._usd_model.set_value(rel)
            self._usd_nucleus_model.set_value(True)
            self._shape_combo.model.get_item_value_model().set_value(
                SHAPE_CHOICES.index("USD"))
        finally:
            self._suppress_live = False
        self._on_object_usd_commit()

    def _on_object_name_commit(self):
        if self._suppress_live or not self._selected_object_name:
            return
        new_name = self._name_model.get_value_as_string().strip()
        if not new_name or new_name == self._selected_object_name:
            return
        existing = [s.get("name") for s in self._env.editor_object_specs()
                    if s.get("name") != self._selected_object_name]
        if new_name in existing:
            self._set_status(f"Name already exists: {new_name}", err=True)
            return
        try:
            self._env.recreate_editor_object(
                self._selected_object_name, dict(name=new_name))
        except Exception as e:
            self._set_status(f"Rename failed: {e}", err=True)
            return
        self._selected_object_name = new_name
        self._obj_edit_name_label.text = f"  Editing: [{new_name}]"
        self._rebuild_object_list()

    def _on_object_shape_change(self):
        if self._suppress_live or not self._selected_object_name:
            return
        new_shape = self._current_shape()
        spec = next((s for s in self._env.editor_object_specs()
                     if s.get("name") == self._selected_object_name), None)
        if spec is None:
            return
        current_shape = spec.get("shape") or ("USD" if spec.get("usd_path") else "cube")
        if new_shape == current_shape:
            return
        new_kwargs: dict = {}
        if new_shape == "USD":
            # 현재 UI에 USD path가 비어있으면 shape만 바꾸는 건 불가능.
            usd_path = self._usd_model.get_value_as_string().strip()
            if not usd_path:
                self._set_status("Set USD path first to switch to Shape=USD.", err=True)
                # Revert combo
                self._suppress_live = True
                try:
                    if current_shape in SHAPE_CHOICES:
                        self._shape_combo.model.get_item_value_model().set_value(
                            SHAPE_CHOICES.index(current_shape))
                finally:
                    self._suppress_live = False
                return
            self._on_object_usd_commit()
            return
        # 프리미티브로 전환: usd_path 제거 + shape 지정
        new_kwargs["shape"] = new_shape
        new_kwargs["usd_path"] = None
        new_kwargs["_nucleus_relpath"] = None
        try:
            self._env.recreate_editor_object(self._selected_object_name, new_kwargs)
        except Exception as e:
            self._set_status(f"Shape change failed: {e}", err=True)
            return
        self._rebuild_object_list()

    def _on_object_usd_commit(self):
        if self._suppress_live or not self._selected_object_name:
            return
        usd_path = self._usd_model.get_value_as_string().strip()
        is_nucleus = bool(self._usd_nucleus_model.get_value_as_bool())
        if not usd_path:
            return
        new_kwargs: dict = {"shape": "USD"}
        if is_nucleus:
            resolved = _resolve_nucleus_path(usd_path)
            if resolved is None:
                self._set_status("Nucleus path resolve 실패 (assets_root 없음).", err=True)
                return
            new_kwargs["usd_path"] = resolved
            new_kwargs["_nucleus_relpath"] = usd_path.lstrip("/")
        else:
            new_kwargs["usd_path"] = usd_path
            new_kwargs["_nucleus_relpath"] = None
        try:
            self._env.recreate_editor_object(self._selected_object_name, new_kwargs)
        except Exception as e:
            self._set_status(f"USD set failed: {e}", err=True)
            return
        self._rebuild_object_list()

    # ── Camera handlers ────────────────────────────────

    def _on_add_camera_click(self):
        existing = [s.get("name") for s in self._env.editor_camera_specs()]
        name = self._unique_name("new_cam", existing)
        kwargs = dict(
            name=name,
            position=[0.5, -1.0, 1.0],
            orientation=[45.0, 0.0, 0.0],
            freq=30,
            publish_rgb=True,
            publish_depth=False,
            focal_length=24.0,
            position_delta=[0.03, 0.03, 0.03],
            orientation_delta=[1.0, 1.0, 1.0],
        )
        try:
            self._env.add_editor_camera(kwargs)
        except Exception as e:
            self._set_status(f"Add camera failed: {e}", err=True)
            return
        self._set_status(f"Camera '{name}' added.")
        self._rebuild_camera_list()
        self._on_select_camera(name)

    def _clear_camera_selection(self):
        self._selected_camera_name = None
        if self._cam_edit_frame is not None:
            self._cam_edit_frame.visible = False
        if hasattr(self, "_cam_edit_name_label"):
            self._cam_edit_name_label.text = "  (nothing selected)"
            self._cam_edit_name_label.set_style({"color": 0xff888888})

    def _on_remove_camera_click(self, name: str):
        try:
            ok = self._env.remove_editor_camera(name)
        except Exception as e:
            self._set_status(f"Remove failed: {e}", err=True)
            return
        if ok:
            if self._selected_camera_name == name:
                self._clear_camera_selection()
            self._set_status(f"Camera '{name}' removed.")
            self._rebuild_camera_list()

    def _on_duplicate_camera(self, name: str):
        spec = next((s for s in self._env.editor_camera_specs()
                     if s.get("name") == name), None)
        if spec is None:
            self._set_status(f"'{name}' not found.", err=True)
            return
        existing = [s.get("name") for s in self._env.editor_camera_specs()]
        new_name = self._unique_name(name, existing)
        new_kwargs = dict(spec)
        new_kwargs["name"] = new_name
        pos = new_kwargs.get("position")
        if pos is not None:
            new_kwargs["position"] = [float(pos[0]) + 0.1,
                                      float(pos[1]) + 0.1,
                                      float(pos[2])]
        try:
            self._env.add_editor_camera(new_kwargs)
        except Exception as e:
            self._set_status(f"Duplicate failed: {e}", err=True)
            return
        self._set_status(f"Duplicated: {name} → {new_name}")
        self._rebuild_camera_list()
        self._on_select_camera(new_name)

    def _on_select_camera(self, name: str):
        spec = next((s for s in self._env.editor_camera_specs()
                     if s.get("name") == name), None)
        if spec is None:
            return
        self._suppress_live = True
        try:
            self._selected_camera_name = name
            self._cam_edit_name_label.text = f"  Editing: [{name}]"
            self._cam_edit_name_label.set_style({"color": 0xffffaa00})
            if self._cam_edit_frame is not None:
                self._cam_edit_frame.visible = True

            self._cam_name_model.set_value(name)
            self._cam_parent_model.set_value(spec.get("parent_prim") or "")
            pos = spec.get("position") or [0, 0, 0]
            ori = spec.get("orientation") or [0, 0, 0]
            for i in range(3):
                self._cam_pos_models[i].set_value(float(pos[i]))
                self._cam_ori_models[i].set_value(float(ori[i]))
            self._cam_focal_model.set_value(float(spec.get("focal_length") or 24.0))
            self._cam_freq_model.set_value(int(spec.get("freq") or 30))
            self._cam_rgb_model.set_value(bool(spec.get("publish_rgb", True)))
            self._cam_depth_model.set_value(bool(spec.get("publish_depth", False)))
            pd = spec.get("position_delta") or [0.0, 0.0, 0.0]
            od = spec.get("orientation_delta") or [0.0, 0.0, 0.0]
            for i in range(3):
                self._cam_pos_delta_models[i].set_value(float(pd[i]))
                self._cam_ori_delta_models[i].set_value(float(od[i]))
        finally:
            self._suppress_live = False

    def _on_camera_live_change(self):
        if self._suppress_live or not self._selected_camera_name:
            return
        pos = _get_xyz(self._cam_pos_models)
        ori = _get_xyz(self._cam_ori_models)
        focal = self._cam_focal_model.get_value_as_float()
        try:
            self._env.update_editor_camera(
                self._selected_camera_name,
                position=pos, orientation=ori,
                focal_length=focal if focal > 0 else None,
            )
        except Exception as e:
            self._set_status(f"Update failed: {e}", err=True)

    def _on_camera_delta_change(self):
        if self._suppress_live or not self._selected_camera_name:
            return
        pd = _get_xyz(self._cam_pos_delta_models)
        od = _get_xyz(self._cam_ori_delta_models)
        try:
            self._env.update_editor_camera(
                self._selected_camera_name,
                position_delta=pd, orientation_delta=od,
            )
        except Exception as e:
            self._set_status(f"Delta update failed: {e}", err=True)

    def _on_camera_name_commit(self):
        if self._suppress_live or not self._selected_camera_name:
            return
        new_name = self._cam_name_model.get_value_as_string().strip()
        if not new_name or new_name == self._selected_camera_name:
            return
        try:
            self._env.recreate_editor_camera(
                self._selected_camera_name, dict(name=new_name))
        except Exception as e:
            self._set_status(f"Rename failed: {e}", err=True)
            return
        self._selected_camera_name = new_name
        self._cam_edit_name_label.text = f"  Editing: [{new_name}]"
        self._rebuild_camera_list()

    def _on_camera_parent_commit(self):
        if self._suppress_live or not self._selected_camera_name:
            return
        parent = self._cam_parent_model.get_value_as_string().strip() or None
        try:
            self._env.recreate_editor_camera(
                self._selected_camera_name, dict(parent_prim=parent))
        except Exception as e:
            self._set_status(f"Parent change failed: {e}", err=True)
            return
        self._rebuild_camera_list()

    def _on_camera_freq_commit(self):
        if self._suppress_live or not self._selected_camera_name:
            return
        freq = max(1, self._cam_freq_model.get_value_as_int())
        try:
            self._env.recreate_editor_camera(
                self._selected_camera_name, dict(freq=freq))
        except Exception as e:
            self._set_status(f"Freq change failed: {e}", err=True)
            return
        self._rebuild_camera_list()

    def _on_camera_publish_change(self):
        if self._suppress_live or not self._selected_camera_name:
            return
        try:
            self._env.recreate_editor_camera(
                self._selected_camera_name,
                dict(publish_rgb=bool(self._cam_rgb_model.get_value_as_bool()),
                     publish_depth=bool(self._cam_depth_model.get_value_as_bool())),
            )
        except Exception as e:
            self._set_status(f"Publish flags failed: {e}", err=True)
            return
        self._rebuild_camera_list()

    # ── Light handlers ─────────────────────────────────

    def _on_add_light_click(self):
        existing = [s.get("name") for s in self._env.editor_light_specs()]
        name = self._unique_name("NewLight", existing)
        kwargs: dict = dict(
            type="dome", name=name,
            intensity=1500.0, intensity_delta=300.0,
            color_temp=6000.0, color_temp_delta=500.0,
        )
        try:
            self._env.add_editor_light(kwargs)
        except Exception as e:
            self._set_status(f"Add light failed: {e}", err=True)
            return
        self._set_status(f"Light '{name}' added.")
        self._rebuild_light_list()
        self._on_select_light(name)

    def _clear_light_selection(self):
        self._selected_light_name = None
        if self._light_edit_frame is not None:
            self._light_edit_frame.visible = False
        if hasattr(self, "_light_edit_name_label"):
            self._light_edit_name_label.text = "  (nothing selected)"
            self._light_edit_name_label.set_style({"color": 0xff888888})

    def _on_remove_light_click(self, name: str):
        try:
            ok = self._env.remove_editor_light(name)
        except Exception as e:
            self._set_status(f"Remove failed: {e}", err=True)
            return
        if ok:
            if self._selected_light_name == name:
                self._clear_light_selection()
            self._set_status(f"Light '{name}' removed.")
            self._rebuild_light_list()

    def _on_duplicate_light(self, name: str):
        spec = next((s for s in self._env.editor_light_specs()
                     if s.get("name") == name), None)
        if spec is None:
            self._set_status(f"'{name}' not found.", err=True)
            return
        existing = [s.get("name") for s in self._env.editor_light_specs()]
        new_name = self._unique_name(name, existing)
        new_kwargs = dict(spec)
        new_kwargs["name"] = new_name
        try:
            self._env.add_editor_light(new_kwargs)
        except Exception as e:
            self._set_status(f"Duplicate failed: {e}", err=True)
            return
        self._set_status(f"Duplicated: {name} → {new_name}")
        self._rebuild_light_list()
        self._on_select_light(new_name)

    def _on_select_light(self, name: str):
        spec = next((s for s in self._env.editor_light_specs()
                     if s.get("name") == name), None)
        if spec is None:
            return
        self._suppress_live = True
        try:
            self._selected_light_name = name
            self._light_edit_name_label.text = f"  Editing: [{name}]"
            self._light_edit_name_label.set_style({"color": 0xffffaa00})
            if self._light_edit_frame is not None:
                self._light_edit_frame.visible = True

            self._light_name_model.set_value(name)
            ltype = spec.get("type") or "dome"
            if ltype in LIGHT_TYPE_CHOICES:
                self._light_type_combo.model.get_item_value_model().set_value(
                    LIGHT_TYPE_CHOICES.index(ltype))
            self._light_intensity_model.set_value(float(spec.get("intensity") or 0.0))
            self._light_intensity_delta_model.set_value(
                float(spec.get("intensity_delta") or 0.0))
            self._light_ct_model.set_value(float(spec.get("color_temp") or 0.0))
            self._light_ct_delta_model.set_value(
                float(spec.get("color_temp_delta") or 0.0))
            ori = spec.get("orientation") or [0.0, 0.0, 0.0]
            for i in range(3):
                self._light_ori_models[i].set_value(float(ori[i]))
            od = spec.get("orientation_delta") or [0.0, 0.0, 0.0]
            for i in range(3):
                self._light_ori_delta_models[i].set_value(float(od[i]))
        finally:
            self._suppress_live = False

    def _on_light_live_change(self):
        if self._suppress_live or not self._selected_light_name:
            return
        intensity = self._light_intensity_model.get_value_as_float()
        ct = self._light_ct_model.get_value_as_float()
        ori = _get_xyz(self._light_ori_models)
        try:
            self._env.update_editor_light(
                self._selected_light_name,
                intensity=intensity if intensity > 0 else None,
                color_temp=ct if ct > 0 else None,
                orientation=ori,
            )
        except Exception as e:
            self._set_status(f"Update failed: {e}", err=True)

    def _on_light_delta_change(self):
        if self._suppress_live or not self._selected_light_name:
            return
        i_d = self._light_intensity_delta_model.get_value_as_float()
        c_d = self._light_ct_delta_model.get_value_as_float()
        o_d = _get_xyz(self._light_ori_delta_models)
        try:
            self._env.update_editor_light(
                self._selected_light_name,
                intensity_delta=i_d, color_temp_delta=c_d, orientation_delta=o_d,
            )
        except Exception as e:
            self._set_status(f"Delta update failed: {e}", err=True)

    def _on_light_name_commit(self):
        if self._suppress_live or not self._selected_light_name:
            return
        new_name = self._light_name_model.get_value_as_string().strip()
        if not new_name or new_name == self._selected_light_name:
            return
        try:
            self._env.recreate_editor_light(
                self._selected_light_name, dict(name=new_name))
        except Exception as e:
            self._set_status(f"Rename failed: {e}", err=True)
            return
        self._selected_light_name = new_name
        self._light_edit_name_label.text = f"  Editing: [{new_name}]"
        self._rebuild_light_list()

    def _on_light_type_change(self):
        if self._suppress_live or not self._selected_light_name:
            return
        ltype = self._current_light_type()
        try:
            self._env.recreate_editor_light(
                self._selected_light_name, dict(type=ltype))
        except Exception as e:
            self._set_status(f"Type change failed: {e}", err=True)
            return
        self._rebuild_light_list()

    # ── Save ───────────────────────────────────────────

    def _on_save_click(self):
        classname = self._class_model.get_value_as_string().strip() or "MyEnv"
        filename = self._file_model.get_value_as_string().strip() or "my_env.py"
        if not filename.endswith(".py"):
            filename += ".py"
        filepath = str(Path(self._save_dir) / filename)
        try:
            written = self._env.save_env_template(filepath, classname)
        except Exception as e:
            self._set_status(f"Save failed: {e}", err=True)
            return
        self._set_status(f"Saved: {written}")

    # ── List rebuilders ────────────────────────────────

    def set_save_defaults(self, classname: str | None = None,
                          filename: str | None = None):
        """Save 섹션의 Class name / File name 필드를 업데이트.

        Load Env 시점에 파일의 실제 classname을 불러와 기본값으로 채워넣는 용도.
        """
        if classname and hasattr(self, "_class_model"):
            self._class_model.set_value(classname)
        if filename and hasattr(self, "_file_model"):
            self._file_model.set_value(filename)

    def refresh_all_lists(self):
        """오브젝트 / 카메라 / 조명 목록을 다시 그린다."""
        self._rebuild_object_list()
        self._rebuild_camera_list()
        self._rebuild_light_list()
        self._selected_camera_name = None
        self._selected_light_name = None
        self._selected_object_name = None
        if hasattr(self, "_cam_edit_name_label"):
            self._cam_edit_name_label.text = "  (none selected)"
            self._cam_edit_name_label.set_style({"color": 0xff888888})
        if hasattr(self, "_light_edit_name_label"):
            self._light_edit_name_label.text = "  (none selected)"
            self._light_edit_name_label.set_style({"color": 0xff888888})
        if hasattr(self, "_obj_edit_name_label"):
            self._obj_edit_name_label.text = "  (none selected)"
            self._obj_edit_name_label.set_style({"color": 0xff888888})

    def _rebuild_object_list(self):
        import omni.ui as ui
        if self._object_list_container is None:
            return
        self._object_list_container.clear()
        with self._object_list_container:
            specs = self._env.editor_object_specs()
            if not specs:
                ui.Label("  (none)", style={"color": 0xff888888}, height=18)
                return
            for spec in specs:
                name = spec.get("name", "?")
                shape = spec.get("shape") or ("USD" if spec.get("usd_path") else "?")
                pos = spec.get("position")
                pos_delta = spec.get("position_delta")
                if pos and pos_delta and any(abs(v) > 1e-9 for v in pos_delta):
                    info = (f"@({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
                            f"±({pos_delta[0]:.2f},{pos_delta[1]:.2f},{pos_delta[2]:.2f})")
                elif pos:
                    info = f"@({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})"
                else:
                    info = ""
                with ui.HStack(height=22):
                    ui.Button(f"  {name} ({shape}) {info}", height=22,
                              clicked_fn=(lambda n=name: self._on_select_object(n)),
                              style={"background_color": 0xff222233})
                    ui.Button("Dup", width=36, height=22,
                              clicked_fn=(lambda n=name: self._on_duplicate_object(n)),
                              style={"background_color": 0xff004488})
                    ui.Button("X", width=24, height=22,
                              clicked_fn=(lambda n=name: self._on_remove_object_click(n)))

    def _rebuild_camera_list(self):
        import omni.ui as ui
        if self._camera_list_container is None:
            return
        self._camera_list_container.clear()
        with self._camera_list_container:
            specs = self._env.editor_camera_specs()
            if not specs:
                ui.Label("  (none)", style={"color": 0xff888888}, height=18)
                return
            for spec in specs:
                name = spec.get("name", "?")
                pos = spec.get("position", [0, 0, 0])
                pos_str = f"({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})"
                with ui.HStack(height=22):
                    ui.Button(f"  {name} {pos_str}", height=22,
                              clicked_fn=(lambda n=name: self._on_select_camera(n)),
                              style={"background_color": 0xff222233})
                    ui.Button("Dup", width=36, height=22,
                              clicked_fn=(lambda n=name: self._on_duplicate_camera(n)),
                              style={"background_color": 0xff004488})
                    ui.Button("X", width=24, height=22,
                              clicked_fn=(lambda n=name: self._on_remove_camera_click(n)))

    def _rebuild_light_list(self):
        import omni.ui as ui
        if self._light_list_container is None:
            return
        self._light_list_container.clear()
        with self._light_list_container:
            specs = self._env.editor_light_specs()
            if not specs:
                ui.Label("  (none)", style={"color": 0xff888888}, height=18)
                return
            for spec in specs:
                name = spec.get("name", "?")
                ltype = spec.get("type", "?")
                intensity = spec.get("intensity", 0.0)
                with ui.HStack(height=22):
                    ui.Button(f"  {name} [{ltype}] I={intensity:.0f}", height=22,
                              clicked_fn=(lambda n=name: self._on_select_light(n)),
                              style={"background_color": 0xff222233})
                    ui.Button("Dup", width=36, height=22,
                              clicked_fn=(lambda n=name: self._on_duplicate_light(n)),
                              style={"background_color": 0xff004488})
                    ui.Button("X", width=24, height=22,
                              clicked_fn=(lambda n=name: self._on_remove_light_click(n)))

    def _set_status(self, msg: str, err: bool = False):
        color = 0xffff5555 if err else 0xff55ff55
        self._status_label.text = f"Status: {msg}"
        self._status_label.set_style({"color": color})
