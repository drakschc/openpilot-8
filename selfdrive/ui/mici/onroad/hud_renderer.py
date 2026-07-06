import math
import time
import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter
from cereal import log

EventName = log.OnroadEvent.EventName

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

SET_SPEED_PERSISTENCE = 2.5  # seconds


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 36
  set_speed: int = 112


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)


FONT_SIZES = FontSizes()
COLORS = Colors()


class TurnIntent(Widget):
  FADE_IN_ANGLE = 30  # degrees

  def __init__(self):
    super().__init__()
    self._pre = False
    self._turn_intent_direction: int = 0

    self._turn_intent_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._turn_intent_rotation_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._txt_turn_intent_left: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20)
    self._txt_turn_intent_right: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20, flip_x=True)

  def _render(self, _):
    if self._turn_intent_alpha_filter.x > 1e-2:
      turn_intent_texture = self._txt_turn_intent_right if self._turn_intent_direction == 1 else self._txt_turn_intent_left
      src_rect = rl.Rectangle(0, 0, turn_intent_texture.width, turn_intent_texture.height)
      dest_rect = rl.Rectangle(self._rect.x + self._rect.width / 2, self._rect.y + self._rect.height / 2,
                               turn_intent_texture.width, turn_intent_texture.height)

      origin = (turn_intent_texture.width / 2, self._rect.height / 2)
      color = rl.Color(255, 255, 255, int(255 * self._turn_intent_alpha_filter.x))
      rl.draw_texture_pro(turn_intent_texture, src_rect, dest_rect, origin, self._turn_intent_rotation_filter.x, color)

  def _update_state(self) -> None:
    sm = ui_state.sm

    left = any(e.name == EventName.preLaneChangeLeft for e in sm['onroadEvents'])
    right = any(e.name == EventName.preLaneChangeRight for e in sm['onroadEvents'])
    if left or right:
      # pre lane change
      if not self._pre:
        self._turn_intent_rotation_filter.x = self.FADE_IN_ANGLE if left else -self.FADE_IN_ANGLE

      self._pre = True
      self._turn_intent_direction = -1 if left else 1
      self._turn_intent_alpha_filter.update(1)
      self._turn_intent_rotation_filter.update(0)
    elif any(e.name == EventName.laneChange for e in sm['onroadEvents']):
      # fade out and rotate away
      self._pre = False
      self._turn_intent_alpha_filter.update(0)

      if self._turn_intent_direction == 0:
        # unknown. missed pre frame?
        self._turn_intent_rotation_filter.update(0)
      else:
        self._turn_intent_rotation_filter.update(self._turn_intent_direction * self.FADE_IN_ANGLE)
    else:
      # didn't complete lane change, just hide
      self._pre = False
      self._turn_intent_direction = 0
      self._turn_intent_alpha_filter.update(0)
      self._turn_intent_rotation_filter.update(0)


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self._set_speed_changed_time: float = 0
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self._engaged: bool = False

    self.tdx_event_active: bool = False
    self.tdx_event_desc: str = ""

    # --- 前車距離變數 ---
    self.lead_dist: str = "-"
    self.lead_dist_raw: float = 0.0

    # --- 新增方向燈與盲區狀態變數 ---
    self.left_blinker: bool = False
    self.right_blinker: bool = False
    self.left_blindspot: bool = False
    self.right_blindspot: bool = False

    self._can_draw_top_icons = True
    self._show_wheel_critical = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)

    self._turn_intent = TurnIntent()
    self._torque_bar = TorqueBar()

    self._txt_wheel: rl.Texture = gui_app.texture('icons_mici/wheel.png', 50, 50)
    self._txt_wheel_critical: rl.Texture = gui_app.texture('icons_mici/wheel_critical.png', 50, 50)
    self._txt_exclamation_point: rl.Texture = gui_app.texture('icons_mici/exclamation_point.png', 9, 44)

    self._wheel_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._wheel_y_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._set_speed_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def set_wheel_critical_icon(self, critical: bool):
    """Set the wheel icon to critical or normal state."""
    self._show_wheel_critical = critical

  def set_can_draw_top_icons(self, can_draw_top_icons: bool):
    """Set whether to draw the top part of the HUD."""
    self._can_draw_top_icons = can_draw_top_icons

  def drawing_top_icons(self) -> bool:
    # whether we're drawing any top icons currently
    return bool(self._set_speed_alpha_filter.x > 1e-2)

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      self.tdx_event_active = False
      self.tdx_event_desc = ""
      self.lead_dist = "-"
      self.lead_dist_raw = 0.0
      
      self.left_blinker = False
      self.right_blinker = False
      self.left_blindspot = False
      self.right_blindspot = False
      return

    # --- 讀取雷達狀態 ---
    radar_state = sm['radarState']
    if radar_state.leadOne.status:
      self.lead_dist_raw = radar_state.leadOne.dRel
      self.lead_dist = f"{self.lead_dist_raw:.0f}m"
    else:
      self.lead_dist_raw = 0.0
      self.lead_dist = "-"

    # --- 讀取 TDX 狀態 (僅保留前方路段) ---
    try:
      tdx = sm['tdx']
      self.tdx_event_active = tdx.roadEvent.isActive
      raw_desc = str(tdx.roadEvent.description)

      EVENT_TYPE_LABEL = {
          '1': '[事故]', '2': '[施工]', '3': '[壅塞]',
          '4': '[管制]', '5': '[天氣]', '8': '[異常]'
      }

      if raw_desc and ":" in raw_desc:
          loc_part, events_part = raw_desc.split(":", 1)
          
          if "前方" in loc_part:
              loc_part = "前方路段"
              label_events = []
              for evt in events_part.split("/"):
                  parts = evt.split("|")
                  evt_type = parts[0] if len(parts) > 1 else '0'
                  label_events.append(EVENT_TYPE_LABEL.get(evt_type, '[其他]'))

              unique_labels = []
              for lbl in label_events:
                  if lbl not in unique_labels:
                      unique_labels.append(lbl)

              self.tdx_event_desc = f"{loc_part}:{ ''.join(unique_labels) }"
          else:
              self.tdx_event_desc = ""
      else:
          self.tdx_event_desc = ""

    except Exception:
      pass

    controls_state = sm['controlsState']
    car_state = sm['carState']

    # =========================================================================
    # --- 測試模式：模擬方向燈與盲區來回顯示 ---
    # =========================================================================
    t = time.time()
    cycle = int(t / 2) % 6  # 每 2 秒切換一個情境

    self.left_blinker = False
    self.right_blinker = False
    self.left_blindspot = False
    self.right_blindspot = False

    is_blinking = int(t * 2) % 2 == 0  # 每 0.5 秒閃爍

    if cycle == 0:
        self.left_blinker = is_blinking
    elif cycle == 1:
        self.right_blinker = is_blinking
    elif cycle == 2:
        self.left_blindspot = True
    elif cycle == 3:
        self.right_blindspot = True
    elif cycle == 4:
        self.left_blinker = is_blinking
        self.left_blindspot = True
    elif cycle == 5:
        self.right_blinker = is_blinking
        self.right_blindspot = True
    # =========================================================================

    v_cruise_cluster = car_state.vCruiseCluster
    set_speed = (
      controls_state.deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    engaged = sm['selfdriveState'].enabled
    if (set_speed != self.set_speed and engaged) or (engaged and not self._engaged):
      self._set_speed_changed_time = rl.get_time()
    self._engaged = engaged
    self.set_speed = set_speed
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    
    if self.is_cruise_set:
      self._draw_set_speed(rect)

    # 顯示動態球體與距離
    self._draw_lead_info(rect)
    
    # 繪製 TDX 警告
    self._draw_tdx_info(rect)

    # 繪製邊緣方向燈與盲區
    self._draw_edge_warnings(rect)

  def _draw_edge_warnings(self, rect: rl.Rectangle) -> None:
    """繪製兩側方向燈與盲區警示 (寬度減半、高度設為 60%、靠上方對齊避開球體)"""
    bar_width = 30  # 寬度從 60 減半為 30
    bar_height = int(rect.height * 0.60) # 高度設為畫面 60%
    y_pos = int(rect.y + 20) # 靠上方對齊，距離頂部留 20px 邊距

    # 左側邊條
    if self.left_blindspot:
      rl.draw_rectangle(int(rect.x), y_pos, bar_width, bar_height, rl.Color(255, 204, 0, 220)) 
    elif self.left_blinker:
      rl.draw_rectangle(int(rect.x), y_pos, bar_width, bar_height, rl.Color(0, 255, 0, 220)) 

    # 右側邊條
    if self.right_blindspot:
      rl.draw_rectangle(int(rect.x + rect.width - bar_width), y_pos, bar_width, bar_height, rl.Color(255, 204, 0, 220)) 
    elif self.right_blinker:
      rl.draw_rectangle(int(rect.x + rect.width - bar_width), y_pos, bar_width, bar_height, rl.Color(0, 255, 0, 220)) 

  def _draw_lead_info(self, rect: rl.Rectangle) -> None:
    """繪製球體與前車距離"""
    pos_x = int(rect.x + 46)
    pos_y = int(rect.y + rect.height - 39)
    
    ball_color = rl.RED
    dist_color = rl.WHITE
    
    if self.lead_dist != "-":
      if self.lead_dist_raw < 15.0:
        ball_color = rl.RED  
        dist_color = rl.Color(255, 100, 100, 255) 
      else:
        ball_color = rl.GREEN 
        dist_color = rl.Color(128, 216, 166, 255) 
    
    rl.draw_circle(pos_x, pos_y, 25, ball_color)

    dist_text = self.lead_dist
    dist_font_size = 40
    dist_size = measure_text_cached(self._font_bold, dist_text, dist_font_size)
    
    text_x = pos_x + 35  
    text_y = pos_y - dist_size.y / 2
        
    rl.draw_text_ex(self._font_bold, dist_text, rl.Vector2(text_x, text_y), dist_font_size, 0, dist_color)

  def _draw_tdx_info(self, rect: rl.Rectangle) -> None:
    """TDX 路況警告：防範干擾兩側盲區"""
    if not self.tdx_event_active or not self.tdx_event_desc:
      return

    bar_width = 30

    # 繪製全區半透明黑色遮罩，避開兩側盲區區域
    safe_x = int(rect.x + bar_width)
    safe_width = int(rect.width - bar_width * 2)
    rl.draw_rectangle(safe_x, int(rect.y), safe_width, int(rect.height), rl.Color(0, 0, 0, 120))

    # 字體設定
    font_size = 60
    text_size = measure_text_cached(self._font_bold, self.tdx_event_desc, font_size)
    
    bg_padding_x = 25
    bg_padding_y = 15

    max_text_width = safe_width - bg_padding_x * 2 - 20 
    display_width = min(text_size.x, max_text_width)

    pos_x = rect.x + (rect.width - display_width) / 2
    pos_y = rect.y + (rect.height - text_size.y) / 2
    
    bg_rect = rl.Rectangle(
        pos_x - bg_padding_x, 
        pos_y - bg_padding_y, 
        display_width + bg_padding_x * 2, 
        text_size.y + bg_padding_y * 2
    )
    
    alpha = 150 + int(60 * math.sin(time.time() * 5))
    rl.draw_rectangle_rounded(bg_rect, 0.2, 10, rl.Color(220, 50, 50, alpha))
    
    if text_size.x > max_text_width:
      rl.begin_scissor_mode(int(bg_rect.x), int(bg_rect.y), int(bg_rect.width), int(bg_rect.height))

      extra_width = text_size.x - max_text_width
      scroll_speed = 80.0     
      scroll_duration = extra_width / scroll_speed
      pause_duration = 2.0    

      cycle_time = time.time() % ((scroll_duration + pause_duration) * 2)

      if cycle_time < pause_duration:
        offset = 0.0
      elif cycle_time < pause_duration + scroll_duration:
        progress = (cycle_time - pause_duration) / scroll_duration
        offset = extra_width * progress
      elif cycle_time < pause_duration * 2 + scroll_duration:
        offset = extra_width
      else:
        progress = (cycle_time - pause_duration * 2 - scroll_duration) / scroll_duration
        offset = extra_width * (1 - progress)

      draw_x = pos_x - offset
      rl.draw_text_ex(self._font_bold, self.tdx_event_desc, rl.Vector2(draw_x, pos_y), font_size, 0, rl.WHITE)

      rl.end_scissor_mode()
    else:
      rl.draw_text_ex(self._font_bold, self.tdx_event_desc, rl.Vector2(pos_x, pos_y), font_size, 0, rl.WHITE)

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    wheel_txt = self._txt_wheel_critical if self._show_wheel_critical else self._txt_wheel

    if self._show_wheel_critical:
      self._wheel_alpha_filter.update(255)
      self._wheel_y_filter.update(0)
    else:
      if ui_state.status == UIStatus.DISENGAGED and not ui_state.dp_alka_active:
        self._wheel_alpha_filter.update(0)
        self._wheel_y_filter.update(wheel_txt.height / 2)
      else:
        self._wheel_alpha_filter.update(255 * 0.9)
        self._wheel_y_filter.update(0)

    pos_x = int(rect.x + 21 + wheel_txt.width / 2)
    pos_y = int(rect.y + rect.height - 14 - wheel_txt.height / 2 + self._wheel_y_filter.x)
    rotation = -ui_state.sm['carState'].steeringAngleDeg

    turn_intent_margin = 25
    self._turn_intent.render(rl.Rectangle(
      pos_x - wheel_txt.width / 2 - turn_intent_margin,
      pos_y - wheel_txt.height / 2 - turn_intent_margin,
      wheel_txt.width + turn_intent_margin * 2,
      wheel_txt.height + turn_intent_margin * 2,
    ))

    src_rect = rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height)
    dest_rect = rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height)
    origin = (wheel_txt.width / 2, wheel_txt.height / 2)

    color = rl.Color(255, 255, 255, int(self._wheel_alpha_filter.x))
    rl.draw_texture_pro(wheel_txt, src_rect, dest_rect, origin, rotation, color)

    if self._show_wheel_critical:
      EXCLAMATION_POINT_SPACING = 10
      exclamation_pos_x = pos_x - self._txt_exclamation_point.width / 2 + wheel_txt.width / 2 + EXCLAMATION_POINT_SPACING
      exclamation_pos_y = pos_y - self._txt_exclamation_point.height / 2
      rl.draw_texture_ex(self._txt_exclamation_point, rl.Vector2(exclamation_pos_x, exclamation_pos_y), 0.0, 1.0, rl.WHITE)

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    alpha = self._set_speed_alpha_filter.update(0 < rl.get_time() - self._set_speed_changed_time < SET_SPEED_PERSISTENCE and
                                                self._can_draw_top_icons and self._engaged)
    if alpha < 1e-2:
      return

    x = rect.x
    y = rect.y

    circle_radius = 162 // 2
    rl.draw_circle_gradient(rl.Vector2(x + circle_radius, y + circle_radius), circle_radius,
                            rl.Color(0, 0, 0, int(255 / 2 * alpha)), rl.BLANK)

    set_speed_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))
    max_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))

    set_speed = self.set_speed
    if self.is_cruise_set and not ui_state.is_metric:
      set_speed *= KM_TO_MILE

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(set_speed))
    rl.draw_text_ex(
      self._font_display,
      set_speed_text,
      rl.Vector2(x + 13 + 4, y + 3 - 8 - 3 + 4),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

    max_text = tr("MAX")
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + 25, y + FONT_SIZES.set_speed - 7 + 4),
      FONT_SIZES.max_speed,
      0,
      max_color,
    )

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)
