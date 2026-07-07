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

# --- 方向燈與盲區閃爍頻率設定 ---
DP_INDICATOR_BLINK_RATE_FAST = int(gui_app.target_fps * 0.25)
DP_INDICATOR_BLINK_RATE_STD = int(gui_app.target_fps * 0.5)
DP_INDICATOR_COLOR_BSM = rl.Color(255, 204, 0, 220)      # 盲區黃色
DP_INDICATOR_COLOR_BLINKER = rl.Color(0, 255, 0, 220)    # 方向燈綠色


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

    # --- 邊緣閃爍狀態變數 ---
    self._dp_indicator_show_left = False
    self._dp_indicator_show_right = False
    self._dp_indicator_count_left = 0
    self._dp_indicator_count_right = 0
    self._dp_indicator_color_left = rl.Color(0, 0, 0, 0)
    self._dp_indicator_color_right = rl.Color(0, 0, 0, 0)

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

  def _update_dp_indicator_side_state(self, blinker_state, bsm_state, show_prev, count_prev):
    """處理單邊的閃爍與顏色邏輯"""
    show = show_prev
    count = count_prev
    color = rl.Color(0, 0, 0, 0)

    if not blinker_state and not bsm_state:
      show = False
      count = 0
    else:
      count += 1

    if bsm_state and blinker_state:
      show = not show if count % DP_INDICATOR_BLINK_RATE_FAST == 0 else show
      color = DP_INDICATOR_COLOR_BSM
    elif blinker_state:
      show = not show if count % DP_INDICATOR_BLINK_RATE_STD == 0 else show
      color = DP_INDICATOR_COLOR_BLINKER
    elif bsm_state:
      show = True
      color = DP_INDICATOR_COLOR_BSM
    else:
      show = False

    return show, count, color

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
      
      self._dp_indicator_show_left = False
      self._dp_indicator_show_right = False
      return

    # --- 讀取雷達狀態 ---
    radar_state = sm['radarState']
    if radar_state.leadOne.status:
      self.lead_dist_raw = radar_state.leadOne.dRel
      self.lead_dist = f"{self.lead_dist_raw:.0f}m"
    else:
      self.lead_dist_raw = 0.0
      self.lead_dist = "-"

    # --- 讀取 TDX 狀態 (顯示標題) ---
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
              label_events = []
              for evt in events_part.split("/"):
                  parts = evt.split("|")
                  evt_type = parts[0] if len(parts) > 1 else '0'
                  label = EVENT_TYPE_LABEL.get(evt_type, '[其他]')
                  label_events.append(label)

              unique_labels = []
              for lbl in label_events:
                  if lbl not in unique_labels:
                      unique_labels.append(lbl)

              self.tdx_event_desc = f"前方:{''.join(unique_labels)}"
          else:
              self.tdx_event_desc = ""
      else:
          self.tdx_event_desc = ""

    except Exception:
      pass

    controls_state = sm['controlsState']
    car_state = sm['carState']

    # --- 更新兩側方向燈與盲區閃爍狀態 ---
    self._dp_indicator_show_left, self._dp_indicator_count_left, self._dp_indicator_color_left = \
      self._update_dp_indicator_side_state(car_state.leftBlinker, car_state.leftBlindspot,
                                           self._dp_indicator_show_left, self._dp_indicator_count_left)
    
    self._dp_indicator_show_right, self._dp_indicator_count_right, self._dp_indicator_color_right = \
      self._update_dp_indicator_side_state(car_state.rightBlinker, car_state.rightBlindspot,
                                           self._dp_indicator_show_right, self._dp_indicator_count_right)

    # =========================================================================
    # --- 測試模式：模擬方向燈與盲區來回顯示 (已註解關閉) ---
    # =========================================================================
    # t = time.time()
    # cycle = int(t / 2) % 6  # 每 2 秒切換一個情境
    #
    # self.left_blinker = False
    # self.right_blinker = False
    # self.left_blindspot = False
    # self.right_blindspot = False
    #
    # is_blinking = int(t * 2) % 2 == 0  # 每 0.5 秒閃爍
    #
    # if cycle == 0:
    #     self.left_blinker = is_blinking
    # elif cycle == 1:
    #     self.right_blinker = is_blinking
    # elif cycle == 2:
    #     self.left_blindspot = True
    # elif cycle == 3:
    #     self.right_blindspot = True
    # elif cycle == 4:
    #     self.left_blinker = is_blinking
    #     self.left_blindspot = True
    # elif cycle == 5:
    #     self.right_blinker = is_blinking
    #     self.right_blindspot = True
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
    """Render HUD elements
