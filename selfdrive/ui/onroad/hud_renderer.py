import math
import shutil
import threading
import time
import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
#from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

# DP Perf Constants (字體恢復為 50)
PERF_FONT_SIZE = 50
PERF_PADDING = 14
PERF_MARGIN_BOTTOM = 0   # 設定為 0 以貼齊底部
PERF_ITEM_GAP = 50
PERF_BG_COLOR = rl.Color(0, 0, 0, 160)


@dataclass(frozen=True)
class UIConfig:
  header_height: int = 300
  border_size: int = 30
  button_size: int = 192
  set_speed_width_metric: int = 200
  set_speed_width_imperial: int = 172
  set_speed_height: int = 204
  wheel_icon_size: int = 144


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 40
  set_speed: int = 90


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  DISENGAGED = rl.Color(145, 155, 149, 255)
  OVERRIDE = rl.Color(145, 155, 149, 255)
  ENGAGED = rl.Color(128, 216, 166, 255)
  DISENGAGED_BG = rl.Color(0, 0, 0, 153)
  OVERRIDE_BG = rl.Color(145, 155, 149, 204)
  ENGAGED_BG = rl.Color(128, 216, 166, 204)
  GREY = rl.Color(166, 166, 166, 255)
  DARK_GREY = rl.Color(114, 114, 114, 255)
  BLACK_TRANSLUCENT = rl.Color(0, 0, 0, 166)
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)
  BORDER_TRANSLUCENT = rl.Color(255, 255, 255, 75)
  HEADER_GRADIENT_START = rl.Color(0, 0, 0, 114)
  HEADER_GRADIENT_END = rl.BLANK


UI_CONFIG = UIConfig()
FONT_SIZES = FontSizes()
COLORS = Colors()


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    
    # 前車距離字串與數值變數
    self.lead_dist: str = "-"
    self.lead_dist_raw: float = 0.0

    # --- TDX 路況預警變數 ---
    self.tdx_speed: int = -1
    self.tdx_status: str = "UNKNOWN"
    self.tdx_event_active: bool = False
    self.tdx_event_desc: str = ""

    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

    self._exp_button: ExpButton = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)

    #self._torque_bar = TorqueBar(scale=4.0)

    # Lincoln perf overlay init
    self._perf_font = gui_app.font(FontWeight.MEDIUM)
    self._perf_stats: dict[str, str] = {"cpu_temp": "-", "mem_usage": "-", "disk_free": "-"}
    self._perf_lock = threading.Lock()
    self._perf_running = True
    self._perf_thread = threading.Thread(target=self._perf_update_loop, daemon=True)
    self._perf_thread.start()

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      self.lead_dist = "-"
      self.lead_dist_raw = 0.0
      
      # 重置 TDX 變數，避免殘留上次熄火前的資料
      self.tdx_speed = -1
      self.tdx_status = "UNKNOWN"
      self.tdx_event_active = False
      self.tdx_event_desc = ""
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']
    radar_state = sm['radarState']

    # 紀錄真實距離並格式化字串
    if radar_state.leadOne.status:
      self.lead_dist_raw = radar_state.leadOne.dRel
      self.lead_dist = f"{self.lead_dist_raw:.0f}m"
    else:
      self.lead_dist_raw = 0.0
      self.lead_dist = "-"

    # --- 讀取 TDX 狀態 ---
    try:
      tdx = sm['tdx']
      self.tdx_speed = tdx.trafficStatus.speed
      self.tdx_status = str(tdx.trafficStatus.status)
      self.tdx_event_active = tdx.roadEvent.isActive
      
      raw_desc = str(tdx.roadEvent.description)
      # 解碼並只取純文字
      if raw_desc and ":" in raw_desc:
          loc_part, events_part = raw_desc.split(":", 1)
          clean_events = []
          for evt in events_part.split("/"):
              parts = evt.split("|")
              # 如果有代碼，就只取 | 後面的文字；否則保留原樣
              clean_events.append(parts[1] if len(parts) > 1 else evt)
          # 重新組裝為純文字供畫面繪製
          self.tdx_event_desc = f"{loc_part}: {' / '.join(clean_events)}"
      else:
          self.tdx_event_desc = raw_desc

    except Exception:
      pass

    v_cruise_cluster = car_state.vCruiseCluster
    self.set_speed = (
      controls_state.deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    if self.is_cruise_set and not ui_state.is_metric:
      self.set_speed *= KM_TO_MILE

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    # Draw the header background
    rl.draw_rectangle_gradient_v(
      int(rect.x),
      int(rect.y),
      int(rect.width),
      UI_CONFIG.header_height,
      COLORS.HEADER_GRADIENT_START,
      COLORS.HEADER_GRADIENT_END,
    )

    if self.is_cruise_available:
      self._draw_set_speed(rect)

    self._draw_current_speed(rect)

    # --- 繪製 TDX 顯示資訊 ---
    self._draw_tdx_info(rect)

    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))

    #if ui_state.sm['controlsState'].lateralControlState.which() != 'angleState':
      #self._torque_bar.render(rect)

    # Draw performance info at bottom
    self._draw_performance_info(rect)

  def user_interacting(self) -> bool:
    return self._exp_button.is_pressed

  def _draw_tdx_info(self, rect: rl.Rectangle) -> None:
    """繪製高公局即時路況與事件 (車速維持原位，事件貼齊下方單行跑馬燈)"""
    if self.tdx_speed <= 0 and not self.tdx_event_active:
      return

    bg_padding_x = 45
    bg_padding_y = 20

    # 1. 決定車速的文字顏色
    if self.tdx_status == "GREEN":
      speed_color = rl.Color(128, 216, 166, 255)
    elif self.tdx_status == "YELLOW":
      speed_color = rl.Color(255, 204, 0, 255)
    elif self.tdx_status == "RED":
      speed_color = rl.Color(255, 100, 100, 255)
    else:
      speed_color = rl.WHITE

    # ==========================================
    # 第一行: 車速 (維持在原位：頂部列下方)
    # 注意：依照你新檔案的邏輯，車速限制在 0 < 且 <= 70 才顯示
    # ==========================================
    if 0 < self.tdx_speed <= 70:
      speed_text = f"前方車速: {self.tdx_speed} km/h"
      tdx_speed_font_size = 90
      speed_size = measure_text
