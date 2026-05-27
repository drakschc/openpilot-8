"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import custom
import numpy as np
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params

# 定義加速度性格的列舉值
AccelPersonality = custom.LongitudinalPlanSP.AccelerationPersonality
ACCEL_PERSONALITY_OPTIONS = [AccelPersonality.eco, AccelPersonality.normal, AccelPersonality.sport]
# Acceleration Profiles
MAX_ACCEL_PROFILES = {
  AccelPersonality.eco:       [1.00, 0.60, 1.00, 1.40,  1.20, 1.00, 0.80, 0.60,  0.50, 0.40, 0.12, 0.08],
  AccelPersonality.normal:    [1.50, 0.80, 1.20, 1.80,  1.40, 1.20, 1.00, 0.80,  0.70, 0.60, 0.24, 0.10],
  AccelPersonality.sport:     [2.00, 1.00, 1.40, 2.00,  1.60, 1.40, 1.20, 1.00,  0.90, 0.80, 0.36, 0.12],
}
MAX_ACCEL_BREAKPOINTS =       [0.0,  0.5,  1.0,  4.0,   6.0,  9.0,  11.0, 16.0,  20.0, 25.0, 30.0, 55.0]

# Braking Profiles
MIN_ACCEL_PROFILES = {
  AccelPersonality.eco:    [-.003, -0.25, -0.35, -0.44, -2.0],
  AccelPersonality.normal: [-.004, -0.27, -0.37, -0.46, -2.0],
  AccelPersonality.sport:  [-.005, -0.29, -0.39, -0.48, -2.0],
}
MIN_ACCEL_BREAKPOINTS =    [3,     4.5,   7.,    9.,     25]


DECEL_SMOOTH_ALPHA = 0.40  # Very aggressive smoothing for decel (lower = smoother)
ACCEL_SMOOTH_ALPHA = 0.90  # Less aggressive for accel (higher = more responsive)

# Asymmetric rate limiting
MAX_DECEL_INCREASE_RATE = 1.3  # When braking harder (m/s² per second)
MAX_DECEL_DECREASE_RATE = 1.0  # When releasing brake (m/s² per second)

# ==============================================================================
# 老司機模式專用設定 (Experienced Driver Mode)
# ==============================================================================
ED_CLASS1_TIME_LIMIT = 3.0  # Class 1: 提早滑行觸發的最遠動態車距 (預設 5 秒)
ED_CLASS2_TIME_LIMIT = 1.5  # Class 2: 跟車微調介入的最遠動態車距 (預設 3 秒)


class AccelPersonalityController:
  def __init__(self):
    self.params = Params()
    self.frame = 0
    self.last_max_accel = 2.0
    self.last_min_accel = -0.01
    self.first_run = True
    self._accel_personality = self.params.get('AccelPersonality') or AccelPersonality.normal
    self._enabled = self.params.get_bool('AccelPersonalityEnabled')

    # --- 效能與防禦性快取機制 (修復 0511 架構缺陷) ---
    self._last_calc_frame = -1
    self._cached_min = -0.01
    self._cached_max = 2.0

    # ==============================================================================
    # 老司機模式開關狀態初始化 (預設開啟以便直接測試，可綁定至 OP UI 參數)
    # ==============================================================================
    self._ed_enabled = True   # 老司機總開關
    self._ed_class1 = True    # Class 1: 前車緩行或靜止提早滑行
    self._ed_class2 = True    # Class 2: 前車加速度不足時匹配並優化加速度

    # 前車狀態與老司機標記
    self._force_early_coast = False
    self._lead_status = False
    self._lead_a_lead = 0.0
    self._in_class1_dist = False      
    self._in_class2_dist = False      
    self._ed_class2_counter = 0       

  def update(self, sm=None):
    self.frame += 1

    # 每個週期重設老司機狀態，確保條件不滿足時能恢復正常控制
    self._force_early_coast = False
    self._lead_status = False
    self._lead_a_lead = 0.0
    self._in_class1_dist = False
    self._in_class2_dist = False

    # 處理老司機模式的前車雷達邏輯
    if sm is not None:
      try:
        if 'radarState' in sm and 'carState' in sm:
          v_ego = float(sm['carState'].vEgo)
          lead_one = sm['radarState'].leadOne
          self._lead_status = lead_one.status
          
          if self._lead_status:
            self._lead_a_lead = lead_one.aLeadK
            
            # 分別判斷前車是否落在 Class 1 與 Class 2 各自定義的秒數距離內
            self._in_class1_dist = lead_one.dRel <= (v_ego * ED_CLASS1_TIME_LIMIT)
            self._in_class2_dist = lead_one.dRel <= (v_ego * ED_CLASS2_TIME_LIMIT)

            # 根據車速動態計算逼近閾值：
            v_rel_thresh = float(np.interp(v_ego, [16.0, 22.0], [0.5, 1.0]))

            # Class 1 條件：有前車、相對速度小於動態閾值、距離 > 10m，且在動態車距內
            self._force_early_coast = bool(
                lead_one.vRel < -v_rel_thresh and 
                lead_one.dRel > 10.0 and
                self._in_class1_dist
            )
      except Exception:
        pass

    if self.frame % int(1.0 / DT_MDL) == 0:
      self._accel_personality = self.params.get('AccelPersonality') or AccelPersonality.normal
      self._enabled = self.params.get_bool('AccelPersonalityEnabled')

  @property
  def accel_personality(self) -> int:
    return self._accel_personality

  def get_accel_personality(self) -> int:
    return int(self._accel_personality)

  def set_accel_personality(self, personality: int):
    if personality in ACCEL_PERSONALITY_OPTIONS:
      self._accel_personality = personality
      self.params.put('AccelPersonality', personality)

  def cycle_accel_personality(self) -> int:
    current = self._accel_personality
    next_personality = ACCEL_PERSONALITY_OPTIONS[(ACCEL_PERSONALITY_OPTIONS.index(current) + 1) % len(ACCEL_PERSONALITY_OPTIONS)]
    self.set_accel_personality(next_personality)
    return int(next_personality)

  def get_accel_limits(self, v_ego: float) -> tuple[float, float]:
    # 防禦機制：若同一個 frame 被多次呼叫，直接回傳快取值，避免狀態重複修改
    if self.frame == self._last_calc_frame and not self.first_run:
      return self._cached_min, self._cached_max

    v_ego = max(0.0, v_ego)
    target_max = np.interp(v_ego, MAX_ACCEL_BREAKPOINTS, MAX_ACCEL_PROFILES[self.accel_personality])
    target_min = np.interp(v_ego, MIN_ACCEL_BREAKPOINTS, MIN_ACCEL_PROFILES[self.accel_personality])

    # ==============================================================================
    # [新增] 老司機模式介入 (Experienced Driver Mode)
    # ==============================================================================
    if self._ed_enabled:
      # 紀錄原始的 target_max 作為絕對上限
      original_t_max = target_max
        
      # Class 1: 前車緩行或靜止提早滑行
      if self._ed_class1 and self._force_early_coast:
        # 強制將加速上限壓至 0.0 (釋放動能完美滑行)
        target_max = 0.0
        self._ed_class2_counter = 0  # 阻斷並重置 Class 2 計數
            
      # Class 2: 在專屬動態車距內，且前車有數值的狀態下
      elif self._ed_class2 and self._lead_status and self._in_class2_dist:
        # 判斷原設定的加速度上限是否大於前車加速度
        if original_t_max > self._lead_a_lead:
          self._ed_class2_counter += 1
        else:
          self._ed_class2_counter = 0
                
        # 連續 4 幀 (4fps) 滿足條件才正式介入
        if self._ed_class2_counter >= 4:
          # 套用前車加速度 + 0.1，但確保絕不超過當下車速的原本加速度上限
          target_max = min(self._lead_a_lead + 0.1, original_t_max)
      else:
        # 若脫離 Class 2 車距或沒有前車，重置濾波計數器
        self._ed_class2_counter = 0
    # ==============================================================================

    if self.first_run:
      self.last_max_accel, self.last_min_accel = target_max, target_min
      self.first_run = False
      self._cached_min, self._cached_max = float(target_min), float(target_max)
      self._last_calc_frame = self.frame
      return self._cached_min, self._cached_max

    # Smoothing (0511 原生演算法)
    self.last_max_accel = ACCEL_SMOOTH_ALPHA * target_max + (1 - ACCEL_SMOOTH_ALPHA) * self.last_max_accel
    smoothed_decel = DECEL_SMOOTH_ALPHA * target_min + (1 - DECEL_SMOOTH_ALPHA) * self.last_min_accel

    # Rate Limiting (Asymmetric)
    raw_change = smoothed_decel - self.last_min_accel

    if raw_change < 0:
      limit = MAX_DECEL_INCREASE_RATE * DT_MDL
      decel_change = np.clip(raw_change, -limit, limit)
    else:
      limit = MAX_DECEL_DECREASE_RATE * DT_MDL
      decel_change = np.clip(raw_change, -limit, limit)

    self.last_min_accel += decel_change

    # Dynamic Safety Corridor
    gap = max(0.1, abs(self.last_max_accel) * 0.05)

    if self.last_min_accel > self.last_max_accel - gap:
      self.last_min_accel = self.last_max_accel - gap

    # 更新快取紀錄
    self._cached_min = float(self.last_min_accel)
    self._cached_max = float(self.last_max_accel)
    self._last_calc_frame = self.frame

    return self._cached_min, self._cached_max

  def get_min_accel(self, v_ego: float) -> float:
    return self.get_accel_limits(v_ego)[0]

  def get_max_accel(self, v_ego: float) -> float:
    return self.get_accel_limits(v_ego)[1]

  def is_enabled(self) -> bool:
    return self._enabled

  def set_enabled(self, enabled: bool):
    self._enabled = enabled
    self.params.put_bool('AccelPersonalityEnabled', enabled)

  def toggle_enabled(self) -> bool:
    current = self._enabled
    self.set_enabled(not current)
    return not current

  def reset(self):
    self._accel_personality = AccelPersonality.normal
    self.params.put('AccelPersonality', AccelPersonality.normal)
    self.frame = 0
    self.last_max_accel = 2.0
    self.last_min_accel = -0.01
    self.first_run = True
    self._last_calc_frame = -1

    # 重置老司機狀態
    self._force_early_coast = False
    self._lead_status = False
    self._lead_a_lead = 0.0
    self._in_class1_dist = False
    self._in_class2_dist = False
    self._ed_class2_counter = 0
