"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import custom
import numpy as np
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params

# 定義加速度個性化設定的列舉型態 (Eco, Normal, Sport)
AccelPersonality = custom.LongitudinalPlanSP.AccelerationPersonality
ACCEL_PERSONALITY_OPTIONS = [AccelPersonality.eco, AccelPersonality.normal, AccelPersonality.sport]

# ==============================================================================
# 加速度與減速度設定檔 (Profiles & Breakpoints)
# ==============================================================================
A_MAX_BP = [0.0,  0.5,  1.0,  4.0,   6.0,  9.0,  11.0, 16.0,  20.0, 25.0, 30.0, 55.0]
A_MAX_V = {
  AccelPersonality.eco:       [1.00, 0.60, 1.00, 1.40,  1.20, 1.00, 0.80, 0.60,  0.50, 0.40, 0.12, 0.08],
  AccelPersonality.normal:    [1.50, 0.80, 1.20, 1.80,  1.40, 1.20, 1.00, 0.80,  0.70, 0.60, 0.24, 0.10],
  AccelPersonality.sport:     [2.00, 1.00, 1.40, 2.00,  1.60, 1.40, 1.20, 1.00,  0.90, 0.80, 0.36, 0.12],
}

COAST_DRAG_BP = [0.0, 10.0, 25.0, 40.0]
COAST_DRAG_V = {
  AccelPersonality.eco:    [-0.03, -0.05, -0.08, -0.12],
  AccelPersonality.normal: [-0.04, -0.07, -0.12, -0.18],
  AccelPersonality.sport:  [-0.06, -0.10, -0.18, -0.28],
}

A_MIN_FLOOR_BP =    [3,     4.5,   7.,    9.,     25]
A_MIN_FLOOR_V = {
  AccelPersonality.eco:    [-.003, -0.25, -0.35, -0.44, -2.0],
  AccelPersonality.normal: [-.004, -0.27, -0.37, -0.46, -2.0],
  AccelPersonality.sport:  [-.005, -0.29, -0.39, -0.48, -2.0],
}

# ==============================================================================
# 控制模型常數設定
# ==============================================================================
DEFICIT_TO_FLOOR = 8.5  
COAST_DEADBAND = 0.5    
RAMP_OFF_RANGE = 3.0    

A_MIN_TIGHTEN_RATE = 1.5  
A_MIN_RELAX_RATE = 0.6    
A_MAX_RATE = 0.8          

# ==============================================================================
# 老司機模式專用設定 (Experienced Driver Mode)
# ==============================================================================
ED_CLASS1_TIME_LIMIT = 3.0  # Class 1: 提早滑行觸發的最遠動態車距 (預設 5 秒)

# Class 2: 動態濾波跟隨
ED_CLASS2_TIME_LIMIT = 1.5  # 動態加速度跟隨的最遠車距 (預設 3 秒)

# 雷達加速度 EMA 動態濾波參數 (控制 EMA 濾波時間常數 Alpha)
# 起步低速時 Alpha 大（時間常數短，動態響應極快，跨越傳動死區，起步輕快有勁）
# 高速巡航時 Alpha 小（時間常數長，提供極致的定速濾震能力，保障高速乘客舒適度）
EMA_ALPHA_BP = [0.0, 5.0, 15.0]      # 自車速度中斷點 (m/s)，分別對應 0, 18, 54 km/h
EMA_ALPHA_V  = [0.30, 0.15, 0.12]   # 各車速中斷點對應的 Alpha 權重值

MIN_MAX_GAP = 0.05
PARAM_REFRESH_FRAMES = max(1, int(1.0 / DT_MDL))


class AccelPersonalityController:
  def __init__(self):
    self.params = Params()
    self.frame = 0
    self._first = True  

    val = self.params.get('AccelPersonality')
    self._personality = val if val is not None else AccelPersonality.normal
    self._enabled = self.params.get_bool('AccelPersonalityEnabled')

    # 老司機模式開關狀態初始化
    self._ed_enabled = True   
    self._ed_class1 = True    
    self._ed_class2 = True    

    self._v_cruise = 0.0
    self._a_min = -0.05
    self._a_max = 1.50

    self._cache_v: float | None = None
    self._cache_v_cruise: float | None = None
    self._cache_a_min = self._a_min
    self._cache_a_max = self._a_max

    self._force_early_coast = False
    self._lead_status = False
    
    # 濾波器專用變數
    self._filtered_a_lead = 0.0
    self._lead_just_seen = True  
    
    self._in_class1_dist = False
    self._in_class2_dist = False

  def update(self, sm=None):
    self.frame += 1
    self._cache_v = None
    self._cache_v_cruise = None

    self._force_early_coast = False
    self._lead_status = False
    self._in_class1_dist = False
    self._in_class2_dist = False

    if sm is not None:
      try:
        v_ego = float(sm['carState'].vEgo)
        self._v_cruise = float(sm['carState'].vCruise) * CV.KPH_TO_MS

        if 'radarState' in sm:
          lead_one = sm['radarState'].leadOne
          self._lead_status = lead_one.status
          
          if self._lead_status:
            # ==============================================================================
            # 動態 EMA 濾波器實作
            # ==============================================================================
            raw_a_lead = lead_one.aLeadK
            if self._lead_just_seen:
                # 剛抓到前車時，直接賦值避免從 0 開始爬升的延遲
                self._filtered_a_lead = raw_a_lead
                self._lead_just_seen = False
            else:
                # 根據當下車速動態計算 EMA Alpha
                current_alpha = float(np.interp(v_ego, EMA_ALPHA_BP, EMA_ALPHA_V))
                # 執行指數平滑計算
                self._filtered_a_lead = (current_alpha * raw_a_lead) + ((1.0 - current_alpha) * self._filtered_a_lead)
            
            self._in_class1_dist = lead_one.dRel <= (v_ego * ED_CLASS1_TIME_LIMIT)
            self._in_class2_dist = lead_one.dRel <= (v_ego * ED_CLASS2_TIME_LIMIT)

            v_rel_thresh = float(np.interp(v_ego, [16.0, 22.0], [0.5, 1.0]))

            self._force_early_coast = bool(
                lead_one.vRel < -v_rel_thresh and 
                lead_one.dRel > 20.0 and
                self._in_class1_dist
            )
          else:
            # 沒看到前車時，重置剛抓到前車的標記，並歸零濾波數值
            self._lead_just_seen = True
            self._filtered_a_lead = 0.0

      except Exception:
        pass

    if self.frame % PARAM_REFRESH_FRAMES == 0:
      val = self.params.get('AccelPersonality')
      self._personality = val if val is not None else AccelPersonality.normal
      self._enabled = self.params.get_bool('AccelPersonalityEnabled')

  @property
  def accel_personality(self) -> int:
    return self._personality

  def get_accel_personality(self) -> int:
    return int(self._personality)

  def set_accel_personality(self, personality: int):
    if personality in ACCEL_PERSONALITY_OPTIONS:
      self._personality = personality
      self.params.put('AccelPersonality', personality)

  def cycle_accel_personality(self) -> int:
    idx = ACCEL_PERSONALITY_OPTIONS.index(self._personality) if self._personality in ACCEL_PERSONALITY_OPTIONS else 0
    nxt = ACCEL_PERSONALITY_OPTIONS[(idx + 1) % len(ACCEL_PERSONALITY_OPTIONS)]
    self.set_accel_personality(nxt)
    return int(nxt)

  def is_enabled(self) -> bool:
    return self._enabled

  def set_enabled(self, enabled: bool):
    self._enabled = bool(enabled)
    self.params.put_bool('AccelPersonalityEnabled', self._enabled)

  def toggle_enabled(self) -> bool:
    self.set_enabled(not self._enabled)
    return self._enabled

  def reset(self, personality: int | None = None):
    if personality is None or personality not in ACCEL_PERSONALITY_OPTIONS:
      personality = AccelPersonality.normal
    self._personality = personality
    self.params.put('AccelPersonality', self._personality)
    self.frame = 0
    self._first = True
    self._a_min = -0.05
    self._a_max = 1.50
    self._cache_v = None
    self._cache_v_cruise = None
    
    self._force_early_coast = False
    self._lead_status = False
    self._filtered_a_lead = 0.0
    self._lead_just_seen = True
    self._in_class1_dist = False
    self._in_class2_dist = False

  def get_accel_limits(self, v_ego: float) -> tuple[float, float]:
    v_ego = max(0.0, v_ego)
    if (self._cache_v is not None
        and abs(self._cache_v - v_ego) < 0.01
        and self._cache_v_cruise == self._v_cruise):
      return self._cache_a_min, self._cache_a_max

    self._cache_a_min, self._cache_a_max = self._step(v_ego)
    self._cache_v = v_ego
    self._cache_v_cruise = self._v_cruise
    return self._cache_a_min, self._cache_a_max

  def get_min_accel(self, v_ego: float) -> float:
    return self.get_accel_limits(v_ego)[0]

  def get_max_accel(self, v_ego: float) -> float:
    return self.get_accel_limits(v_ego)[1]

  def _ramp_off(self, v_ego: float) -> float:
    if self._v_cruise <= 0.0:
      return 1.0
    return float(np.clip((self._v_cruise - v_ego) / RAMP_OFF_RANGE, 0.0, 1.0))

  def _target_max(self, v_ego: float) -> float:
    base = float(np.interp(v_ego, A_MAX_BP, A_MAX_V[self._personality]))
    return base * self._ramp_off(v_ego)

  def _target_min(self, v_ego: float) -> float:
    coast = float(np.interp(v_ego, COAST_DRAG_BP, COAST_DRAG_V[self._personality]))
    if self._v_cruise <= 0.0 or v_ego >= self._v_cruise:
      return coast

    floor = float(np.interp(v_ego, A_MIN_FLOOR_BP, A_MIN_FLOOR_V[self._personality]))
    deficit = self._v_cruise - v_ego
    t = float(np.clip(deficit / DEFICIT_TO_FLOOR, 0.0, 1.0)) ** 1.5
    return coast + t * (floor - coast)

  def _apply_coast_deadband(self, v_ego: float, t_min: float, t_max: float) -> tuple[float, float]:
    if self._v_cruise <= 0.0 or abs(v_ego - self._v_cruise) >= COAST_DEADBAND:
      return t_min, t_max
    coast = float(np.interp(v_ego, COAST_DRAG_BP, COAST_DRAG_V[self._personality]))
    return coast, max(0.05, t_max * 0.25)

  def _rate_limit(self, last: float, target: float, rate_down: float, rate_up: float) -> float:
    rate = rate_up if target > last else rate_down
    step = rate * DT_MDL
    return float(np.clip(target, last - step, last + step))

  def _step(self, v_ego: float) -> tuple[float, float]:
    t_max = self._target_max(v_ego)
    t_min = self._target_min(v_ego)

    t_min, t_max = self._apply_coast_deadband(v_ego, t_min, t_max)

    # ==============================================================================
    # 老司機模式管理 (Experienced Driver Mode)
    # ==============================================================================
    if self._ed_enabled:
        
        original_t_max = t_max
        
        # Class 1: 前車緩行或靜止提早滑行
        if self._ed_class1 and self._force_early_coast:
            t_max = -1e-3
            
        # Class 2: 動態濾波雷達跟隨
        elif self._ed_class2 and self._lead_status:
            # 在 1.25 秒動態車距內，直接採信過濾後的平滑加速度
            if self._in_class2_dist:
                # 只有當原始上限大於前車過濾後的加速度時才介入抑制
                if original_t_max > self._filtered_a_lead:
                    # 套用動態平滑處理後的前車加速度 + 0.1
                    t_max = min(self._filtered_a_lead + 0.1, original_t_max)

    # ==============================================================================

    if self._first:
      self._a_min, self._a_max = t_min, t_max
      self._first = False
      return self._a_min, self._a_max

    new_min = self._rate_limit(self._a_min, t_min, rate_down=A_MIN_TIGHTEN_RATE, rate_up=A_MIN_RELAX_RATE)
    new_max = self._rate_limit(self._a_max, t_max, rate_down=A_MAX_RATE, rate_up=A_MAX_RATE)

    new_min = min(new_min, new_max - MIN_MAX_GAP)

    self._a_min, self._a_max = new_min, new_max
    return self._a_min, self._a_max
