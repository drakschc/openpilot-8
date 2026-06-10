#!/usr/bin/env python3
import capnp
import numpy as np
from typing import Any
from cereal import messaging, car
from opendbc.car import structs

# ==============================================================================
# 1. 引入整個 radard 模組進行 Monkey Patch (動態替換)
# ==============================================================================
from openpilot.selfdrive.controls import radard

# 2. 正常引入我們需要的元件與原始函數
from openpilot.selfdrive.controls.radard import (
    KalmanParams, Track, RadarD, match_vision_to_track,
    get_RadarState_from_vision, get_custom_yrel, RADAR_TO_CAMERA
)

# 3. 引入 cloudlog 用於記錄我們自訂的提早鎖定事件
from openpilot.common.swaglog import cloudlog

# ==============================================================================
# 提早鎖定 (Early Lock) 擴充模組參數設定
# ==============================================================================
# 📐 階段一與二：空間遲滯與物理打分邊界
LANE_WIDTH_FALLBACK = 1.5           # 預測車道基準單側半寬 (m)，無實體線時使用
LANE_HYSTERESIS_MARGIN = 0.5        # 邊界外的遲滯容錯預度 (m)，完美消滅邊緣抽搐閃爍
FUZZY_BOUNDS = [0.5, 1.5]           # 物理誤差 (m 或 m/s): 0.5 以內給滿分 1.0，大於 1.5 總分歸零

# 🧠 階段三：EMA 基礎學習率設定
ALPHA_BASE = 0.2                    # 常規上升學習率 (確保真車在 0.2~0.4 秒內穩定鎖定)
ALPHA_DOWN = 0.1                    # 常規下降與短路過濾時的衰減學習率 (極速踢出雜訊)

# 🔥 階段三：威脅乘數融合矩陣 (限制最高 1.2 倍，保留空間讓 EMA 平滑爬升)
BRAKE_THRES_RANGE = [-3.0, -1.2]    # 急煞觸發區間 (m/s²)
MULT_RANGE = [1.2, 1.0]             # 對應威脅倍率 (從最高 1.2 緩降至 1.0)
CUTIN_DIST_LIMIT = 40.0             # 評估切入威脅的最大縱向有效距離 (m)
DYNAMIC_SPEED_PCT = 0.2             # 動態相對速度閥值比例 (以當前車速的 20% 為基準)

# 🐢 階段四：慢速與靜止防護網 (基於電腦視覺物理極限的反轉門檻)
CAM_PROB_SPEED_RANGE = [10.0, 25.0] # 動態相機門檻車速區間 (約 36 ~ 90 km/h)
CAM_PROB_RANGE = [0.5, 0.3]         # 動態相機審查門檻 (市區低速要求 0.5 嚴格把關，高速放寬至 0.3)
STATIC_EMA_CAP = 0.6                # 目標未達審查門檻時，EMA 絕對不可超越的天花板

# 🎛️ 階段五：全局反向插值映射
EMA_VAL_RANGE = [0.4, 0.8]          # 本地 EMA 信心度 X 軸 (0.4 起跳，0.8 達峰值)
PROB_THRES_RANGE = [0.5, 0.3]       # 映射出對應的「視覺提早放行門檻」 Y 軸 (最高可強勢拉低至 0.3)

# 🛡️ 階段六：狀態機記憶與抗斷流防護 (防閃爍機制)
RELEASE_FRAMES = 5                  # 目標短暫丟失或出界時的 EMA 續命凍結幀數 (約 0.25 秒)
SELECT_HOLDOVER_FRAMES = 3          # 雷達硬體斷流時，強制維持上一幀鎖定的幀數 (約 0.15 秒)

# ⚖️ 階段七：視覺加速度雙重驗證阻尼 (防幽靈急煞)
MODEL_TAU_MIN_PROB = 0.5            # 啟動驗證的最低視覺機率
MODEL_TAU_BRAKE_A = -0.5            # 啟動驗證的最低急煞門檻 (m/s²)
MODEL_TAU_SUSTAINED = 0.5           # 視覺確認急煞持續 (給予小阻尼，讓底層全力煞車)
MODEL_TAU_SPURIOUS = 3.0            # 視覺預測即將加速 (給予大阻尼，快速化解幽靈急煞訊號)

# ==============================================================================
# 全域快取：用於儲存上一幀的鎖定目標，修補硬體層級的斷流
# ==============================================================================
_LEAD_STATE_CACHE = {
    0: {'track': None, 'absent': 0},
    1: {'track': None, 'absent': 0}
}


def get_model_lead_tau(lead_msg, lead_prob: float) -> float | None:
  """
  讀取相機模型對未來加速度的預測，進行交叉驗證。
  當雷達判定急煞，但相機認為即將加速時，賦予極大阻尼，化解潛在危險。
  全面改用外部傳入的平滑化 lead_prob 取代 lead_msg.prob。
  """
  if lead_prob < MODEL_TAU_MIN_PROB or len(lead_msg.a) < 2:
    return None
  
  a0 = float(lead_msg.a[0])
  a1 = float(lead_msg.a[1])
  
  if a0 > MODEL_TAU_BRAKE_A:
    return None
  if a1 < 0.5 * a0:
    return MODEL_TAU_SUSTAINED
  if a1 > 0.1 * a0:
    return MODEL_TAU_SPURIOUS
  
  return None


class TrackSP(Track):
  """
  繼承自原始 Track，加入非對稱 EMA 信心度、模糊邏輯打分、動態威脅乘數，
  以及抗干擾的短期續命狀態機，確保在硬體閃爍時依然提供平滑的數值。
  """

  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    super().__init__(identifier, v_lead, kalman_params)
    # 【極速起跑點】：初始信心度設定為 0.4，配合數學斜率迅速突破門檻。
    self.ema_confidence = {0: 0.4, 1: 0.4}
    # 【續命計時器】：當目標短暫消失或出界時，給予緩衝幀數防止 EMA 遭誤殺。
    self.holdover_frames = {0: 0, 1: 0}
    self.is_out_of_lane = False

  def _check_spatial_boundaries(self, vision_y: float) -> bool:
    """
    【階段一：空間幾何絕對邊界審查】建立遲滯狀態機，過濾車道外的無效雜訊。
    """
    left_bound = vision_y + LANE_WIDTH_FALLBACK
    right_bound = vision_y - LANE_WIDTH_FALLBACK
    current_y = self.yRel

    if not self.is_out_of_lane:
      if current_y > (left_bound + LANE_HYSTERESIS_MARGIN) or current_y < (right_bound - LANE_HYSTERESIS_MARGIN):
        self.is_out_of_lane = True
    else:
      if right_bound <= current_y <= left_bound:
        self.is_out_of_lane = False

    return not self.is_out_of_lane

  def _calculate_fuzzy_score(self, offset_vision_dist: float, vision_y: float, vision_v: float, v_ego: float) -> float:
    """
    【階段二：嚴格三維物理模糊打分】確保雷達與視覺在三維空間重合才准許打分。
    """
    err_d = abs(self.dRel - offset_vision_dist)
    err_y = abs(self.yRel - vision_y)
    err_v = abs((self.vRel + v_ego) - vision_v)

    score_d = float(np.interp(err_d, FUZZY_BOUNDS, [1.0, 0.0]))
    score_y = float(np.interp(err_y, FUZZY_BOUNDS, [1.0, 0.0]))
    score_v = float(np.interp(err_v, FUZZY_BOUNDS, [1.0, 0.0]))

    return score_d * score_y * score_v

  def _calculate_threat_multipliers(self, v_ego: float) -> float:
    """
    【階段三：威脅乘數融合】根據前車急煞或切入動態，疊加威脅倍率。
    """
    brake_mult = float(np.interp(self.aLeadK, BRAKE_THRES_RANGE, MULT_RANGE))
    cutin_mult = 1.0
    
    if self.dRel < CUTIN_DIST_LIMIT and abs(self.yRel) > 1.0:
      v_limit = max(1.0, DYNAMIC_SPEED_PCT * v_ego)
      cutin_mult = float(np.interp(self.vRel, [-v_limit, v_limit], MULT_RANGE))

    final_alpha = ALPHA_BASE * brake_mult * cutin_mult
    return min(1.0, final_alpha)

  def _apply_slow_protection(self, v_ego: float, cam_prob: float, current_ema: float) -> float:
    """
    【階段四：慢速與靜止防護網】低速靜止目標需嚴格審核相機機率，防幽靈急煞。
    """
    abs_v_lead = abs(self.vRel + v_ego)
    dynamic_v_limit = max(1.0, DYNAMIC_SPEED_PCT * v_ego)

    if abs_v_lead < dynamic_v_limit:
      dynamic_cam_prob_thres = float(np.interp(v_ego, CAM_PROB_SPEED_RANGE, CAM_PROB_RANGE))
      if cam_prob < dynamic_cam_prob_thres:
        return min(current_ema, STATIC_EMA_CAP)

    return current_ema

  def process_track_logic(self, lead_idx: int, lead_msg: capnp._DynamicStructReader, v_ego: float, lead_prob: float):
    """
    【主控管線：包含剪枝、緩衝續命與打分更新】
    新增 lead_prob 參數，與原廠非對稱濾波機率同步。
    """
    offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
    vision_y = -lead_msg.y[0]
    vision_v = lead_msg.v[0]

    # 🌪️ 漏斗第一關：初步判斷是否為無效雜訊或嚴重出界
    is_invalid = not self.measured or abs(self.yRel - vision_y) > (LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN)
    
    fuzzy_score = 0.0
    if not is_invalid:
      # 執行精細幾何邊界審查與三維物理打分
      is_valid_spatial = self._check_spatial_boundaries(vision_y)
      fuzzy_score = self._calculate_fuzzy_score(offset_vision_dist, vision_y, vision_v, v_ego)
      is_invalid = not is_valid_spatial or fuzzy_score == 0.0

    # ⚡ 續命與衰減處理 (Release / Decay)
    if is_invalid:
      if self.holdover_frames[lead_idx] > 0:
        # 在續命期間，凍結 EMA 衰減，維持高分狀態度過硬體閃爍
        self.holdover_frames[lead_idx] -= 1
        return
      else:
        # 徹底失效，快速衰減
        self.ema_confidence[lead_idx] = ALPHA_DOWN * 0.0 + (1 - ALPHA_DOWN) * self.ema_confidence[lead_idx]
        return

    # 若順利通過驗證，補滿續命計時器
    self.holdover_frames[lead_idx] = RELEASE_FRAMES

    # 階段三與階段四：精算學習率與狀態更新
    final_alpha_up = self._calculate_threat_multipliers(v_ego)
    target_ema = fuzzy_score
    alpha = final_alpha_up if fuzzy_score > 0.5 else ALPHA_DOWN
    
    new_ema = alpha * target_ema + (1 - alpha) * self.ema_confidence[lead_idx]
    new_ema = self._apply_slow_protection(v_ego, lead_prob, new_ema)

    self.ema_confidence[lead_idx] = new_ema


def get_lead_ext(
  v_ego: float,
  ready: bool,
  tracks: dict[int, TrackSP],
  lead_msg: capnp._DynamicStructReader,
  model_v_ego: float,
  lead_prob: float,
  CP: structs.CarParams,
  CP_SP: structs.CarParamsSP,
  low_speed_override: bool = True,
) -> dict[str, Any]:
  """
  擴充的前車評估函數：導入多階段過濾、反向動態門檻、硬體斷流記憶修補。
  完美對齊新版原廠簽名，接收並使用 lead_prob 作為判斷基準。
  """
  lead_idx = 0 if low_speed_override else 1
  max_ema_confidence = 0.0

  if ready:
    for track in tracks.values():
      track.process_track_logic(lead_idx, lead_msg, v_ego, lead_prob)

  # 🛡️ 提取乾淨名單：排除徹底出界與分數歸零的目標
  valid_tracks = {k: v for k, v in tracks.items() if not v.is_out_of_lane and v.ema_confidence[lead_idx] > 0.0}

  if len(valid_tracks) > 0:
    max_ema_confidence = max(track.ema_confidence[lead_idx] for track in valid_tracks.values())

  # 階段五：將全場最高 EMA 無縫映射為視覺門檻
  current_prob_thres = float(np.interp(max_ema_confidence, EMA_VAL_RANGE, PROB_THRES_RANGE))

  # 判斷融合
  selected_track = None
  if len(valid_tracks) > 0 and ready and lead_prob > current_prob_thres:
    selected_track = match_vision_to_track(v_ego, lead_msg, valid_tracks)

  # ==============================================================================
  # 🛡️ 狀態機記憶：雷達硬體斷流修補 (Hardware Dropout Patch)
  # 若雷達突然失效 1~3 幀，使用 Python 記憶體中保留的上一幀物件強行頂替，維持平滑動力。
  # ==============================================================================
  cache = _LEAD_STATE_CACHE[lead_idx]
  if selected_track is not None:
    cache['track'] = selected_track
    cache['absent'] = 0
  elif cache['track'] is not None:
    cache['absent'] += 1
    if cache['absent'] <= SELECT_HOLDOVER_FRAMES:
      selected_track = cache['track']
    else:
      cache['track'] = None
      cache['absent'] = 0

  # 封裝輸出數據
  lead_dict = {'status': False}
  if selected_track is not None:
    # 傳入最新的平滑化機率給原廠數據包
    lead_dict = selected_track.get_RadarState(lead_prob)
    lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)

    # ⚖️ 視覺加速度雙重驗證阻尼：防止短暫雜訊引發幽靈急煞
    model_tau = get_model_lead_tau(lead_msg, lead_prob)
    if model_tau is not None:
      lead_dict['aLeadTau'] = model_tau

    if current_prob_thres < 0.5 and (0.5 >= lead_prob > current_prob_thres):
      cloudlog.debug(
        f"[RadarDSP_EarlyLock] 提早鎖定/續命成功！目標 {lead_idx} | "
        f"相機機率: {lead_prob:.2f} (動態門檻: {current_prob_thres:.2f})"
      )

  elif (selected_track is None) and ready and (lead_prob > current_prob_thres):
    # 對齊最新版，補齊缺失的 lead_prob 參數
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  # 🛡️ 原廠底線救援 (極低速專屬防護)
  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


# ==============================================================================
# 【雙重 Monkey Patching】動態替換原廠類別與判斷函數
# ==============================================================================
radard.Track = TrackSP
radard.get_lead = get_lead_ext


class RadarDSP(RadarD):
  """
  繼承自 RadarD。無需改寫主迴圈，透過 Monkey Patching 即可自動套用擴充邏輯。
  """
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParams, delay: float = 0.0):
    super().__init__(CP, CP_SP, delay)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    super().update(sm, rr)
