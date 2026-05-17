"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
# Version = Simplified (60 On / 70 Off) + Cruise Speed Limit

from cereal import messaging
from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.dec.constants import WMACConstants

class DynamicExperimentalController:
  def __init__(self, CP: structs.CarParams, mpc, params=None):
    self._CP = CP
    self._mpc = mpc
    self._params = params or Params()
    self._enabled: bool = self._params.get_bool("DynamicExperimentalControl")
    self._active: bool = False
    self._frame: int = 0
    
    # 預設模式為 acc (關閉狀態)
    self._mode = 'acc'

  def _read_params(self) -> None:
    if self._frame % int(1. / DT_MDL) == 0:
      self._enabled = self._params.get_bool("DynamicExperimentalControl")

  def mode(self) -> str:
    return self._mode

  def enabled(self) -> bool:
    return self._enabled

  def active(self) -> bool:
    return self._active

  def set_mpc_fcw_crash_cnt(self) -> None:
    # 邏輯已簡化，不再需要此功能
    pass

  def update(self, sm: messaging.SubMaster) -> None:
    self._read_params()

    # 取得當前車速並轉換為 km/h
    v_ego_kph = sm['carState'].vEgo * 3.6
    
    # 取得儀表板巡航設定車速並轉換為 km/h (cruiseState.speed 原始單位為 m/s)
    v_cruise_kph = sm['carState'].cruiseState.speed * 3.6

    # 核心簡化邏輯：加入儀表板設定車速限制
    if v_cruise_kph > WMACConstants.MAX_CRUISE_SET_SPEED:
      # 條件 1：當儀表板設定車速大於 70 時，強制關閉實驗模式 (acc)
      self._mode = 'acc'
    else:
      # 條件 2：當儀表板設定車速在 70(含) 以下時，依據實際車速切換
      if v_ego_kph < WMACConstants.ENABLE_SPEED:
        self._mode = 'blended'
      elif v_ego_kph > WMACConstants.DISABLE_SPEED:
        self._mode = 'acc'

    self._active = sm['selfdriveState'].experimentalMode and self._enabled
    self._frame += 1
