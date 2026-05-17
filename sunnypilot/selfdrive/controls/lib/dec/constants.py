class WMACConstants:
  # 簡化版速度門檻 (km/h)
  ENABLE_SPEED = 60.0   # 低於 60 開啟 (blended)
  DISABLE_SPEED = 70.0  # 高於 70 關閉 (acc)
  
  # 追加條件：儀表板巡航設定車速門檻
  MAX_CRUISE_SET_SPEED = 70.0 # 設定高於 70 強制關閉 (acc)
