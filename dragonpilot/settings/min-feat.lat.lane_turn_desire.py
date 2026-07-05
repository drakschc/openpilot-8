from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Lateral",
    "key": "dp_lane_turn_desire",
    "type": "toggle_item",
    "title": lambda: tr("Use Lane Turn Desires (LTD)"),
    "description": lambda: tr("If you're driving below the set speed and have your blinker on, the car will plan a turn in that direction at the nearest drivable path. This prevents situations (like at red lights) where the car might plan the wrong turn direction. From SunnyPilot."),
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
    "on_change": [{
      "target": "dp_lane_turn_value",
      "action": "set_visible",
      "condition": "value == '1'"  # 修改此處
    }]
  },
  {
    "section": "Lateral",
    "key": "dp_lane_turn_value",
    "type": "spin_button_item",
    "title": lambda: tr("Adjust Lane Turn Speed"),
    "description": lambda: tr("Set the maximum speed for lane turn desires (Default is 20 km/h)."),
    "flags": "PERSISTENT",
    "param_type": "INT",
    "default": "20",
    "min_val": 10,
    "max_val": 35,
    "step": 5,
    "suffix": lambda: tr(" km/h"),
    "initially_visible_by": {
      "param": "dp_lane_turn_desire",
      "condition": "value == '1'", # 修改此處
      "default": 0
    }
  },
]
