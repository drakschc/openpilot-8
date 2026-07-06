#!/usr/bin/env python3
import os
import sys
import time
import math
import json
import re
import requests
import xml.etree.ElementTree as ET
import urllib3

# 忽略因為 verify=False 產生的 SSL 警告，適合 C3X 嵌入式環境
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# 依據你的檔案位置，設定正確的 openpilot 路徑以載入 messaging
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../../../"))
import cereal.messaging as messaging

# ==========================================
# 工具函數：開機自動下載 TDX 線形圖資 (針對 Comma 3X 網路延遲最佳化)
# ==========================================
def download_freeway_shapes_guest(filepath):
    print(f"[系統] 偵測到開機啟動，準備透過訪客額度下載最新 TDX 圖資...")
    url = "https://tdx.transportdata.tw/api/basic/v2/Road/Traffic/SectionShape/Freeway?$format=JSON"
    headers = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) openpilot/tdxd'}
    
    for attempt in range(5):
        try:
            print(f"[系統] 第 {attempt+1}/5 次嘗試連線 TDX...")
            response = requests.get(url, headers=headers, timeout=15, verify=False)
            response.raise_for_status()
            data = response.json()
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
                
            print(f"[系統] ✅ 圖資下載成功！已更新 {filepath} (共 {len(data)} 筆路段)")
            return 
            
        except Exception as e:
            print(f"[警告] ⚠️ 嘗試失敗 ({e})。可能是 C3X 網路尚未就緒，20秒後重試...")
            time.sleep(20)
            
    print(f"[警告] ❌ 網路連線逾時，放棄下載，將沿用本地舊有圖資檔案。")


# ==========================================
# 工具函數：WKT 解析、距離與方位角運算
# ==========================================
def _parse_wkt_linestring(wkt_str):
    inner = re.search(r'LINESTRING\s*\((.+)\)', wkt_str, re.IGNORECASE)
    if not inner: return []
    pairs = inner.group(1).split(',')
    return [tuple(map(float, p.strip().split())) for p in pairs]

def _segment_bearing(lon1, lat1, lon2, lat2):
    dy = lat2 - lat1
    dx = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return math.degrees(math.atan2(dx, dy)) % 360

def _point_to_segment_dist_sq_and_bearing(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0:
        return (px - ax) ** 2 + (py - ay) ** 2, 0.0
    
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    dist_sq = (px - proj_x) ** 2 + (py - proj_y) ** 2
    bearing = _segment_bearing(ax, ay, bx, by)
    return dist_sq, bearing

def _angle_diff(a, b):
    diff = abs(a - b) % 360
    return diff if diff <= 180 else 360 - diff

def _get_distance_meters(lon1, lat1, lon2, lat2):
    dx = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2)) * 111320.0
    dy = (lat2 - lat1) * 111320.0
    return math.sqrt(dx*dx + dy*dy)

def bearing_to_direction(bearing_deg):
    b = bearing_deg % 360
    if b < 45 or b >= 315: return '北向'
    elif b < 135: return '東向'
    elif b < 225: return '南向'
    else: return '西向'

# ==========================================
# 1. 核心圖資配對引擎 (向量內積防護版)
# ==========================================
class LocalMapMatcher:
    def __init__(self, filepath):
        self.sections = {}          
        self.spatial_grid = {}      
        self.grid_size = 0.01       
        self.topology_next = {}     
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
                raw_shapes = data.get("SectionShapes", data) if isinstance(data, dict) else data
                print("[圖資] 正在建立空間網格與幾何拓樸...")
                
                for item in raw_shapes:
                    geom_wkt = item.get('Geometry')
                    sec_id = item.get('SectionID')
                    if not geom_wkt or not sec_id: continue
                        
                    coords = _parse_wkt_linestring(geom_wkt)
                    if len(coords) < 2: continue
                        
                    segments = []
                    total_length = 0.0
                    for i in range(len(coords) - 1):
                        p1, p2 = coords[i], coords[i+1]
                        dist = _get_distance_meters(p1[0], p1[1], p2[0], p2[1])
                        total_length += dist
                        segments.append({'p1': p1, 'p2': p2, 'dist': dist})
                        
                    self.sections[sec_id] = {
                        'id': sec_id,
                        'coords': coords,
                        'segments': segments,
                        'total_length': total_length,
                        'start_pt': coords[0],
                        'end_pt': coords[-1]
                    }
                    
                    min_lon = min(p[0] for p in coords)
                    max_lon = max(p[0] for p in coords)
                    min_lat = min(p[1] for p in coords)
                    max_lat = max(p[1] for p in coords)
                    
                    min_gx, max_gx = int(min_lon / self.grid_size), int(max_lon / self.grid_size)
                    min_gy, max_gy = int(min_lat / self.grid_size), int(max_lat / self.grid_size)
                    
                    for gx in range(min_gx, max_gx + 1):
                        for gy in range(min_gy, max_gy + 1):
                            grid_key = (gy, gx)
                            if grid_key not in self.spatial_grid:
                                self.spatial_grid[grid_key] = []
                            self.spatial_grid[grid_key].append(sec_id)
                
                # 建立拓樸
                for sec_a, data_a in self.sections.items():
                    end_pt = data_a['end_pt']
                    end_bearing = _segment_bearing(data_a['segments'][-1]['p1'][0], data_a['segments'][-1]['p1'][1], 
                                                   data_a['segments'][-1]['p2'][0], data_a['segments'][-1]['p2'][1])
                    best_next = None
                    min_gap = float('inf')
                    
                    for sec_b, data_b in self.sections.items():
                        if sec_a == sec_b: continue
                        start_pt = data_b['start_pt']
                        gap = _get_distance_meters(end_pt[0], end_pt[1], start_pt[0], start_pt[1])
                        
                        if gap < 50.0 and gap < min_gap:
                            start_bearing = _segment_bearing(data_b['segments'][0]['p1'][0], data_b['segments'][0]['p1'][1], 
                                                             data_b['segments'][0]['p2'][0], data_b['segments'][0]['p2'][1])
                            if _angle_diff(end_bearing, start_bearing) < 45: 
                                best_next = sec_b
                                min_gap = gap
                                
                    if best_next:
                        self.topology_next[sec_a] = best_next

                print(f"[圖資] ✅ 載入 {len(self.sections)} 筆，建立 {len(self.topology_next)} 筆道路連結！")
        except Exception as e:
            print(f"[圖資] 讀取 freeway_shapes.json 失敗: {e}")

    def _get_candidates_from_grid(self, lat, lon, radius_km=3):
        candidates = set()
        gy_center, gx_center = int(lat / self.grid_size), int(lon / self.grid_size)
        span = math.ceil((radius_km / 111.0) / self.grid_size) 
        
        for gy in range(gy_center - span, gy_center + span + 1):
            for gx in range(gx_center - span, gx_center + span + 1):
                candidates.update(self.spatial_grid.get((gy, gx), []))
        return candidates

    def find_current_section(self, lat, lon, threshold_meters=300, bearing_deg=None, direction_text=None):
        candidates = self._get_candidates_from_grid(lat, lon, radius_km=2)
        if not candidates:
            return None
        
        best_match = None
        min_dist = float('inf')

        # 針對 XML 事件：建立文字方向向量
        evt_vec = None
        if direction_text:
            if '北' in direction_text: evt_vec = (0, 1)
            elif '南' in direction_text: evt_vec = (0, -1)
            elif '東' in direction_text: evt_vec = (1, 0)
            elif '西' in direction_text: evt_vec = (-1, 0)
        
        for sec_id in candidates:
            sec_data = self.sections[sec_id]
            
            # [核心修復] 利用向量內積，徹底解決錯綁對向車道的問題
            if evt_vec:
                start_pt = sec_data['start_pt']
                end_pt = sec_data['end_pt']
                sec_vec_x = end_pt[0] - start_pt[0]
                sec_vec_y = end_pt[1] - start_pt[1]
                dot_product = sec_vec_x * evt_vec[0] + sec_vec_y * evt_vec[1]
                if dot_product < 0: # 內積為負，代表這是反向車道，直接剔除！
                    continue
            
            for seg in sec_data['segments']:
                dist_sq, local_bearing = _point_to_segment_dist_sq_and_bearing(
                    lon, lat, seg['p1'][0], seg['p1'][1], seg['p2'][0], seg['p2'][1])
                
                dist_meters = math.sqrt(dist_sq) * 111320.0
                
                if dist_meters < min_dist:
                    # [車輛防護] 嚴格維持 60 度，避免十字交流道上下層誤判
                    if bearing_deg is not None and _angle_diff(bearing_deg, local_bearing) > 60:
                        continue
                    
                    min_dist = dist_meters
                    best_match = sec_id
                    
        if best_match and min_dist < threshold_meters:
            return best_match
        return None

    def find_ahead_section(self, current_sec_id, target_distance_m=3000):
        if current_sec_id not in self.sections: return None
            
        accumulated_dist = 0.0
        current = current_sec_id
        
        for _ in range(20): 
            sec_len = self.sections[current]['total_length']
            if accumulated_dist + sec_len >= target_distance_m:
                return current
                
            accumulated_dist += sec_len
            if current in self.topology_next:
                current = self.topology_next[current]
            else:
                return current
                
        return current


# ==========================================
# 2. 高公局 XML 雙核心抓取器
# ==========================================
class FreewayDataClient:
    def __init__(self):
        self.events_url = "https://tisvcloud.freeway.gov.tw/history/motc20/LiveEvents.xml"
        self.traffic_url = "https://tisvcloud.freeway.gov.tw/history/motc20/LiveTraffic.xml"
        self.headers = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'}
        self.cached_events = []
        self.cached_speeds = {}

    def update_data(self, matcher):
        print("[網路] 🔄 背景更新高公局 LiveEvents & LiveTraffic...")
        try:
            res_evt = requests.get(self.events_url, headers=self.headers, timeout=5, verify=False)
            res_evt.encoding = 'utf-8'
            xml_evt = re.sub(r'\sxmlns="[^"]+"', '', res_evt.text, count=1)
            root_evt = ET.fromstring(xml_evt)

            new_events = []
            for event in root_evt.findall('.//LiveEvent'):
                event_type = (event.findtext('EventType') or '0').strip()
                desc = event.findtext('Description', '未知事件')
                positions = event.findtext('Positions')
                direction = (event.findtext('.//Direction') or '').strip()

                if positions and positions.startswith('POINT('):
                    coords_str = positions.replace('POINT(', '').replace(')', '').strip().split()
                    if len(coords_str) == 2:
                        evt_lon, evt_lat = float(coords_str[0]), float(coords_str[1])
                        # 帶入 direction_text，讓內積演算法判斷對錯車道
                        sec_id = matcher.find_current_section(evt_lat, evt_lon, threshold_meters=1000, direction_text=direction)
                        
                        # 【修改】加入 lat, lon, direction，且就算找不到完美的 SectionID 也存起來供雷達掃描用
                        new_events.append({
                            'EventType': event_type,
                            'Description': desc,
                            'SectionID': str(sec_id).strip() if sec_id else "",
                            'lat': evt_lat,
                            'lon': evt_lon,
                            'direction': direction
                        })
            self.cached_events = new_events

            res_spd = requests.get(self.traffic_url, headers=self.headers, timeout=5, verify=False)
            res_spd.encoding = 'utf-8'
            xml_spd = re.sub(r' xmlns="[^"]+"', '', res_spd.text, count=1)
            xml_spd = re.sub(r' xmlns:xsi="[^"]+"', '', xml_spd, count=1)
            xml_spd = re.sub(r' xsi:[^=]+="[^"]+"', '', xml_spd)
            root_spd = ET.fromstring(xml_spd)

            new_speeds = {}
            for traffic in root_spd.findall('.//LiveTraffic'):
                link_id = (traffic.findtext('SectionID') or '').strip()
                speed_str = (traffic.findtext('TravelSpeed') or '').strip()
                if link_id and speed_str:
                    spd = int(speed_str)
                    if spd > 0:
                        new_speeds[link_id] = spd
            self.cached_speeds = new_speeds
            print(f"[網路] ✅ 快取 {len(self.cached_events)} 筆事件, {len(self.cached_speeds)} 筆有效車速。")
        except Exception as e:
            print(f"[網路] 更新失敗: {e}")

# ==========================================
# 3. 主循環
# ==========================================
def main():
    print("🚀 啟動 tdxd 路況發布系統 (空間雷達掃描版)...")
    pm = messaging.PubMaster(['tdx'])

    BASE_DIR = os.path.dirname(os.path.realpath(__file__))
    JSON_PATH = os.path.join(BASE_DIR, "freeway_shapes.json")
    
    download_freeway_shapes_guest(JSON_PATH)

    matcher = LocalMapMatcher(JSON_PATH)
    client = FreewayDataClient()
    sm = messaging.SubMaster(['liveGPS', 'gpsLocationExternal'],
                              ignore_alive=['liveGPS', 'gpsLocationExternal'])

    MAX_HORIZONTAL_ACCURACY = 50.0
    last_api_call = 0
    UPDATE_INTERVAL = 60 

    last_calc_time = 0           
    CALC_INTERVAL = 1.0          
    
    ahead_section_history = []   
    SMOOTH_COUNT = 3             
    stable_ahead_section = None  
    stable_current_section = None

    CURRENT_SECTION_MISS_LIMIT = 2   
    current_section_miss_count = 0
    
    # ==========================================
    # 測試點設定 (可自行開關) 切換回真實的車輛 GPS，只要把 TEST_MODE = True 改成 TEST_MODE = False
    # ==========================================
    TEST_MODE = False
    TEST_LAT = 24.860332
    TEST_LON = 121.218465
    TEST_BEARING = 65.0   
    # ==========================================

    while True:
        if TEST_MODE:
            time.sleep(0.1) 
        else:
            sm.update()
            
        current_time = time.time()

        if current_time - last_api_call >= UPDATE_INTERVAL:
            client.update_data(matcher)
            last_api_call = current_time

        gps_source = None
        is_gps_ready = False

        if TEST_MODE:
            is_gps_ready = True
            lat, lon, bearing = TEST_LAT, TEST_LON, TEST_BEARING
            gps_source = 'TEST_MODE'
        else:
            if sm.updated.get('liveGPS', False):
                live_gps = sm['liveGPS']
                if live_gps.gpsOK and (live_gps.horizontalAccuracy <= 0 or
                                        live_gps.horizontalAccuracy <= MAX_HORIZONTAL_ACCURACY):
                    lat = live_gps.latitude
                    lon = live_gps.longitude
                    bearing = live_gps.bearingDeg
                    is_gps_ready = True
                    gps_source = 'liveGPS'

            if not is_gps_ready and sm.updated.get('gpsLocationExternal', False):
                raw_gps = sm['gpsLocationExternal']
                acc = getattr(raw_gps, 'horizontalAccuracy', 0.0)
                if acc <= 0 or acc <= MAX_HORIZONTAL_ACCURACY:
                    lat = raw_gps.latitude
                    lon = raw_gps.longitude
                    bearing = raw_gps.bearingDeg
                    is_gps_ready = True
                    gps_source = 'gpsLocationExternal(備援)'

        if is_gps_ready and (current_time - last_calc_time >= CALC_INTERVAL):
            last_calc_time = current_time
            my_direction = bearing_to_direction(bearing)

            raw_current_section = matcher.find_current_section(lat, lon, threshold_meters=300, bearing_deg=bearing)

            if raw_current_section is not None:
                stable_current_section = raw_current_section
                current_section_miss_count = 0
            else:
                current_section_miss_count += 1
                if current_section_miss_count >= CURRENT_SECTION_MISS_LIMIT:
                    stable_current_section = None

            raw_ahead_section = None
            if stable_current_section is not None:
                raw_ahead_section = matcher.find_ahead_section(stable_current_section, target_distance_m=3000)
                
                ahead_section_history.append(raw_ahead_section)
                if len(ahead_section_history) > SMOOTH_COUNT:
                    ahead_section_history.pop(0)

                if len(ahead_section_history) == SMOOTH_COUNT and len(set(ahead_section_history)) == 1:
                    stable_ahead_section = ahead_section_history[0]
            else:
                ahead_section_history.clear()
                stable_ahead_section = None

            msg = messaging.new_message('tdx')
            traffic = msg.tdx.init('trafficStatus')
            
            ahead_speed = client.cached_speeds.get(str(stable_ahead_section).strip(), -1) if stable_ahead_section else -1
            display_speed = ahead_speed

            if display_speed > 0:
                traffic.sectionId = str(stable_ahead_section)
                traffic.speed = int(display_speed)
                if display_speed >= 80:
                    traffic.status = "GREEN"
                elif display_speed >= 40:
                    traffic.status = "YELLOW"
                else:
                    traffic.status = "RED"
            else:
                traffic.sectionId = ""
                traffic.speed = -1
                traffic.status = "GREEN"

            current_event = None
            ahead_event = None

            # 【全新邏輯】結合 SectionID 與「前方扇形半徑空間」比對，並嚴格檢查南北向
            for evt in client.cached_events:
                evt_sid = evt.get('SectionID', '')
                evt_desc = evt['Description']
                evt_type = evt.get('EventType', '0')
                evt_lat = evt.get('lat')
                evt_lon = evt.get('lon')
                evt_dir = evt.get('direction', '')

                formatted_evt = f"{evt_type}|{evt_desc}"

                # 1. 目前路段比對 (維持原來的精準 SectionID 綁定)
                if stable_current_section and evt_sid == str(stable_current_section).strip():
                    current_event = (current_event + "/" + formatted_evt) if current_event else formatted_evt
                    continue  # 已經發生在目前路段，跳過前方計算

                # 2. 前方路段比對：方法A (拓樸推導) 或 方法B (空間雷達)
                is_ahead = False
                
                # 方法A：原本的拓樸推導 (對付主線事件最精準)
                if stable_ahead_section and evt_sid == str(stable_ahead_section).strip():
                    is_ahead = True
                
                # 方法B：空間半徑 + 車頭方位角 + 文字南北向防呆 (專抓匝道與未綁定事件)
                elif evt_lat and evt_lon:
                    # 計算絕對距離
                    dist_m = _get_distance_meters(lon, lat, evt_lon, evt_lat)
                    if dist_m <= 3000:  # 事件在 3 公里範圍內
                        # 計算從車頭指向事件的方位角
                        evt_bearing = _segment_bearing(lon, lat, evt_lon, evt_lat)
                        # 如果夾角小於 60 度，代表事件在我們正前方的扇形視野內
                        if _angle_diff(bearing, evt_bearing) <= 60:
                            # 雙重防呆：用當前行駛方向 (例如 '南向')的第一個字去比對 XML 的方向文字 (例如 '南')
                            if evt_dir and (my_direction[0] in evt_dir):
                                is_ahead = True
                            # 如果高公局 XML 剛好沒寫方向，但角度這麼完美落在正前方，也視為前方事件
                            elif not evt_dir:
                                is_ahead = True

                if is_ahead:
                    ahead_event = (ahead_event + "/" + formatted_evt) if ahead_event else formatted_evt

            cycle_state = int(current_time // 10) % 2
            event = msg.tdx.init('roadEvent')
            event.isActive = False
            event.description = ""
            event.sectionId = ""

            if cycle_state == 0:
                if current_event:
                    event.description = f"目前:{current_event}"
                    event.isActive = True
                elif ahead_event:
                    event.description = f"前方:{ahead_event}"
                    event.isActive = True
            else:
                if ahead_event:
                    event.description = f"前方:{ahead_event}"
                    event.isActive = True
                elif current_event:
                    event.description = f"目前:{current_event}"
                    event.isActive = True

            pm.send('tdx', msg) 

            print(f"GPS: {gps_source} | 航向: {bearing:.1f}°({my_direction})")
            print(f"路段判定 -> 目前: {stable_current_section or '無'} | 前方: {stable_ahead_section or '無'}")
            
            if traffic.speed > 0:
                print(f" => 前方車速顯示: {traffic.speed} km/h")
            if event.isActive:
                print(f" => 事件警告: {event.description}")
            print("-" * 40)

if __name__ == "__main__":
    main()
