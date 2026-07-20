#!/usr/bin/env python3
"""同步诊断：发布一个种子后，立即检查地图 + 投影结果"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from collections import Counter
import json, math, time

rclpy.init()
node = Node('sync_diag')
map_msg = [None]

def map_cb(m):
    map_msg[0] = m

node.create_subscription(OccupancyGrid, '/map', map_cb, 1)

# 等地图
while map_msg[0] is None:
    rclpy.spin_once(node, timeout_sec=1.0)

m = map_msg[0]
w, h, res = m.info.width, m.info.height, m.info.resolution
ox, oy = m.info.origin.position.x, m.info.origin.position.y
data = m.data

print(f'Map loaded: {w}x{h}, res={res:.3f}m')

# 测试点：用用户实际看到的吸附点坐标
test_points = [
    (0.668, 3.168, "吸附点"),
    (-2.0, 1.0, "机器人出生点附近"),
    (0.0, 0.0, "原点"),
]

for tx, ty, label in test_points:
    gx = int((tx - ox) / res)
    gy = int((ty - oy) / res)
    idx = gy * w + gx
    val = data[idx] if 0 <= idx < len(data) else None

    # 在该点周围 search_radius=0.2m (5 cells) 搜索
    r = int(0.2 / res) + 1
    occupied_nearby = []
    for dy in range(-r, r+1):
        for dx in range(-r, r+1):
            ix, iy = gx + dx, gy + dy
            if 0 <= ix < w and 0 <= iy < h:
                v = data[iy * w + ix]
                if v >= 50:
                    wx = ox + (ix + 0.5) * res
                    wy = oy + (iy + 0.5) * res
                    occupied_nearby.append((ix, iy, v, wx, wy))

    occ_80 = [(ix,iy,v,wx,wy) for ix,iy,v,wx,wy in occupied_nearby if v >= 80]
    occ_65 = [(ix,iy,v,wx,wy) for ix,iy,v,wx,wy in occupied_nearby if v >= 65]

    print(f'\n{label}: world=({tx:.2f},{ty:.2f}) grid=({gx},{gy}) value={val}')
    print(f'  半径0.2m内 v>=50: {len(occupied_nearby)}个, v>=65: {len(occ_65)}个, v>=80: {len(occ_80)}个')
    if occ_65:
        for ix, iy, v, wx, wy in occ_65[:5]:
            print(f'    ({ix},{iy}) v={v} world=({wx:.2f},{wy:.2f})')

# 最终统计
print(f'\n=== 全图统计 ===')
c = Counter(data)
non_minus_one = {k:v for k,v in c.items() if k >= 0}
occupied_like = sum(v for k,v in c.items() if 50 <= k <= 100)
print(f'  确定障碍 (>=100): {c.get(100, 0)} cells')
print(f'  高概率障碍 (80-99): {sum(v for k,v in c.items() if 80 <= k <= 99)} cells')
print(f'  中概率障碍 (65-79): {sum(v for k,v in c.items() if 65 <= k <= 79)} cells')
print(f'  低概率障碍 (50-64): {sum(v for k,v in c.items() if 50 <= k <= 64)} cells')
print(f'  空闲 (1-49): {sum(v for k,v in c.items() if 1 <= k <= 49)} cells')
print(f'  确定空闲 (0): {c.get(0, 0)} cells')
print(f'  未知 (-1): {c.get(-1, 0)} cells')

node.destroy_node()
rclpy.shutdown()
