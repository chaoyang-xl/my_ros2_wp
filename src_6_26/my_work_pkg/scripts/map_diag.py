#!/usr/bin/env python3
"""一次性诊断：打印 /map 的数值分布和吸附点周围区域"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from collections import Counter

rclpy.init()
node = Node('map_diag', allow_undeclared_parameters=True)
msg = None

def cb(m):
    global msg
    msg = m

sub = node.create_subscription(OccupancyGrid, '/map', cb, 1)
while msg is None:
    rclpy.spin_once(node, timeout_sec=2.0)

data = msg.data
w = msg.info.width
h = msg.info.height
res = msg.info.resolution
ox = msg.info.origin.position.x
oy = msg.info.origin.position.y

c = Counter(data)
total = len(data)
print(f'Map: {w}x{h}, res={res:.3f}m, origin=({ox:.2f},{oy:.2f})')
print(f'Total cells: {total}')
print(f'Value distribution:')
for k in sorted(c.keys()):
    pct = 100 * c[k] / total
    print(f'  {k:4d}: {c[k]:6d} cells ({pct:.1f}%)')

# occupied cells (>= 50)
occ = [(i % w, i // w, data[i]) for i in range(total) if data[i] >= 50]
print(f'\nCells with value >= 50: {len(occ)}')
if occ:
    print('First 20:')
    for gx, gy, v in occ[:20]:
        wx = ox + (gx + 0.5) * res
        wy = oy + (gy + 0.5) * res
        print(f'  grid=({gx},{gy}) world=({wx:.2f},{wy:.2f}) value={v}')

# 吸附点 (0.668, 3.168) 周围 15x15 窗口
px, py = 0.668, 3.168
pgx = int((px - ox) / res)
pgy = int((py - oy) / res)
print(f'\n=== 吸附点 world=({px:.2f},{py:.2f}) grid=({pgx},{pgy}) 周围 15x15 ===')
r = 7
for dy in range(-r, r + 1):
    row = []
    for dx in range(-r, r + 1):
        ix, iy = pgx + dx, pgy + dy
        idx = iy * w + ix
        if 0 <= idx < total:
            row.append(f'{data[idx]:4d}')
        else:
            row.append('   ?')
    print(' '.join(row))

node.destroy_node()
rclpy.shutdown()
