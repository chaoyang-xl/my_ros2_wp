import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
# g值：node.g表示从起点到该节点的实际代价 考虑转向代价等。

# h值：node.h是通过heuristic(node, goal)计算的，即欧几里得距离。

# f值：node.f()返回self.g + self.h，即f = g + h。

# ========================
# 地图生成
# ========================

def create_map():
    grid = np.zeros((50, 50))
    
    # 加障碍
    grid[20:30, 20:30] = 1
    grid[35:40, 5:20] = 1
    grid[10:15, 35:45] = 1
    grid[10:25, 10:25] = 1
    
    return grid


# ========================
# Node定义
# ========================

class Node:
    def __init__(self, x, y, theta ,direction=1): # direction: 1 for forward, -1 for backward
        self.x = x
        self.y = y
        self.theta = theta
        self.direction = direction
        self.g = 0
        self.h = 0
        self.parent = None

    def f(self):
        return self.g + self.h

    def __lt__(self, other):
        return self.f() < other.f()

# ========================
# 2D BFS 启发函数预计算
# ========================
def calc_2d_heuristic(grid, goal):
    h_map = np.full(grid.shape, np.inf)
    goal_x, goal_y = int(goal.x), int(goal.y)

    # 修复边界检查：x 对应 shape[1]，y 对应 shape[0]
    if goal_x < 0 or goal_y < 0 or goal_x >= grid.shape[1] or goal_y >= grid.shape[0] or grid[goal_y, goal_x] == 1:
        return h_map

    queue = deque([(goal_x, goal_y)])
    h_map[goal_y, goal_x] = 0  # 修复为 [y, x]

    motions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    costs = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]

    while queue:
        cx, cy = queue.popleft()
        for i in range(8):
            nx, ny = cx + motions[i][0], cy + motions[i][1]
            
            # 修复边界检查
            if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
                if grid[ny, nx] == 0:  # 修复为 [y, x]
                    new_cost = h_map[cy, cx] + costs[i] # 修复为 [y, x]
                    if new_cost < h_map[ny, nx]:        # 修复为 [y, x]
                        h_map[ny, nx] = new_cost        # 修复为 [y, x]
                        queue.append((nx, ny))
    return h_map

# ========================
# 启发函数
# ========================

def heuristic(node, goal):
    dist=math.hypot(node.x - goal.x, node.y - goal.y)
    dtheta=abs(node.theta - goal.theta)
    dtheta = min(dtheta, 2*math.pi - dtheta)
    return dist + dtheta*0.5

# ========================
# 碰撞检测
# ========================

def is_collision(node, grid):
    x = int(node.x)
    y = int(node.y)
    
    if x < 0 or y < 0 or x >= grid.shape[1] or y >= grid.shape[0]:
        return True
    
    return grid[y, x] == 1


# ========================
# 运动模型
# ========================

def simulate_motion(node, steering_angle, direction=1, step_size=1.0, L=2.5):
    dt = 0.5
    v = 1.0
    # 假设每次希望探索前进 4 个格子 (大约 0.2 米)
    step_cells = 4.0 
    # 假设机器人的轴距等效于 8 个格子
    L_cells = 8.0
    # x = node.x + direction * v * math.cos(node.theta) * dt
    # y = node.y + direction * v * math.sin(node.theta) * dt
    #theta = node.theta + direction * v / L * math.tan(steering_angle) * dt


    x = node.x + direction * step_cells * math.cos(node.theta)
    y = node.y + direction * step_cells * math.sin(node.theta)

    theta = node.theta + direction * step_cells / L_cells * math.tan(steering_angle)
    #将角度严格限制在 [-pi, pi] 之间，防止原地转圈产生无限个新状态
    theta = (theta + math.pi) % (2 * math.pi) - math.pi
    return Node(x, y, theta, direction)


# ========================
# Hybrid A*
# ========================

# def hybrid_astar(start, goal, grid):

#     open_list = []
#     closed_set = set()

#     heapq.heappush(open_list, start)

#     steering_angles = np.deg2rad([-30, -15, 0, 15, 30])
#     directions = [1,-1]  # 前进和后退

#     while open_list:

#         current = heapq.heappop(open_list)
#         theta_bin = int(round(current.theta / (math.pi / 36)))
#         theta_bin = theta_bin % 72
#         key = (int(current.x), int(current.y), theta_bin, current.direction)
#         if key in closed_set:
#             continue

#         closed_set.add(key)

#         if heuristic(current, goal) < 2:
#             return current

#         for delta in steering_angles:
#             for direction in directions:
#                 new_node = simulate_motion(current, delta, direction)


#                 if is_collision(new_node, grid):
#                     continue
#                 steer_cost = abs(delta)
#                 reverse_cost = 0.5 if direction == -1 else 0 # 后退有额外代价
#                 new_node.g = current.g + math.hypot(new_node.x - current.x, new_node.y - current.y) + steer_cost*0.1 + reverse_cost
#                 new_node.h = heuristic(new_node, goal)
#                 new_node.parent = current

#                 heapq.heappush(open_list, new_node)

#     return None


#引入 2D 距离启发  局部最优陷阱

def hybrid_astar(start, goal, grid):
    # 【新增】：在正式开始搜索前，先计算出整张地图的 2D 绕行启发值
    h_map = calc_2d_heuristic(grid, goal)

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start)

    steering_angles = np.deg2rad([-30, -15, 0, 15, 30])
    directions = [1, -1]  

    while open_list:
        current = heapq.heappop(open_list)

        # 这里的 key 保留你上次修改的 72 份离散化
        theta_bin = int(round(current.theta / (math.pi / 36))) % 72
        key = (int(current.x), int(current.y), theta_bin, current.direction)
        
        if key in closed_set:
            continue
        closed_set.add(key)

        # 判断是否到达终点
        if math.hypot(current.x - goal.x, current.y - goal.y) < 2.0:
            return current

        for delta in steering_angles:
            for direction in directions:
                # 记得保留你上次修改的包含方向和角度归一化的 simulate_motion
                new_node = simulate_motion(current, delta, direction)

                if is_collision(new_node, grid):
                    continue
                
                nx, ny = int(new_node.x), int(new_node.y)
                if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
                    new_node.h = h_map[ny, nx]
                else:
                    new_node.h = float('inf') # 超出边界的代价为无穷大

                steer_cost = abs(delta)
                reverse_cost = 0.5 if direction == -1 else 0 
                
                # 更新 g 值 (实际代价)
                new_node.g = current.g + math.hypot(new_node.x - current.x, new_node.y - current.y) + steer_cost * 0.1 + reverse_cost
                new_node.parent = current

                # 只有 h 值不是无穷大时才加入 open_list（过滤掉被障碍物完全死锁的节点）
                if new_node.h != float('inf'):
                    heapq.heappush(open_list, new_node)

    return None
# ========================
# 轨迹回溯
# ========================

def extract_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]


# ========================
# 主函数
# ========================

if __name__ == "__main__":

    grid = create_map()

    start = Node(5, 5, 0)
    goal = Node(45, 45, 0)

    result = hybrid_astar(start, goal, grid)

    if result:
        path = extract_path(result)

        plt.imshow(grid.T, origin='lower', cmap='gray')
        xs, ys = zip(*path)
        plt.plot(xs, ys, 'r')
        plt.scatter(start.x, start.y, c='green')
        plt.scatter(goal.x, goal.y, c='blue')
        plt.title("Hybrid A*")
        plt.show()
    else:
        print("No Path Found")
        plt.imshow(grid.T, origin='lower', cmap='gray')
        plt.scatter(start.x, start.y, c='green')
        plt.scatter(goal.x, goal.y, c='blue')
        plt.title("Hybrid A* - No Path Found")
        plt.show()