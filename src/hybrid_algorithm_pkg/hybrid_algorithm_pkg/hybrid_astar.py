import math
import heapq
import numpy as np
import matplotlib.pyplot as plt

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
    
    if x < 0 or y < 0 or x >= grid.shape[0] or y >= grid.shape[1]:
        return True
    
    return grid[x, y] == 1


# ========================
# 运动模型
# ========================

def simulate_motion(node, steering_angle, direction=1, step_size=1.0, L=2.5):
    dt = 0.5
    v = 1.0

    x = node.x + direction * v * math.cos(node.theta) * dt
    y = node.y + direction * v * math.sin(node.theta) * dt
    theta = node.theta + direction * v / L * math.tan(steering_angle) * dt

    return Node(x, y, theta, direction)


# ========================
# Hybrid A*
# ========================

def hybrid_astar(start, goal, grid):

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start)

    steering_angles = np.deg2rad([-30, -15, 0, 15, 30])
    directions = [1,-1]  # 前进和后退

    while open_list:

        current = heapq.heappop(open_list)

        key = (int(current.x), int(current.y), int(current.theta*10) ,current.direction)
        if key in closed_set:
            continue

        closed_set.add(key)

        if heuristic(current, goal) < 2:
            return current

        for delta in steering_angles:
            for direction in directions:
                new_node = simulate_motion(current, delta)


                if is_collision(new_node, grid):
                    continue
                steer_cost = abs(delta)
                reverse_cost = 0.5 if direction == -1 else 0 # 后退有额外代价
                new_node.g = current.g + math.hypot(new_node.x - current.x, new_node.y - current.y) + steer_cost*0.1 + reverse_cost
                new_node.h = heuristic(new_node, goal)
                new_node.parent = current

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