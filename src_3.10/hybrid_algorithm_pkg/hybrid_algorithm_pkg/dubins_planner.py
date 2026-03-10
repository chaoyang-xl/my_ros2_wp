import math
import numpy as np


def mod2pi(theta):
    return theta - 2.0 * math.pi * math.floor(theta / (2.0 * math.pi))


def dubins_path_planning(sx, sy, syaw,
                         gx, gy, gyaw,
                         curvature, step_size=0.5):

    dx = gx - sx
    dy = gy - sy
    D = math.hypot(dx, dy)
    d = D * curvature

    theta = mod2pi(math.atan2(dy, dx))
    alpha = mod2pi(syaw - theta)
    beta = mod2pi(gyaw - theta)

    best_cost = float("inf")
    best_path = None

    # 这里只实现 LSL 类型（够用做解析连接）
    tmp = d + alpha - beta

    if tmp > 0:
        path = generate_course(tmp, curvature, step_size, sx, sy, syaw)
        best_path = path

    return best_path


def generate_course(length, curvature, step_size, sx, sy, syaw):

    x, y, yaw = sx, sy, syaw
    path = []

    for _ in np.arange(0, length, step_size):
        x += step_size * math.cos(yaw)
        y += step_size * math.sin(yaw)
        path.append((x, y, yaw))

    return path