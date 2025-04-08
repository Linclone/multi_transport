import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Vehicle:
    def __init__(self):
        self.x = 12.0  # 初始x坐标
        self.y = 0.0   # 初始y坐标
        self.yaw = np.pi/2  # 初始朝向（弧度）
        self.v = 2.0   # 固定线速度 m/s
        self.L = 2.0   # 轮距
        self.max_steer = np.radians(180)  # 最大转向角

    def update(self, omega, dt):
        # 限制角速度范围
        omega = np.clip(omega, -self.max_steer, self.max_steer)
        
        # 差速轮速度计算
        vr = self.v + (self.L * omega) / 2
        vl = self.v - (self.L * omega) / 2
        
        # 更新位姿
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += omega * dt
        
        return self.x, self.y, self.yaw

class PurePursuit:
    def __init__(self, path):
        self.path = path
        self.lookahead_distance = 3.0  # 预瞄距离
        self.current_idx = 0

    def search_target(self, vehicle):
        # 寻找最近的路径点
        dx = [vehicle.x - x for x, y in self.path]
        dy = [vehicle.y - y for x, y in self.path]
        d = np.hypot(dx, dy)
        nearest_idx = np.argmin(d)

        # 寻找预瞄点
        for i in range(nearest_idx, len(self.path)):
            if np.hypot(self.path[i][0]-vehicle.x, self.path[i][1]-vehicle.y) >= self.lookahead_distance:
                self.current_idx = i
                return self.path[i]
        return self.path[-1]

def generate_ellipse_path(a=10, b=5, num=100):
    theta = np.linspace(0, 2*np.pi, num)
    x = a * np.cos(theta)
    y = b * np.sin(theta)
    return list(zip(x, y))

def pure_pursuit_control(vehicle, target):
    alpha = np.arctan2(target[1] - vehicle.y, target[0] - vehicle.x) - vehicle.yaw
    omega = 2 * vehicle.v * np.sin(alpha) / np.hypot(target[0]-vehicle.x, target[1]-vehicle.y)
    return omega

# 生成椭圆路径
path = generate_ellipse_path()

# 初始化车辆和控制器
vehicle = Vehicle()
controller = PurePursuit(path)

# 动画初始化函数
def init():
    path_line.set_data([x for x, y in path], [y for x, y in path])
    return path_line, vehicle_plot, trajectory,

# 动画更新函数
def update(frame):
    target = controller.search_target(vehicle)
    omega = pure_pursuit_control(vehicle, target)
    vehicle.update(omega, dt=0.1)
    
    # 更新轨迹
    x_data = list(trajectory.get_xdata()) + [vehicle.x]
    y_data = list(trajectory.get_ydata()) + [vehicle.y]
    
    vehicle_plot.set_data([vehicle.x], [vehicle.y])  # 添加方括号创建列表
    trajectory.set_data(x_data, y_data)
    
    return vehicle_plot, trajectory,

# 运行动画
ani = FuncAnimation(fig, update, frames=200,
                    init_func=init, blit=True, interval=50)

plt.show()