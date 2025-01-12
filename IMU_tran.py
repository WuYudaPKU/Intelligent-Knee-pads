import numpy as np
from scipy.spatial.transform import Rotation as R

# 初始化四元数
def quaternion_update(q, omega, dt):
    """根据角速度更新四元数"""
    omega_quat = np.array([0, omega[0], omega[1], omega[2]])
    dq = 0.5 * quat_multiply(q, omega_quat) * dt
    return q + dq / np.linalg.norm(q + dq)

# 四元数乘法
def quat_multiply(q1, q2):
    """四元数乘法"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])

# 数据融合函数
def imu_fusion(imu_positions, imu_data):
    """
    利用多个 IMU 数据计算刚体的运动状态
    :param imu_positions: 每个 IMU 的相对位置，形状 (3, 3)。
    :param imu_data: 每个 IMU 的测量数据（加速度和角速度），形状 (3, 6)。
    :return: 刚体的线性加速度和角速度
    """
    # 解构 IMU 数据
    accelerations = imu_data[:, :3]  # 加速度 (3, 3)
    angular_velocities = imu_data[:, 3:]  # 角速度 (3, 3)
    
    # 线性加速度：各 IMU 数据简单平均
    acc = np.mean(accelerations, axis=0)
    
    # 角速度：考虑位置权重
    ang_vel = np.mean(angular_velocities, axis=0)
    
    return acc, ang_vel

# 主函数
def rigid_body_motion(imu_positions, imu_data, dt, steps):
    """
    计算刚体的位置和姿态
    :param imu_positions: IMU 相对位置
    :param imu_data: IMU 数据
    :param dt: 时间间隔
    :param steps: 模拟步数
    :return: 刚体的姿态（四元数）和位置
    """
    # 初始化状态
    position = np.zeros(3)  # 初始位置
    velocity = np.zeros(3)  # 初始速度
    quaternion = np.array([1, 0, 0, 0])  # 初始姿态（四元数）
    
    # 记录结果
    positions = []
    quaternions = []
    
    for _ in range(steps):
        # 数据融合
        acc, ang_vel = imu_fusion(imu_positions, imu_data)
        
        # 更新姿态
        quaternion = quaternion_update(quaternion, ang_vel, dt)
        
        # 更新位置
        world_acc = R.from_quat(quaternion).apply(acc)  # 转为世界坐标系加速度
        velocity += world_acc * dt
        position += velocity * dt
        
        # 存储结果
        positions.append(position.copy())
        quaternions.append(quaternion.copy())
    
    return np.array(positions), np.array(quaternions)

# 示例输入
imu_positions = np.array([
    [0.1, 0.0, 0.0],  # IMU1 相对位置
    [0.0, 0.1, 0.0],  # IMU2 相对位置
    [-0.1, 0.0, 0.0], # IMU3 相对位置
])

imu_data = np.array([
    [0, 0, 9.8, 0, 0.1, 0],  # IMU1 加速度和角速度
    [0, 0, 9.8, 0, 0.1, 0],  # IMU2 加速度和角速度
    [0, 0, 9.8, 0, 0.1, 0],  # IMU3 加速度和角速度
])

# 模拟计算
positions, quaternions = rigid_body_motion(imu_positions, imu_data, dt=0.01, steps=100)

# 输出结果
print("位置：", positions)
print("姿态（四元数）：", quaternions)