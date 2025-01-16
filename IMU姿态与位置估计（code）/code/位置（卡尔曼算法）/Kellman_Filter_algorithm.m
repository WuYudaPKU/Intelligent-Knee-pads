H = [1 0 0 0 0 0;  % 观测矩阵
     0 1 0 0 0 0;
     0 0 1 0 0 0];

Q = eye(6) * 0.1; % 过程噪声协方差
R = eye(3) * 0.5; % 测量噪声协方差
P = eye(6);       % 状态协方差
x = zeros(6, 1);  % 初始状态

% kellman filter主循环
for i=1:n-1
    dt=time(i+1)-time(i);
    F=[1 0 0 dt 0 0;
       0 1 0 0 dt 0;
       0 0 1 0 0 dt;
       0 0 0 1 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1];

    x = F * x;           % 状态预测
    P = F * P * F' + Q;  % 误差协方差预测

    % 2. 观测值更新（假设传感器数据为 z_k）
    z_k = imu_measurements(k, :); % 当前加速度数据
    K = P * H' / (H * P * H' + R); % 卡尔曼增益
    x = x + K * (z_k' - H * x);    % 更新状态估计
    P = (eye(6) - K * H) * P;      % 更新协方差
end