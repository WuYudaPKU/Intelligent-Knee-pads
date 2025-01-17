% Date         Author        Notes
% 2023.10.25   Yutong Shi    

addpath('quaternion_library');      % 添加四元数库
close all;  clear;  clc;
                               
% 传感器数据预处理及绘制
% [num,txt,raw]是固定的数据读取方式;num将储存所有的纯数值部分;txt保存所有的非纯数值部分;raw储存所有的数据（元胞形式）
[num,txt,raw] = xlsread('D:\储存\KneesPad_wyd\Intelligent-Knee-pads\石师姐代码\data\data\1_imu_data.xlsx');

Accelerometer = num(:,1:3); %%根据原始数据，提取所有行的1-3列（分别加速度的xyz）
Accelerometer= Accelerometer*9.807; %原始的加速度数据单位是g
Gyroscope = num(:,4:6);  %%角速度（xyz）
Magnetometer = num(:,10:12);  %%磁场（xyz）

% 时间戳预处理
timeStamp = txt(2:end,1); %从第二行到结尾的第一列，所有的原始时间
timechar = char(timeStamp); 
for i = 1: length(timeStamp)
    split_timecell(i,:) = strsplit(timechar(i,:), ':');    
end

% 获取 Cell 数组的维度
rows = size(split_timecell, 1);
cols = size(split_timecell, 2);
doubleArray = zeros(size(split_timecell));              % 初始化一个与 split_timecell 大小相同的 double 数组
for i = 1:numel(split_timecell)
    doubleArray(i) = str2double(split_timecell{i});     % 将每个字符串转换为 double 存储到 doubleArray 中
end
doubleArray = reshape(doubleArray, [rows, cols]);       % 将 doubleArray 转换为适当的维度

time_a = doubleArray(:,3);
time_b = doubleArray(:,4);
time = time_a + time_b./1000;

time = time - time(1,:);
for i = 1:length(time)-1
    if time(i+1) - time(i) < 0
        time(i+1:end,:) = time(i+1:end,:) + 60;
    end
end

%绘图，绘制三个方向角速度数据
figure('Name', 'Sensor Data');
axis(1) = subplot(2,1,1);                   %表示一个二行一列的图中第一个位置
hold on;
plot(time, Gyroscope(:,1));
plot(time, Gyroscope(:,2));
plot(time, Gyroscope(:,3));
legend('X', 'Y', 'Z'); %%添加了图例
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');             %注意原始陀螺仪数据是角度值
title('Gyroscope');
hold off;

%绘图，绘制三个方向加速度数据
axis(1) = subplot(2,1,2);                   %表示一个二行一列的图中第二个位置
hold on;
plot(time, Accelerometer(:,1));
plot(time, Accelerometer(:,2));
plot(time, Accelerometer(:,3));
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration ');
title('Accelerometer');
hold off;
linkaxes(axis, 'x');                        %将一幅图中的三个图像坐标轴同步

% 处理数据
% SamplePeriod = 0.04;
quaternion = zeros(length(time), 4);
quaternion(1,:) = [1 0 0 0];
q = quaternion;                             %为了方便起见，以下全部用q代替
Gyroscope = Gyroscope * (pi/180);           %陀螺仪数据角度转弧度

% 纯四元数算法，得到每个时刻的四元数
for i = 1:length(time)-1
    qDot = 0.5 * quaternProd(q(i,:), [0 Gyroscope(i+1,1) Gyroscope(i+1,2) Gyroscope(i+1,3)]);
    q(i+1,:) = q(i,:) + qDot * (time(i+1)-time(i));
    q(i+1,:) = q(i+1,:) / norm(q(i+1,:));             % 四元数归一化 
end

R = quatern2rotMat((q));
euler = quatern2euler(quaternConj(q)) * (180/pi);
% 有些角度超过+-180
for i = 2:length(time)
    if euler(i,1)-euler(i-1,1) > 180
        euler(i,1) = euler(i,1) - 360;
    end
    if euler(i,2)-euler(i-1,2) > 180
        euler(i,2) = euler(i,2) - 360;
    end
    if euler(i,3)-euler(i-1,3) > 180
        euler(i,3) = euler(i,3) - 360;
    end
end

% 绘制欧拉角数据
figure('Name', 'Euler Angles');
set(gcf,'unit','centimeters','position',[10,10,8,6])
hold on;
plot(time, euler(:,1),'color',[0.85,0.33,0.1],'LineWidth',1.5);
plot(time, euler(:,2),'color',[0,0.45,0.74],'LineWidth',1.5);
plot(time, euler(:,3),'color',[0.93,0.69,0.13],'LineWidth',1.5);
title('Euler angles');
set(gca,'FontSize',10,'linewidth',1,'FontName','Times New Roman');    %横纵坐标轴
xlabel('Time (s)','FontSize',10,'FontName','Times New Roman');
ylabel('Angle (deg)','FontSize',10,'FontName','Times New Roman');
lgd=legend('\phi', '\theta', '\psi');
% lgd.NumColumns = 3;
hold off;


% 初始化线性加速度
linear_acc = zeros(length(time), 3);

% 估算重力加速度方向 (假设重力加速度在全局z方向)
gravity = [0, 0, 9.807]; % 重力加速度 (m/s^2)

% 去除重力加速度
for i = 1:length(time)
    % 获取当前时刻的旋转矩阵 R
    Ri = R(:,:,i); 
    % 将加速度从IMU坐标系转换到全局坐标系
    acc_global = (Ri * Accelerometer(i,:)')'; 
    % 计算去除重力后的线性加速度
    linear_acc(i,:) = acc_global - gravity; 
end

% 绘制线性加速度
figure('Name', 'Linear Acceleration');
hold on;
plot(time, linear_acc(:,1),'color',[0.85,0.33,0.1],'LineWidth',1.5);
plot(time, linear_acc(:,2),'color',[0,0.45,0.74],'LineWidth',1.5);
plot(time, linear_acc(:,3),'color',[0.93,0.69,0.13],'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Linear Acceleration (m/s^2)');
title('Linear Acceleration');
legend('X', 'Y', 'Z');
hold off;

% 初始化状态向量 [位置; 速度; 四元数]
x = [0; 0; 0; 0; 0; 0; 1; 0; 0; 0];  % 假设初始位置在原点，零速度，单位四元数

% 初始化协方差矩阵
P = eye(10);  % 初始协方差矩阵，10维状态向量

% 状态转移矩阵 F
F = [eye(3), delta_t * eye(3), zeros(3, 4); 
     zeros(3, 3), eye(3), zeros(3, 4); 
     zeros(4, 6), eye(4)];

% 控制输入矩阵 B
B = [delta_t * eye(3), zeros(3, 3); 
     zeros(3, 3), delta_t * eye(3); 
     zeros(4, 6)];

% 预测步骤
x_pred = F * x + B * u;  % 状态预测
P_pred = F * P * F' + Q;  % 协方差预测

% 更新步骤
K = P_pred * H' * inv(H * P_pred * H' + R);  % 卡尔曼增益
x = x_pred + K * (z - H * x_pred);  % 状态更新
P = (eye(10) - K * H) * P_pred;  % 协方差更新

% 假设已经初始化了以下变量：
% x: 初始状态向量 [位置; 速度; 四元数]
% P: 初始协方差矩阵
% Q: 过程噪声协方差矩阵
% R: 观测噪声协方差矩阵
% F: 状态转移矩阵
% B: 控制输入矩阵
% H: 观测矩阵
% time: 时间序列
% Accelerometer: 加速度计数据
% Gyroscope: 陀螺仪数据
% delta_t: 时间步长
% 控制输入 u = [加速度; 角速度]

for k = 2:length(time)  % 从第二个时间步开始，第一步是初始化
    delta_t=time(k+1)-time(k);
    % 1. 计算当前时间步的控制输入 u (加速度和角速度)
    acc_k = Accelerometer(k, :);  % 当前加速度数据
    gyro_k = Gyroscope(k, :);     % 当前角速度数据
    u = [acc_k, gyro_k];  % 控制输入向量 [加速度; 角速度]
    
    % 2. 状态预测步骤 (预测位置、速度、四元数)
    x_pred = F * x + B * u';  % 预测的状态向量
    P_pred = F * P * F' + Q;  % 预测的协方差矩阵
    
    % 3. 计算当前的观测值 (根据加速度和角速度进行更新)
    % 对应于加速度（线性加速度）和角速度
    z = [linear_acc(k, :), gyro_k];  % 当前时刻的观测值
    
    % 4. 计算卡尔曼增益 K
    K = P_pred * H' * inv(H * P_pred * H' + R);  % 卡尔曼增益
    
    % 5. 状态更新步骤
    x = x_pred + K * (z' - H * x_pred);  % 更新状态向量
    P = (eye(10) - K * H) * P_pred;  % 更新协方差矩阵
    
    % 6. 处理四元数的归一化 (确保四元数的单位性)
    q = x(7:10);  % 提取四元数部分
    q = q / norm(q);  % 归一化四元数
    
    % 7. 更新状态向量
    x(7:10) = q;  % 更新四元数部分
end
