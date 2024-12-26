clear all;
close all;
clc;

%% 定义系统参数
% 定义重力加速度
g = 10;
% 定义平衡车连杆长度
d = 1;

A = [0 1; g/d 0];
B = [0; 1];
C = [1, 0];
D = 0;

Q = [100 0;0 1];
R = 1;
% 状态初始化
x0 = [pi/20; 0]; x = x0;

% 代数Riccati方程求解
[P, L1, G1] = care(A, B, Q, R);
% 计算反馈增益，参考式（4.4.52）
K = inv(R) * B' * P;

% 构建闭环系统
sys_cl = ss(A-B*K, [0;0], C, D);

%% 系统仿真
t_span = 0.01;
% 系统时间离散
t = 0 : t_span : 5;
% 闭环系统初值响应
[y, t, x] = initial(sys_cl, x, t);

%% 系统响应视图，状态量x1（角度）
subplot(3,1,1);
plot(t, x(:,1), "linewidth", 2);
% 近似计算x1的积分
% x1_1_cost = x(:,1)' * x(:,1) * t_span;
xlim([0 4]);
grid on;

%% 系统响应结果视图，状态量x2（角速度）
subplot(3,1,2);
plot(t, x(:,2), "linewidth", 2);
% 近似计算x2的积分
% x1_2_cost = x(:,2)'*x(:,2)*t_span;
xlim([0 4]);
grid on;

%% 系统响应结果视图，输入（加速度）
subplot(3,1,3);
plot(t, -K*x', "linewidth",2);
% 近似计算u的总代价
% u1_cost = (-K*x')*(-K*x')'*t_span';
xlim([0 4]);
grid on;