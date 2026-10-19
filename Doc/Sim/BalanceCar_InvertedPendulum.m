%% Full_Balance_Sim.m
% 根据提供的 motionCtrlTask.c 完整还原的平衡车仿真
clear; clc; close all;

%% 1. 物理参数 (根据实际情况可微调)
p.g       = 9.81;       
p.m       = 0.60;       % 车身质量 kg
p.M       = 0.20;       % 轮子等效质量 kg
p.l       = 0.07;       % 质心到轴距离 m
p.I       = 0.006;      % 转动惯量 kg*m^2
p.b       = 0.04;       % 平移阻尼
p.c       = 0.002;      % 旋转阻尼
p.r       = 0.03175;    % 轮子半径 m

p.Ts      = 0.002;      % 仿真步长 5ms (500Hz, 对应 TASK_FREQ_MOTIONCTRL)
p.Tend    = 10.0;       % 仿真时间 10s
p.theta0  = deg2rad(-8);% 初始倾角 10度
p.xdot0   = 0.0;        
p.thetadot0 = 0.0;      

% 编码器增益：假设车速 1m/s 对应 v_enc = 100
p.enc_gain = 100;       

% 电机推力增益：假设 1000 PWM 对应 10N 推力
p.pwm_gain = 0.01;      

%% 2. 控制器参数 (完全照搬 C 代码)
% PID 结构体模拟
pid_pitch.Kp = 200.0; pid_pitch.Ki = 0.0;   pid_pitch.Kd = 10;
pid_pitch.err = [0 0 0]; pid_pitch.Iout = 0; pid_pitch.out = 0;
pid_pitch.max_out = 3000.0; pid_pitch.max_iout = 500.0;

pid_vel.Kp = 0.1;   pid_vel.Ki = 0.001;  pid_vel.Kd = 0.001;
pid_vel.err = [0 0 0]; pid_vel.Iout = 0; pid_vel.out = 0;
pid_vel.max_out = 10.0; pid_vel.max_iout = 0.01; % 注意：输出限幅是10度

pid_turn.Kp = 1.0;  pid_turn.Ki = 0.0;   pid_turn.Kd = 0.0;
pid_turn.err = [0 0 0]; pid_turn.Iout = 0; pid_turn.out = 0;
pid_turn.max_out = 1.0; pid_turn.max_iout = 1.0;

% 设定值
v_enc_set = 0;   % 期望前进
wz_enc_set = 0;   % 期望转向

%% 3. 初始化数组
N = floor(p.Tend / p.Ts) + 1;
t = (0:N-1)' * p.Ts;
x = zeros(N,1); xdot = zeros(N,1);
theta = zeros(N,1); thetadot = zeros(N,1);
v_enc = zeros(N,1); 
pwm_cn1 = zeros(N,1); pwm_cn2 = zeros(N,1);
force = zeros(N,1); % 实际作用于车体的等效推力

theta(1) = p.theta0;

%% 4. 闭环仿真循环
for k = 1:N-1
    % 1. 模拟 sensor 读取
    v_enc(k) = xdot(k) * p.enc_gain;    % 当前速度对应编码器值
    pitch_deg = -rad2deg(theta(k));      % 当前倾角 (度)
    gyro_z = thetadot(k);               % Z轴角速度，用于转向，此处近似为0
    
    % 2. 模拟 usr_pid_caculate 函数 (位置式)
    % --- 外环：速度环 ---
    pid_vel.err = [v_enc_set - v_enc(k), pid_vel.err(1:2)];
    pid_vel.Pout = pid_vel.Kp * pid_vel.err(1);
    pid_vel.Iout = pid_vel.Iout + pid_vel.Ki * pid_vel.err(1); % 注意C代码里可能没乘Ts
    pid_vel.Iout = min(max(pid_vel.Iout, -pid_vel.max_iout), pid_vel.max_iout);
    pid_vel.Dout = pid_vel.Kd * (pid_vel.err(1) - pid_vel.err(2));
    pid_vel.out = pid_vel.Pout + pid_vel.Iout + pid_vel.Dout;
    pid_vel.out = min(max(pid_vel.out, -pid_vel.max_out), pid_vel.max_out);
    
    % --- 内环：直立环 ---
    pid_pitch.err = [pid_vel.out - pitch_deg, pid_pitch.err(1:2)];
    pid_pitch.Pout = pid_pitch.Kp * pid_pitch.err(1);
    pid_pitch.Iout = pid_pitch.Iout + pid_pitch.Ki * pid_pitch.err(1);
    pid_pitch.Iout = min(max(pid_pitch.Iout, -pid_pitch.max_iout), pid_pitch.max_iout);
    pid_pitch.Dout = pid_pitch.Kd * (pid_pitch.err(1) - pid_pitch.err(2));
    pid_pitch.out = pid_pitch.Pout + pid_pitch.Iout + pid_pitch.Dout;
    pid_pitch.out = min(max(pid_pitch.out, -pid_pitch.max_out), pid_pitch.max_out);
    
    % --- 转向环 ---
    pid_turn.err = [wz_enc_set - gyro_z, pid_turn.err(1:2)];
    pid_turn.out = pid_turn.Kp * pid_turn.err(1); % 仅P控制
    pid_turn.out = min(max(pid_turn.out, -pid_turn.max_out), pid_turn.max_out);
    
    % 3. 输出合成
    pwm_cn1(k) = pid_pitch.out + pid_turn.out;
    pwm_cn2(k) = -pid_pitch.out - pid_turn.out;
    
    % 4. 转换为实际物理推力 (两个轮子合力)
    % 假设 pwm > 0 驱动车轮向前，产生向前的推力
    force(k) = pwm_cn1(k) * p.pwm_gain; 
    
    % 5. 动力学解算 (Cart-Pole 非线性模型)
    A11 = p.M + p.m;
    A12 = p.m * p.l * cos(theta(k));
    A21 = A12;
    A22 = p.I + p.m * p.l^2;
    
    rhs1 = force(k) - p.b * xdot(k) + p.m * p.l * sin(theta(k)) * thetadot(k)^2;
    rhs2 = p.m * p.g * p.l * sin(theta(k)) - p.c * thetadot(k);
    
    detA = A11 * A22 - A12 * A21;
    xdd = (rhs1 * A22 - A12 * rhs2) / detA;
    thetadd = (A11 * rhs2 - A21 * rhs1) / detA;
    
    % 6. 积分更新
    xdot(k+1) = xdot(k) + xdd * p.Ts;
    thetadot(k+1) = thetadot(k) + thetadd * p.Ts;
    x(k+1) = x(k) + xdot(k+1) * p.Ts;
    theta(k+1) = theta(k) + thetadot(k+1) * p.Ts;
    
    % 翻车保护
    if abs(theta(k+1)) > deg2rad(60)
        disp('车翻啦！');
        break;
    end
end

%% 5. 绘图与分析
figure('Name', '平衡车状态跟踪', 'Position', [100, 100, 800, 600]);

subplot(3,1,1);
plot(t, rad2deg(theta), 'r', 'LineWidth', 1.5); hold on;
yline(0, 'k--');
ylabel('Pitch (deg)');
title('车身倾角');
grid on;

subplot(3,1,2);
plot(t, v_enc, 'b', 'LineWidth', 1.5); hold on;
yline(v_enc_set, 'k--');
ylabel('v_{enc}');
title('速度环反馈与设定');
grid on;

subplot(3,1,3);
plot(t, x, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('位移 (m)');
title('车轮位移');
grid on;