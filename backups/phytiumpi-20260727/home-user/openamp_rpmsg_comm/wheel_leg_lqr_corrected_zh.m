clear;
clc;
close all;

%% 固定腿高轮腿机器人：四状态离散 LQR 长期调参模板
%
% 本文件不是一次性算 K 的脚本，而是固定腿高阶段的调参模板。
% 日常使用只需要修改“用户配置区”，其余建模、批量仿真、筛选、绘图和
% 固件命令输出会自动完成。
%
% 状态：
%   x = [theta; theta_dot; position; velocity]
%
% 输入：
%   tau_total = tau_left + tau_right，单位 N*m
%
% 正方向：
%   theta > 0      车身向前倾
%   position > 0   机器人向前移动
%   tau_total > 0  轮子驱动机器人向前
%
% 适用范围：腿部相对车身完全固定。腿部关节参与运动后，必须重新建立
% 至少六状态的轮腿模型，不能继续直接使用这里的四状态增益。

%% 1. 用户配置区：机械和固件参数

cfg.m_total = 6.95;       % 整车总质量，kg
cfg.m_wheels = 0.19;      % 两个旋转轮组总质量，kg
cfg.M_body = cfg.m_total - cfg.m_wheels;

cfg.wheel_radius_m = 0.03225;      % 车轮有效滚动半径，m
cfg.com_height_m = 0.073;          % 固定车身质心到轮轴高度，m
cfg.body_pitch_inertia = 0.025;    % 车身俯仰惯量，kg*m^2
cfg.gravity = 9.81;

% 没有 CAD/实测结果时，将两个轮组近似为实心圆盘。
cfg.wheel_inertia = ...
    0.5 * cfg.m_wheels * cfg.wheel_radius_m^2;

% 必须与从核固件完全一致。
cfg.control_hz = 100;
cfg.Ts = 1 / cfg.control_hz;
cfg.pitch_rate_filter_hz = 20.0;
cfg.posture_priority_angle_deg = 3.0;

% 当前电机协议和固件限制。
cfg.single_torque_limit_nm = 0.22;
cfg.single_torque_resolution_nm = 0.01;
cfg.total_torque_limit_nm = 2 * cfg.single_torque_limit_nm;
cfg.wheel_speed_limit_m_s = 1.0;

% 固件和执行器近似。命令延迟为1表示本拍计算、下一拍生效。
cfg.command_delay_samples = 1;
cfg.use_torque_quantization = true;

% 如果后续通过阶跃测试测出了力矩响应时间常数，再填入非零值。
% 当前先设为0，表示除了上面的一拍延迟外，执行器立即跟随命令。
cfg.actuator_time_constant_s = 0.0;

% 仿真时长和最终稳态统计窗口。
cfg.simulation_time_s = 8.0;
cfg.steady_window_s = 1.0;
cfg.fall_angle_deg = 15.0;

%% 2. 用户配置区：候选 Q/R 计划
%
% 2026-07-16 实机日志显示：
%   1. K2角速度项占用的单轮力矩 RMS 约0.192 N*m；
%   2. 单轮力矩限制为0.22 N*m；
%   3. 后半段约44%%采样点饱和，并出现约5 Hz可见振荡；
%   4. 因此先降低角速度通道和整体控制强度，再单独增强位置回零。
%
% 列定义：
%   id, q_theta, q_theta_dot, q_position, q_velocity, R_scale
%
% 调参顺序：
%   A. baseline 只用于和旧参数比较；
%   B. log_target_soft 是下一轮优先候选；
%   C. lower_rate 用于 B 仍有高频振荡时；
%   D. stronger_position 只能在姿态振荡解决后测试；
%   E. global_soft 用于整体动作仍然过猛时。

candidate_definitions = {
    'baseline',          1200, 80, 2,  10, 0.50;
    'log_target_soft',   1200, 50, 2,  10, 0.75;
    'lower_rate',        1200, 30, 2,  10, 0.75;
    'stronger_position', 1200, 50, 4,  16, 0.75;
    'global_soft',       1200, 40, 2,  10, 1.00;
};

% 选择要绘制详细曲线和输出部署命令的候选。修改这里即可切换。
selected_candidate_id = 'log_target_soft';

% 当前实机手动组合，仅用于对照。它不是同一组 Q/R 的 dlqr 完整结果。
runtime_reference_K = ...
    [-3.759673794, -0.400000000, -0.062456846, -0.247058408];

% 旧的等效水平力输入惩罚。转换到总力矩输入后再乘 R_scale。
R_force_reference = 0.5;

%% 3. 用户配置区：批量仿真场景
%
% x0 = [初始倾角deg, 初始角速度rad/s, 初始位置m, 初始速度m/s]
% pitch_bias_deg 模拟没有校准的 IMU/机械平衡点偏差。
% score_enabled=false 的场景只诊断，不参与候选排名。

scenario_definitions = {
    'pitch_2deg',      [2.0, 0.0, 0.0, 0.0],  0.0, true;
    'pitch_3deg',      [3.0, 0.0, 0.0, 0.0],  0.0, true;
    'pitch_5deg',      [5.0, 0.0, 0.0, 0.0],  0.0, true;
    'forward_speed',   [0.0, 0.0, 0.0, 0.30], 0.0, true;
    'position_offset', [0.0, 0.0, 0.20, 0.0], 0.0, true;
    'pitch_bias_-1',   [0.0, 0.0, 0.0, 0.0], -1.0, false;
};

%% 4. 参数检查和连续模型

validate_configuration(cfg, candidate_definitions, scenario_definitions);

fprintf('================ 模板配置 ================\n');
fprintf('整车总质量：                 %.6f kg\n', cfg.m_total);
fprintf('固定车身质量：               %.6f kg\n', cfg.M_body);
fprintf('两个旋转轮组总质量：         %.6f kg\n', cfg.m_wheels);
fprintf('车轮有效半径：               %.8f m\n', cfg.wheel_radius_m);
fprintf('质心高度：                   %.8f m\n', cfg.com_height_m);
fprintf('车身俯仰惯量：               %.9f kg*m^2\n', ...
        cfg.body_pitch_inertia);
fprintf('两个轮组总转动惯量：         %.9f kg*m^2\n', ...
        cfg.wheel_inertia);
fprintf('控制频率：                   %d Hz\n', cfg.control_hz);
fprintf('角速度低通截止频率：         %.3f Hz\n', ...
        cfg.pitch_rate_filter_hz);
fprintf('单轮力矩限制/分辨率：        %.3f / %.3f N*m\n', ...
        cfg.single_torque_limit_nm, cfg.single_torque_resolution_nm);
fprintf('命令延迟：                   %d 个控制周期\n', ...
        cfg.command_delay_samples);

M = cfg.M_body;
m = cfg.m_wheels;
r = cfg.wheel_radius_m;
l = cfg.com_height_m;
I = cfg.body_pitch_inertia;
Iw = cfg.wheel_inertia;
g = cfg.gravity;

a = I + M * l^2;
b = M * l;
c = M + m + Iw / r^2;
den = a * c - b^2;

if den <= 0
    error('模型分母 den <= 0，请检查机械参数。');
end

A = [0,                 1, 0, 0;
     c*M*g*l/den,       0, 0, 0;
     0,                 0, 0, 1;
    -b*M*g*l/den,       0, 0, 0];

% 输入是左右轮总力矩，不是外部水平推力。
B_tau = [0;
        -(c + b/r)/den;
         0;
         (b + a/r)/den];

C = eye(4);
D = zeros(4, 1);

fprintf('\n================ 连续模型 ================\n');
disp('A =');
disp(A);
disp('B_tau（输入为左右轮总力矩，N*m）=');
disp(B_tau);

continuous_rank = rank(ctrb(A, B_tau));
fprintf('连续可控性矩阵秩：%d / 4\n', continuous_rank);
if continuous_rank ~= 4
    error('连续模型不可控。');
end

%% 5. 按真实控制周期离散化

sys_c = ss(A, B_tau, C, D);
sys_d = c2d(sys_c, cfg.Ts, 'zoh');
Ad = sys_d.A;
Bd = sys_d.B;

discrete_rank = rank(ctrb(Ad, Bd));
fprintf('离散可控性矩阵秩：%d / 4\n', discrete_rank);
if discrete_rank ~= 4
    error('离散模型不可控。');
end

model.Ad = Ad;
model.Bd = Bd;

%% 6. 计算所有候选并执行带固件约束的批量仿真

candidate_count = size(candidate_definitions, 1);
scenario_count = size(scenario_definitions, 1);
R_tau_nominal = R_force_reference / r^2;

results = repmat(struct( ...
    'id', '', 'Q', [], 'R_tau', 0, 'K', [], 'poles', [], ...
    'stable', false, 'score', inf, 'worst_pitch_deg', inf, ...
    'worst_steady_pitch_deg', inf, 'worst_speed_m_s', inf, ...
    'worst_saturation_percent', inf, 'scenario_results', []), ...
    1, candidate_count);

fprintf('\n================ 候选增益和仿真 ================\n');

for candidate_index = 1:candidate_count
    definition = candidate_definitions(candidate_index, :);
    candidate_id = definition{1};
    q_values = cell2mat(definition(2:5));
    R_scale = definition{6};

    Q = diag(q_values);
    R_tau = R_tau_nominal * R_scale;
    K = dlqr(Ad, Bd, Q, R_tau);
    poles = eig(Ad - Bd * K);
    stable = all(abs(poles) < 1.0);

    scenario_results = [];
    score = 0.0;
    worst_pitch_deg = 0.0;
    worst_steady_pitch_deg = 0.0;
    worst_speed_m_s = 0.0;
    worst_saturation_percent = 0.0;

    for scenario_index = 1:scenario_count
        scenario = make_scenario(scenario_definitions(scenario_index, :));
        sim = simulate_firmware_controller(model, cfg, K, scenario);
        if scenario_index == 1
            scenario_results = repmat(sim, 1, scenario_count);
        else
            scenario_results(scenario_index) = sim;
        end

        worst_pitch_deg = max(worst_pitch_deg, sim.max_pitch_deg);
        worst_steady_pitch_deg = max(worst_steady_pitch_deg, ...
                                     sim.steady_pitch_rms_deg);
        worst_speed_m_s = max(worst_speed_m_s, sim.max_speed_m_s);
        worst_saturation_percent = max(worst_saturation_percent, ...
                                       sim.saturation_percent);

        if scenario.score_enabled
            % 排名只用于同一模板内的相对比较，不代表实机安全认证。
            score = score + ...
                3.0 * sim.max_pitch_deg / 5.0 + ...
                4.0 * sim.steady_pitch_rms_deg / 0.5 + ...
                2.0 * sim.saturation_percent / 10.0 + ...
                1.5 * sim.max_speed_m_s / 0.5 + ...
                1.0 * abs(sim.final_position_m) / 0.10;
        end
    end

    if ~stable || worst_pitch_deg >= cfg.fall_angle_deg
        score = inf;
    end

    results(candidate_index).id = candidate_id;
    results(candidate_index).Q = Q;
    results(candidate_index).R_tau = R_tau;
    results(candidate_index).K = K;
    results(candidate_index).poles = poles;
    results(candidate_index).stable = stable;
    results(candidate_index).score = score;
    results(candidate_index).worst_pitch_deg = worst_pitch_deg;
    results(candidate_index).worst_steady_pitch_deg = ...
        worst_steady_pitch_deg;
    results(candidate_index).worst_speed_m_s = worst_speed_m_s;
    results(candidate_index).worst_saturation_percent = ...
        worst_saturation_percent;
    results(candidate_index).scenario_results = scenario_results;

    fprintf('\n[%s]\n', candidate_id);
    fprintf('Q = diag([%.3f %.3f %.3f %.3f]), R_scale=%.3f\n', ...
            q_values(1), q_values(2), q_values(3), q_values(4), R_scale);
    fprintf('K = [%.9f %.9f %.9f %.9f]\n', K);
    fprintf('最坏倾角=%.3f deg，稳态倾角RMS=%.3f deg，', ...
            worst_pitch_deg, worst_steady_pitch_deg);
    fprintf('最坏速度=%.3f m/s，最坏饱和率=%.2f%%\n', ...
            worst_speed_m_s, worst_saturation_percent);
end

%% 7. 候选排名和所选部署命令

ids = cell(candidate_count, 1);
K1 = zeros(candidate_count, 1);
K2 = zeros(candidate_count, 1);
K3 = zeros(candidate_count, 1);
K4 = zeros(candidate_count, 1);
scores = zeros(candidate_count, 1);
worst_pitch = zeros(candidate_count, 1);
steady_pitch = zeros(candidate_count, 1);
worst_speed = zeros(candidate_count, 1);
worst_saturation = zeros(candidate_count, 1);

for i = 1:candidate_count
    ids{i} = results(i).id;
    K1(i) = results(i).K(1);
    K2(i) = results(i).K(2);
    K3(i) = results(i).K(3);
    K4(i) = results(i).K(4);
    scores(i) = results(i).score;
    worst_pitch(i) = results(i).worst_pitch_deg;
    steady_pitch(i) = results(i).worst_steady_pitch_deg;
    worst_speed(i) = results(i).worst_speed_m_s;
    worst_saturation(i) = results(i).worst_saturation_percent;
end

summary_table = table(ids, K1, K2, K3, K4, scores, worst_pitch, ...
    steady_pitch, worst_speed, worst_saturation, ...
    'VariableNames', {'id', 'K1', 'K2', 'K3', 'K4', 'score', ...
    'max_pitch_deg', 'steady_pitch_rms_deg', 'max_speed_m_s', ...
    'max_saturation_percent'});
summary_table = sortrows(summary_table, 'score');

fprintf('\n================ 候选排名 ================\n');
disp(summary_table);
fprintf(['排名仅用于线性模型和当前约束下的相对筛选。任何候选都必须经过' ...
         '保护绳、小角度、低风险实机测试。\n']);

selected_index = find(strcmp({results.id}, selected_candidate_id), 1);
if isempty(selected_index)
    error('selected_candidate_id 不存在：%s', selected_candidate_id);
end
selected = results(selected_index);

fprintf('\n================ 选择的候选 ================\n');
fprintf('候选：%s\n', selected.id);
fprintf('K1 = %.9ff;\n', selected.K(1));
fprintf('K2 = %.9ff;\n', selected.K(2));
fprintf('K3 = %.9ff;\n', selected.K(3));
fprintf('K4 = %.9ff;\n', selected.K(4));
fprintf('当前实机手动参考 K：\n');
fprintf('K = [%.9f %.9f %.9f %.9f]\n', runtime_reference_K);
fprintf('\n停用平衡后执行以下命令：\n');
fprintf('rprun balance-disable\n');
fprintf('rprun balance-gains %.9f %.9f %.9f %.9f\n', selected.K);
fprintf('rprun balance-config\n');

%% 8. 输出所选候选的逐场景指标

fprintf('\n================ 所选候选逐场景结果 ================\n');
for i = 1:scenario_count
    sim = selected.scenario_results(i);
    fprintf(['%-18s 最大倾角=%6.3f deg  稳态RMS=%6.3f deg  ' ...
             '最大速度=%6.3f m/s  饱和=%6.2f%%  最终位置=%7.3f m\n'], ...
            sim.name, sim.max_pitch_deg, sim.steady_pitch_rms_deg, ...
            sim.max_speed_m_s, sim.saturation_percent, ...
            sim.final_position_m);
end

%% 9. 绘制所选候选的主要场景

figure('Name', ['LQR候选：' selected.id]);
plot_count = min(5, scenario_count);

subplot(2, 2, 1);
hold on;
for i = 1:plot_count
    sim = selected.scenario_results(i);
    plot(sim.time, rad2deg(sim.x(1, :)), 'LineWidth', 1.2, ...
         'DisplayName', sim.name);
end
yline(cfg.fall_angle_deg, '--r');
yline(-cfg.fall_angle_deg, '--r');
grid on;
xlabel('时间 / s');
ylabel('俯仰角 / deg');
title('车身俯仰角');
legend('Location', 'best');

subplot(2, 2, 2);
hold on;
for i = 1:plot_count
    sim = selected.scenario_results(i);
    plot(sim.time, sim.filtered_pitch_rate, 'LineWidth', 1.2, ...
         'DisplayName', sim.name);
end
grid on;
xlabel('时间 / s');
ylabel('滤波角速度 / rad/s');
title('固件实际使用的俯仰角速度');
legend('Location', 'best');

subplot(2, 2, 3);
hold on;
for i = 1:plot_count
    sim = selected.scenario_results(i);
    plot(sim.time, sim.x(3, :), 'LineWidth', 1.2, ...
         'DisplayName', sim.name);
end
grid on;
xlabel('时间 / s');
ylabel('轮轴位置 / m');
title('轮轴水平位置');
legend('Location', 'best');

subplot(2, 2, 4);
hold on;
for i = 1:plot_count
    sim = selected.scenario_results(i);
    plot(sim.time, sim.single_torque, 'LineWidth', 1.2, ...
         'DisplayName', sim.name);
end
yline(cfg.single_torque_limit_nm, '--r', '正力矩限制');
yline(-cfg.single_torque_limit_nm, '--r', '负力矩限制');
grid on;
xlabel('时间 / s');
ylabel('单轮力矩 / N*m');
title('单轮力矩命令');
legend('Location', 'best');

%% 10. 保存完整结果，便于下一轮复现和比较

save('wheel_leg_lqr_tuning_zh_result.mat', ...
     'cfg', 'candidate_definitions', 'scenario_definitions', ...
     'selected_candidate_id', 'runtime_reference_K', ...
     'A', 'B_tau', 'Ad', 'Bd', 'R_tau_nominal', ...
     'results', 'summary_table');

fprintf('\n结果已保存到：wheel_leg_lqr_tuning_zh_result.mat\n');
fprintf('请把“候选排名”和“所选候选逐场景结果”完整复制回来。\n');

%% 局部函数

function validate_configuration(cfg, candidates, scenarios)
    if abs(cfg.m_total - cfg.M_body - cfg.m_wheels) > 1.0e-9
        error('质量不一致：m_total 必须等于 M_body+m_wheels。');
    end
    if cfg.control_hz <= 0 || cfg.pitch_rate_filter_hz <= 0
        error('控制频率和角速度滤波频率必须大于零。');
    end
    if cfg.single_torque_limit_nm <= 0 || ...
            cfg.single_torque_resolution_nm <= 0
        error('力矩限制和分辨率必须大于零。');
    end
    if cfg.command_delay_samples < 0 || ...
            fix(cfg.command_delay_samples) ~= cfg.command_delay_samples
        error('command_delay_samples 必须是非负整数。');
    end
    if isempty(candidates) || size(candidates, 2) ~= 6
        error('candidate_definitions 必须有6列。');
    end
    if isempty(scenarios) || size(scenarios, 2) ~= 4
        error('scenario_definitions 必须有4列。');
    end
    numeric_candidates = cell2mat(candidates(:, 2:6));
    if any(numeric_candidates(:) <= 0)
        error('所有 Q 权重和 R_scale 都必须大于零。');
    end
end

function scenario = make_scenario(definition)
    scenario.name = definition{1};
    values = definition{2};
    scenario.x0 = [deg2rad(values(1)); values(2); values(3); values(4)];
    scenario.pitch_bias_rad = deg2rad(definition{3});
    scenario.score_enabled = definition{4};
end

function sim = simulate_firmware_controller(model, cfg, K, scenario)
    time = 0:cfg.Ts:cfg.simulation_time_s;
    sample_count = numel(time);
    x = zeros(4, sample_count);
    x(:, 1) = scenario.x0;

    filtered_rate = zeros(1, sample_count);
    filtered_rate(1) = x(2, 1);
    filter_rc = 1.0 / (2.0 * pi * cfg.pitch_rate_filter_hz);
    alpha_rate = cfg.Ts / (filter_rc + cfg.Ts);

    raw_total_torque = zeros(1, sample_count);
    command_total_torque = zeros(1, sample_count);
    applied_total_torque = zeros(1, sample_count);
    single_torque = zeros(1, sample_count);
    saturated = false(1, sample_count);

    delay_line = zeros(1, cfg.command_delay_samples);
    actuator_torque = 0.0;

    if cfg.actuator_time_constant_s > 0
        actuator_alpha = 1.0 - ...
            exp(-cfg.Ts / cfg.actuator_time_constant_s);
    else
        actuator_alpha = 1.0;
    end

    priority_angle_rad = deg2rad(cfg.posture_priority_angle_deg);

    for k = 1:(sample_count - 1)
        measured_pitch = x(1, k) + scenario.pitch_bias_rad;

        if k > 1
            filtered_rate(k) = filtered_rate(k - 1) + ...
                alpha_rate * (x(2, k) - filtered_rate(k - 1));
        end

        posture_torque = -(K(1) * measured_pitch + ...
                           K(2) * filtered_rate(k));
        travel_torque = -(K(3) * x(3, k) + K(4) * x(4, k));

        % 与当前 lqr_controller.c 的姿态优先逻辑一致。
        if abs(measured_pitch) >= priority_angle_rad && ...
                posture_torque * travel_torque < 0
            travel_torque = 0.0;
        end

        raw_total_torque(k) = posture_torque + travel_torque;
        limited_total = min(max(raw_total_torque(k), ...
                                -cfg.total_torque_limit_nm), ...
                            cfg.total_torque_limit_nm);
        saturated(k) = abs(raw_total_torque(k)) > ...
                       cfg.total_torque_limit_nm;

        single_command = limited_total / 2.0;
        if cfg.use_torque_quantization
            single_command = round(single_command / ...
                cfg.single_torque_resolution_nm) * ...
                cfg.single_torque_resolution_nm;
        end
        single_command = min(max(single_command, ...
            -cfg.single_torque_limit_nm), cfg.single_torque_limit_nm);

        single_torque(k) = single_command;
        command_total_torque(k) = 2.0 * single_command;

        if cfg.command_delay_samples == 0
            delayed_torque = command_total_torque(k);
        else
            delayed_torque = delay_line(1);
            if cfg.command_delay_samples > 1
                delay_line(1:end-1) = delay_line(2:end);
            end
            delay_line(end) = command_total_torque(k);
        end

        actuator_torque = actuator_torque + ...
            actuator_alpha * (delayed_torque - actuator_torque);
        applied_total_torque(k) = actuator_torque;

        x(:, k + 1) = model.Ad * x(:, k) + ...
                      model.Bd * applied_total_torque(k);
    end

    filtered_rate(end) = filtered_rate(end - 1);
    raw_total_torque(end) = raw_total_torque(end - 1);
    command_total_torque(end) = command_total_torque(end - 1);
    applied_total_torque(end) = applied_total_torque(end - 1);
    single_torque(end) = single_torque(end - 1);
    saturated(end) = saturated(end - 1);

    steady_start = max(1, sample_count - ...
        round(cfg.steady_window_s / cfg.Ts));
    steady_indices = steady_start:sample_count;

    sim.name = scenario.name;
    sim.time = time;
    sim.x = x;
    sim.filtered_pitch_rate = filtered_rate;
    sim.raw_total_torque = raw_total_torque;
    sim.command_total_torque = command_total_torque;
    sim.applied_total_torque = applied_total_torque;
    sim.single_torque = single_torque;
    sim.saturated = saturated;
    sim.max_pitch_deg = max(abs(rad2deg(x(1, :))));
    sim.steady_pitch_rms_deg = sqrt(mean(...
        rad2deg(x(1, steady_indices)).^2));
    sim.max_speed_m_s = max(abs(x(4, :)));
    sim.saturation_percent = 100 * nnz(saturated) / sample_count;
    sim.final_pitch_deg = rad2deg(x(1, end));
    sim.final_position_m = x(3, end);
    sim.final_speed_m_s = x(4, end);
end
