clear;
clc;
close all;

%% Fixed-leg wheel-legged robot: corrected 4-state LQR model
%
% State:
%   x = [theta; theta_dot; position; velocity]
%
% Input:
%   tau_total = tau_left + tau_right, N*m
%
% Sign convention:
%   theta > 0 and position > 0 point forward.
%   tau_total > 0 drives the robot forward.
%
% The wheel motors apply both a drive torque to the wheels and an equal,
% opposite reaction torque to the body. The linearized equations are:
%
%   a*theta_ddot + b*position_ddot - M*g*l*theta = -tau_total
%   b*theta_ddot + c*position_ddot             =  tau_total/r
%
% This model is valid only when the legs are rigidly fixed relative to the
% body. If leg angle can move, the system needs at least the 6-state model.

%% 1. Mechanical parameters

m_total = 6.95;       % Complete robot mass, kg
m = 0.19;             % Total mass of both rotating wheel assemblies, kg
M = m_total - m;      % Rigid body mass excluding rotating wheel assemblies

r = 0.03225;          % Effective rolling radius, m
l = 0.073;            % Body COM height above wheel axle, m
I = 0.025;            % Body pitch inertia about its COM, kg*m^2
g = 9.81;             % Gravity, m/s^2

% Total inertia of two solid disks with combined mass m. Replace this with
% a measured/CAD value if available. With m=0.19 kg, this is 9.8806e-5.
Iw = 0.5 * m * r^2;

% Actual current firmware loop rate. Raise both the firmware IMU ODR and
% this value together when moving to 250 Hz or 500 Hz.
control_hz = 100;
Ts = 1 / control_hz;

% JC motor protocol and current firmware limits.
tau_single_limit = 0.22;       % N*m per wheel
tau_total_limit = 2 * tau_single_limit;
tau_single_resolution = 0.01;  % N*m per wheel

fprintf('================ Mechanical parameters ================\n');
fprintf('Total mass:                  %.6f kg\n', m_total);
fprintf('Rigid body mass M:           %.6f kg\n', M);
fprintf('Two-wheel rotating mass m:   %.6f kg\n', m);
fprintf('Mass sum M+m:                %.6f kg\n', M + m);
fprintf('Wheel radius r:              %.8f m\n', r);
fprintf('COM height l:                %.8f m\n', l);
fprintf('Body pitch inertia I:        %.9f kg*m^2\n', I);
fprintf('Two-wheel inertia Iw:        %.9f kg*m^2\n', Iw);
fprintf('Control period Ts:           %.6f s (%d Hz)\n', Ts, control_hz);

if abs(m_total - M - m) > 1.0e-9
    error('Mass consistency check failed.');
end

%% 2. Corrected continuous state-space model

a = I + M * l^2;
b = M * l;
c = M + m + Iw / r^2;
den = a * c - b^2;

if den <= 0
    error('Model denominator is not positive. Check M, m, l, I, Iw, r.');
end

A = [0,                 1, 0, 0;
     c*M*g*l/den,       0, 0, 0;
     0,                 0, 0, 1;
    -b*M*g*l/den,       0, 0, 0];

% Direct input is total motor torque, not an external cart force.
B_tau = [0;
        -(c + b/r)/den;
         0;
         (b + a/r)/den];

% Equivalent force-input form, provided only for comparison with the old
% script. tau_total = force_equivalent * r.
B_force = B_tau * r;

C = eye(4);
D_tau = zeros(4, 1);

fprintf('\n================ Corrected continuous model ================\n');
disp('A =');
disp(A);
disp('B_tau, input tau_left + tau_right in N*m =');
disp(B_tau);
disp('B_force, input tau_total/r in N =');
disp(B_force);

continuous_rank = rank(ctrb(A, B_tau));
fprintf('Continuous controllability rank: %d / 4\n', continuous_rank);
if continuous_rank ~= 4
    error('The continuous model is not controllable.');
end

%% 3. Discretize at the real control period

sys_c = ss(A, B_tau, C, D_tau);
sys_d = c2d(sys_c, Ts, 'zoh');
Ad = sys_d.A;
Bd = sys_d.B;

discrete_rank = rank(ctrb(Ad, Bd));
fprintf('Discrete controllability rank:   %d / 4\n', discrete_rank);
if discrete_rank ~= 4
    error('The discrete model is not controllable.');
end

%% 4. Discrete LQR design

% Keep the original state weights for the first comparison.
Q = diag([300, 20, 1, 5]);

% The old R=0.5 penalized force^2. Since force=tau/r, the equivalent
% penalty for the total-torque input is R_tau=R_force/r^2.
R_force = 0.5;
R_tau = R_force / r^2;

K_tau = dlqr(Ad, Bd, Q, R_tau);
Acl_d = Ad - Bd * K_tau;
poles_d = eig(Acl_d);

% Equivalent gain when the displayed control variable is force in N.
K_force_equivalent = K_tau / r;

fprintf('\n================ Discrete LQR result ================\n');
disp('K_tau, output is total wheel torque in N*m =');
disp(K_tau);
disp('Equivalent K_force, output is tau_total/r in N =');
disp(K_force_equivalent);
disp('Discrete closed-loop poles eig(Ad-Bd*K_tau) =');
disp(poles_d);

if any(abs(poles_d) >= 1)
    error('Discrete closed-loop poles are not all inside the unit circle.');
end

fprintf('\nDirect-total-torque controller values:\n');
fprintf('K1 = %.9ff;\n', K_tau(1));
fprintf('K2 = %.9ff;\n', K_tau(2));
fprintf('K3 = %.9ff;\n', K_tau(3));
fprintf('K4 = %.9ff;\n', K_tau(4));
fprintf('tau_total = -(K1*theta + K2*theta_dot + K3*position + K4*velocity)\n');
fprintf('tau_left = tau_total/2, tau_right = tau_total/2 after direction mapping\n');

% Kept only for comparison with the removed force-interface controller.
fprintf('\nLegacy force-interface comparison values:\n');
fprintf('K1 = %.9ff;\n', K_force_equivalent(1));
fprintf('K2 = %.9ff;\n', K_force_equivalent(2));
fprintf('K3 = %.9ff;\n', K_force_equivalent(3));
fprintf('K4 = %.9ff;\n', K_force_equivalent(4));
fprintf('force_equivalent = -(K1*theta + K2*theta_dot + K3*position + K4*velocity)\n');
fprintf('tau_single = force_equivalent*r/2\n');

%% 5. Realistic sampled simulation

simulation_time = 5.0;
time = 0:Ts:simulation_time;
sample_count = numel(time);

% Start at 2 degrees. A 5-degree test already approaches the measured
% actuator limit and is not appropriate for the first saturated test.
theta_initial_deg = 2.0;
x = zeros(4, sample_count);
x(:, 1) = [deg2rad(theta_initial_deg); 0; 0; 0];

x_reference = zeros(4, 1);
tau_command = zeros(1, sample_count);
tau_applied = zeros(1, sample_count);
tau_single = zeros(1, sample_count);
saturated = false(1, sample_count);

use_torque_quantization = true;
use_one_sample_delay = true;

for k = 1:(sample_count - 1)
    tau_raw = -K_tau * (x(:, k) - x_reference);
    tau_limited = min(max(tau_raw, -tau_total_limit), tau_total_limit);
    saturated(k) = abs(tau_raw) > tau_total_limit;

    single_command = tau_limited / 2;
    if use_torque_quantization
        single_command = round(single_command / tau_single_resolution) * ...
                         tau_single_resolution;
    end
    single_command = min(max(single_command, -tau_single_limit), ...
                         tau_single_limit);

    tau_single(k) = single_command;
    tau_command(k) = 2 * single_command;

    if use_one_sample_delay && k > 1
        tau_applied(k) = tau_command(k - 1);
    elseif ~use_one_sample_delay
        tau_applied(k) = tau_command(k);
    end

    x(:, k + 1) = Ad * x(:, k) + Bd * tau_applied(k);
end

tau_single(end) = tau_single(end - 1);
tau_command(end) = tau_command(end - 1);
tau_applied(end) = tau_applied(end - 1);
saturated(end) = saturated(end - 1);

fprintf('\n================ Saturated simulation ================\n');
fprintf('Initial pitch:                %.3f deg\n', theta_initial_deg);
fprintf('Peak total torque command:    %.6f N*m\n', max(abs(tau_command)));
fprintf('Peak single-wheel torque:     %.6f N*m\n', max(abs(tau_single)));
fprintf('Saturated sample ratio:       %.2f %%\n', ...
        100 * nnz(saturated) / sample_count);
fprintf('Final pitch:                  %.6f deg\n', rad2deg(x(1, end)));
fprintf('Final position:               %.6f m\n', x(3, end));

if max(abs(x(1, :))) > deg2rad(15)
    warning('Pitch exceeded 15 degrees in simulation. Do not deploy this K.');
end
if abs(x(1, end)) > deg2rad(0.5)
    warning('The delayed/quantized simulation did not settle near upright.');
end

%% 6. Plots

figure('Name', 'Corrected fixed-leg LQR simulation');

subplot(3, 2, 1);
plot(time, rad2deg(x(1, :)), 'LineWidth', 1.4);
grid on;
xlabel('Time / s');
ylabel('Pitch / deg');
title('Body pitch');

subplot(3, 2, 2);
plot(time, x(2, :), 'LineWidth', 1.4);
grid on;
xlabel('Time / s');
ylabel('Pitch rate / rad/s');
title('Pitch rate');

subplot(3, 2, 3);
plot(time, x(3, :), 'LineWidth', 1.4);
grid on;
xlabel('Time / s');
ylabel('Position / m');
title('Wheel-axle position');

subplot(3, 2, 4);
plot(time, x(4, :), 'LineWidth', 1.4);
grid on;
xlabel('Time / s');
ylabel('Velocity / m/s');
title('Wheel-axle velocity');

subplot(3, 2, 5);
plot(time, tau_single, 'LineWidth', 1.4);
hold on;
yline(tau_single_limit, '--r');
yline(-tau_single_limit, '--r');
grid on;
xlabel('Time / s');
ylabel('Torque / N*m');
title('Single-wheel torque command');

subplot(3, 2, 6);
stairs(time, saturated, 'LineWidth', 1.2);
grid on;
xlabel('Time / s');
ylabel('Saturated');
ylim([-0.1, 1.1]);
title('Torque saturation');

%% 7. Save result

save('wheel_leg_lqr_corrected_result.mat', ...
     'M', 'm', 'm_total', 'r', 'l', 'I', 'Iw', 'g', ...
     'control_hz', 'Ts', 'A', 'B_tau', 'B_force', 'Ad', 'Bd', ...
     'Q', 'R_tau', 'K_tau', 'K_force_equivalent', 'poles_d', ...
     'tau_single_limit', 'tau_single_resolution');

fprintf('\nSaved: wheel_leg_lqr_corrected_result.mat\n');
