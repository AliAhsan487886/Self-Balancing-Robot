%% Lab 14 – Model Validation & Parameter Identification
%  Self-Balancing Robot – UET Lahore Control Systems Lab
%
%  Usage:
%    1. Copy CSV output from Serial Monitor into 'data/impulse_response.csv'
%    2. Fill in your PID gains and initial physical parameters below
%    3. Run script and adjust m, h, J to match experimental response

clear; clc; close all;

%% ── 1. Load Experimental Data ────────────────────────────────────────────────
data = readmatrix('../data/impulse_response.csv');   % columns: time_ms, angle_deg, motor_cmd

t_exp   = data(:,1) / 1000;       % convert ms → seconds
theta_exp = data(:,2);             % tilt angle  (degrees)
t_exp   = t_exp - t_exp(1);       % zero-time at disturbance instant

%% ── 2. Known PID Gains (from config.h) ──────────────────────────────────────
Kp = 25.0;
Ki = 0.5;
Kd = 1.2;

%% ── 3. Physical Parameters – TUNE THESE ─────────────────────────────────────
m  = 0.5;          % total mass [kg]       – measure with scale
h  = 0.12;         % CoM height [m]        – measure mechanically
g  = 9.81;         % gravity [m/s²]

J  = m * h^2;      % moment of inertia approximation [kg·m²]

% Derived model coefficients:  θ'' = a·θ + b·u
a  =  m * g * h / J;
b  =  1 / J;

fprintf('Model coefficients:\n  a = %.4f  (open-loop instability)\n  b = %.4f\n', a, b);

%% ── 4. Closed-Loop Transfer Function ────────────────────────────────────────
%
%  θ'' + (Kd/J)·θ' + ((Kp-mgh)/J)·θ + (Ki/J)·∫θ = 0
%
%  State vector: x = [∫θ, θ, θ']
%  Numerator / Denominator of closed-loop TF (from Laplace):
%
%  Denominator: s^3 + (Kd/J)·s^2 + ((Kp-mgh)/J)·s + Ki/J

den = [1, Kd/J, (Kp - m*g*h)/J, Ki/J];
num = [b];           % impulse input gain (simplified)

% Pad numerator to match denominator length
num_padded = [zeros(1, length(den)-length(num)), num];

sys_cl = tf(num_padded, den);

fprintf('\nClosed-Loop Poles:\n');  disp(pole(sys_cl));

%% ── 5. Simulate Impulse Response ────────────────────────────────────────────
t_sim = linspace(0, max(t_exp), 2000);
[theta_sim, t_sim] = impulse(sys_cl, t_sim);
theta_sim = theta_sim * RAD2DEG;   % convert to degrees for comparison

%% ── 6. Compare & Plot ───────────────────────────────────────────────────────
figure('Name','Lab 14 – Model Validation','NumberTitle','off');

subplot(2,1,1);
plot(t_exp, theta_exp, 'b-', 'LineWidth', 1.5); hold on;
plot(t_sim, theta_sim, 'r--','LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Tilt Angle (°)');
title('Impulse Response: Experimental vs Simulated');
legend('Experimental','Simulated (model)');
grid on;

subplot(2,1,2);
% Interpolate simulation onto experimental time grid for error
theta_sim_interp = interp1(t_sim, theta_sim, t_exp, 'linear', 'extrap');
error_vec = theta_exp - theta_sim_interp;
plot(t_exp, error_vec, 'k-','LineWidth',1.2);
xlabel('Time (s)'); ylabel('Error (°)');
title('Model Error');
grid on;

%% ── 7. Quantitative Metrics ─────────────────────────────────────────────────
rmse = sqrt(mean(error_vec.^2));
fprintf('\nRMSE between experimental and simulated: %.4f degrees\n', rmse);

if rmse <= 1.0
    fprintf('✓ RMSE ≤ 1° – Model validation PASSED\n');
else
    fprintf('✗ RMSE > 1° – Adjust m, h, or J and re-run\n');
end
