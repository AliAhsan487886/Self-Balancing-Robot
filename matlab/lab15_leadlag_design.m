%% Lab 15 – Lead–Lag Controller Design
%  Self-Balancing Robot – UET Lahore Control Systems Lab
%
%  Requires: validated model parameters from lab14_model_validation.m

clear; clc; close all;

%% ── 1. Plant Model (from Lab 14 identified parameters) ──────────────────────
% G(s) = b / (s^2 - a)   open-loop inverted pendulum
m  = 0.5;   h = 0.12;   g = 9.81;
J  = m * h^2;
a  = m * g * h / J;
b  = 1 / J;

G = tf([b], [1, 0, -a]);          % unstable second-order plant
fprintf('Open-loop plant G(s) = %.3f / (s^2 - %.3f)\n', b, a);

%% ── 2. Bode Plot – Open Loop ─────────────────────────────────────────────────
figure('Name','Bode Plot – Open Loop');
bode(G); grid on;
title('Open-Loop Bode Plot');
[Gm, Pm, Wcg, Wcp] = margin(G);
fprintf('Open-loop: Gain Margin = %.2f dB, Phase Margin = %.2f°\n', ...
        20*log10(Gm), Pm);

%% ── 3. Design Lead Compensator ──────────────────────────────────────────────
% C_lead(s) = (s + z) / (s + p),  z < p
%
% Objective: add phase margin, increase damping, speed up response.
% Place zero near dominant poles, pole 5–10x further.

z_lead = 5;     % zero location  [rad/s]  – tune this
p_lead = 30;    % pole location  [rad/s]  – tune this

C_lead = tf([1, z_lead], [1, p_lead]);
fprintf('\nLead compensator: zero = %.1f, pole = %.1f rad/s\n', z_lead, p_lead);

%% ── 4. Design Lag Compensator ───────────────────────────────────────────────
% C_lag(s) = (s + z_l) / (s + p_l),  z_l > p_l
%
% Objective: improve steady-state accuracy without hurting PM much.

z_lag = 0.5;   % zero location
p_lag = 0.05;  % pole location (lower for more low-freq gain)

C_lag = tf([1, z_lag], [1, p_lag]);
fprintf('Lag  compensator: zero = %.2f, pole = %.3f rad/s\n', z_lag, p_lag);

%% ── 5. Combined Controller ───────────────────────────────────────────────────
K_gain = 20;    % overall gain – tune to meet bandwidth requirement
C = K_gain * C_lead * C_lag;

L = C * G;      % open-loop with controller

[Gm2, Pm2, Wcg2, Wcp2] = margin(L);
fprintf('\nWith Lead-Lag: Gain Margin = %.2f dB, Phase Margin = %.2f°\n', ...
        20*log10(Gm2), Pm2);

figure('Name','Bode Plot – With Lead-Lag');
bode(L); grid on; hold on;
title('Open-Loop with Lead-Lag Compensator');

%% ── 6. Closed-Loop Simulation ────────────────────────────────────────────────
sys_pid = feedback(K_gain * G, 1);          % approximate PID closed-loop
sys_ll  = feedback(L, 1);                   % lead-lag closed-loop

t = 0:0.001:3;

figure('Name','Step Response Comparison');
[y_pid, ~] = step(sys_pid, t);
[y_ll,  ~] = step(sys_ll,  t);

plot(t, y_pid, 'b--', 'LineWidth', 1.5); hold on;
plot(t, y_ll,  'r-',  'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Angle Response');
title('Step Response: PID vs Lead–Lag');
legend('PID (approx)', 'Lead–Lag');
grid on;

% Compute step-response metrics
info_pid = stepinfo(sys_pid);
info_ll  = stepinfo(sys_ll);

fprintf('\n--- Performance Comparison ---\n');
fprintf('%-20s %-12s %-12s\n', 'Metric', 'PID', 'Lead-Lag');
fprintf('%-20s %-12.3f %-12.3f\n', 'Overshoot (%%)',    info_pid.Overshoot,   info_ll.Overshoot);
fprintf('%-20s %-12.3f %-12.3f\n', 'Settling Time (s)', info_pid.SettlingTime, info_ll.SettlingTime);
fprintf('%-20s %-12.3f %-12.3f\n', 'Rise Time (s)',     info_pid.RiseTime,     info_ll.RiseTime);

%% ── 7. Discretize Controller (Tustin / Bilinear) ────────────────────────────
Ts = 0.005;   % 200 Hz sampling (FAST_LOOP_MS = 5 ms)
C_d = c2d(C, Ts, 'tustin');

fprintf('\nDiscrete Lead-Lag Controller (Ts = %.3f s):\n', Ts);
disp(C_d);

[num_d, den_d] = tfdata(C_d, 'v');
fprintf('\nDifference equation coefficients:\n');
fprintf('  Numerator  b = '); disp(num_d);
fprintf('  Denominator a = '); disp(den_d);
fprintf('\nEmbedded implementation (paste into ESP32 code):\n');
fprintf('  u[k] = %.6f*u[k-1] + %.6f*u[k-2]\n', -den_d(2), -den_d(3));
fprintf('       + %.6f*e[k]   + %.6f*e[k-1] + %.6f*e[k-2];\n', ...
        num_d(1), num_d(2), num_d(3));
