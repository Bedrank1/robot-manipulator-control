%% Report_3.m  -  First Order System: Stable / Unstable / P-Control (Q1 - Q2)
clear; clc; close all;

%% Q1a) Stable first-order system - position, velocity, acceleration (Figure 1.1)
figure;
SSG = 2;                 % steady state gain
a   = 2;                 % pole (-a)
b   = SSG * a;           % b = 4
tau = 1/a;               % time constant
tfinal = 4*tau;          % 98% settling time
dt  = 0.01;
t   = 0:dt:tfinal;
x0  = 0.2;

x   = x0*exp(-a*t) + (b/a)*(1 - exp(-a*t));   % position
v   = (b - a*x0)*exp(-a*t);                   % velocity
acc = -a*(b - a*x0)*exp(-a*t);                % acceleration

subplot(3,1,1);
plot(t, x, 'LineWidth', 2);
title('Position');
ylabel('Rad');
grid on;

subplot(3,1,2);
plot(t, v, 'LineWidth', 2, 'Color', 'r');
title('Velocity');
ylabel('Rad/s');
grid on;

subplot(3,1,3);
plot(t, acc, 'LineWidth', 2, 'Color', 'b');
title('Acceleration');
ylabel('Rad/s^2');
xlabel('Time (s)');
grid on;

%% Q1b) Dynamic system matrix (3 x N)
a_pole  = 2;
ss_gain = 2;
x_init  = 0.2;
b_param = ss_gain * a_pole;

dt      = input('Enter step size (dt) : ');
T_upper = input('Enter upper time limit: ');
t_vec   = 0:dt:T_upper;

x_vec = x_init*exp(-a_pole*t_vec) + (b_param/a_pole)*(1 - exp(-a_pole*t_vec));

dx     = diff(x_vec);
v_calc = dx / dt;
v      = [v_calc, v_calc(end)];      % padding

dv     = diff(v);
a_calc = dv / dt;
acc    = [a_calc, a_calc(end)];      % padding

System_Matrix = [x_vec; v; acc];
[r, c] = size(System_Matrix);
fprintf('Matrix size: %d x %d\n', r, c);

%% Q1c) Last five columns of the system matrix (steady-state behavior)
Terminal_Matrix = System_Matrix(:, end-4:end);
disp(Terminal_Matrix);

%% Q2a) Unstable system with 10% parameter erosion (Figure 2.a)
figure;
dt      = 0.01;
t_final = 5;
t_vec   = 0:dt:t_final;
a_nom   = -2;
b       = 1;
u       = 1;
x0      = 0.1;
ero_pct  = 10;
ero_time = 5;

x    = zeros(1, length(t_vec));
x(1) = x0;
for k = 1:length(t_vec)-1
    t_now = t_vec(k);
    ero   = (ero_pct/100) * (t_now/ero_time);
    a_t   = a_nom * (1 - ero);
    x(k+1) = x(k) + (-a_t*x(k) + b*u)*dt;
end

plot(t_vec, x, 'r', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Position');
title(['Unstable System Response with ' num2str(ero_pct) '% Parameter Erosion']);
fprintf('Initial Pole a(0): %.2f\n', a_nom);
fprintf('Final Pole a(%.1f): %.2f\n', ero_time, a_t);

%% Q2b) Unstable system with external disturbance - 10% jump at t = 2.5 s (Figure 2.b)
figure;
dt   = 0.01;
tfinal = 5;
t    = 0:dt:tfinal;
x    = zeros(1, length(t));
x(1) = 0.1;
u    = 1;
a    = -2;
b    = 1;
tdist     = 2.5;
jumpratio = 0.10;

for k = 1:length(t)-1
    dx = -a*x(k) + b*u;
    x(k+1) = x(k) + dx*dt;
    if abs(t(k) - tdist) < dt/2
        jumpvalue = x(k+1) * jumpratio;
        x(k+1) = x(k+1) + jumpvalue;
        fprintf('Disturbance Applied at t = %.2f s. State Jumped by %.4f\n', t(k), jumpvalue);
    end
end

plot(t, x, 'b', 'LineWidth', 2);
hold on;
x_at_dist = interp1(t, x, tdist);
plot(tdist, x_at_dist, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position (State x)');
title(['Unstable System Response with External Disturbance t=' num2str(tdist) 's']);
legend('System Response', 'Instant 10% Jump');
grid on;

%% Q2d) P-controller design (tau = 2 s) and simulation (Figure 2.d)
figure;
a0       = -2;
b        = 1;
tau      = 2;
ref      = 1;
ero_pct  = 10;
ero_time = 5;
dist_t   = 4;
jump_p   = 10;
dt       = 0.01;
sim_time = 10;
t        = 0:dt:sim_time;

pole = 1 / tau;
Kp   = pole - a0;                    % Kp = 2.5

x    = zeros(1, length(t));
x(1) = 0;
for k = 1:length(t)-1
    tk = t(k);
    if tk <= ero_time
        erate = ero_pct/100 * (tk / ero_time);
        a = a0 * (1 - erate);
    else
        a = a0 * (1 - ero_pct/100);
    end
    e = ref - x(k);
    u = Kp * e;
    dx = -a*x(k) + b*u;
    x(k+1) = x(k) + dx*dt;
    if abs(tk - dist_t) < dt/2
        jmp = x(k+1) * (jump_p/100);
        if abs(jmp) < 0.1
            jmp = 0.2;
        end
        x(k+1) = x(k+1) + jmp;
    end
end

plot(t, x, 'b', 'LineWidth', 2); hold on;
yline(ref, '--k', 'LineWidth', 1.5);
xline(dist_t, '--r', 'LineWidth', 1.5);
xline(ero_time, '--g', 'LineWidth', 1.5);
title(['Control Simulation (Kp = ' num2str(Kp) ')']);
xlabel('Time (s)');
ylabel('Output');
legend('System', 'Reference', 'Disturbance', 'Erosion End');
grid on;
fprintf('Calculated Kp: %.2f\n', Kp);