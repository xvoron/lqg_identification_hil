%% 
%    _     ___   ____ 
%   | |   / _ \ / ___|
%   | |  | | | | |  _ 
%   | |__| |_| | |_| |
%   |_____\__\_\\____|
%                     
%   Artyom Voronin
%   Brno, 2021

clc
clear all
close all
path2fig = "../doc/img/";

Fs = 1e2;
Ts = 1/Fs;

%% Aircraft longitudinal dynamic 
A = [-0.3149 235.8928 0;
     -0.0034 -0.4282 0;
      0      1       0];
B = [-5.5079;
      0.0021;
      0     ];
C = [0 0 1];
D = 0;

states = {'w' 'q' 'theta'};
inputs = {'delta_e'};
outputs = {'theta'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
sys_disc = c2d(sys_ss, Ts);

%% Simulate discrete system with non zero initial conditions
time = 0:Ts:20;
i = 1;

x_disc = zeros(3, length(time));
u = zeros(1, length(time));
y_disc = zeros(1, length(time));
x_disc(:,1) = [0;1;0];

for t=time
    x_disc(:,i+1) = sys_disc.A*x_disc(:,i) + sys_disc.B*u(i);
    y_disc(i) = sys_disc.C*x_disc(:,i);
    i = i + 1;
end


f = figure;  
f.Position = [10 10 1000 300]; 
hold on
plot(time, y_disc', 'LineWidth', 3)
grid on
xlabel("Time (sec)")
ylabel("Pitch angle theta (rad)")
title("System response with non-zero initial conditions x0 = [0;1;0]")
hold off
%%saveas(f, path2fig+"non_zero.png")

%% Open loop system dynamic 

pols = eig(A);
control = rank(ctrb(A, B));
observ= rank(obsv(A, C));
fprintf("Pols: %d\n", pols)
fprintf("Controlability %d\n",  control)
fprintf("Observability %d\n",  observ)
fprintf("System is controllable and observable \n")

%% Open-Loop Impulse response
t = 0:0.1:20;

f = figure;
f.Position = [100 100 1000 500]; 
subplot(2,1,1)
[y,t] = impulse(sys_ss, t);
plot(t,y, 'LineWidth', 3)
title('Open-Loop Impulse Response')
xlabel('Time (sec)')
ylabel('Pitch angle theta (rad)')

% Open-Loop Step response
subplot(2,1,2)
[y,t] = step(sys_ss,t);

plot(t,y, 'LineWidth', 3)
title('Open-Loop Step Response')
xlabel('Time (sec)')
ylabel('Pitch angle theta (rad)')

%%saveas(f, path2fig+"open_loop.png")


%% Close-loop impulse Response different Q and R matrix
f = figure;  
f.Position = [10 10 1000 300]; 
for i=1:50:501  
    Q_ = diag([0, 0, i]);
    R_ = .1;
    K = lqr(A,B,Q_,R_);
    N = rscale(A,B,C,D,K);
    sys_cl = ss(A-B*K,B*N,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
    [y,t] = impulse(sys_cl,t);
    plot(t,y, 'LineWidth', 3)
    title('Close-Loop Impulse Response for different Q matrices')
    xlabel('Time (sec)')
    ylabel('Pitch angle theta (rad)')
    hold on
end
%saveas(f, path2fig+"lqr_diff_Q.png")
%% LQR implementation
% Close loop

% LQR K gain implementation
Q = diag([0, 0, 500]);
R = .1;
K = lqr(A,B,Q,R);
N = rscale(A,B,C,D,K);

Ac = (A-B*K);
Bc = B*N;
Cc = C;
Dc = D;

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
%% Close-loop zero control
sys_disc_cl = c2d(sys_cl, Ts);
time = 0:Ts:20;
i = 1;

x_disc_cl = zeros(3, length(time));
u = zeros(1, length(time));
y_disc_cl = zeros(1, length(time));
x_disc_cl(:,1) = [0;0;1];

for t=time
    x_disc_cl(:,i+1) = sys_disc_cl.A*x_disc_cl(:,i) + sys_disc_cl.B*u(i);
    y_disc_cl(i) = sys_disc_cl.C*x_disc_cl(:,i);
    i = i + 1;
end


f = figure;  
f.Position = [10 10 1000 300]; 
hold on
plot(time, y_disc_cl', 'LineWidth', 3)
grid on
xlabel("Time (sec)")
ylabel("Pitch angle theta (rad)")
title("Close loop system control to zero with non-zero initial conditions x0 = [0;0;1]")
hold off
%saveas(f, path2fig+"control2zero.png")

%% Close-loop impulse Response
f = figure;
f.Position = [100 100 1000 500]; 
subplot(2,1,1)
[y,t] = impulse(sys_cl,t);
plot(t,y, 'LineWidth', 3)
title('Close-Loop Impuls Response')
xlabel('Time (sec)')
ylabel('Pitch angle theta (rad)')
% Close-Loop Step Response
subplot(2,1,2)
[y,t] = step(sys_cl,t);
plot(t,y, 'LineWidth',3)
title('Close-Loop Step Response')
xlabel('Time (sec)')
ylabel('Pitch angle theta (rad)')

%saveas(f, path2fig+"close_loop.png")

%% Limit input in system, saturation
f = @(u, u_min, u_max) min(max(u, u_min), u_max); 
time = 0:Ts:60;
x_disc = zeros(3, length(time));
u = zeros(1, length(time));
y_disc = zeros(1, length(time));

u_min = -1.5;
u_max = 1.5;
r = u;
r(1/Ts:end) = 1;
u_actuator = zeros(1, length(time));
i = 1;
x_disc(:,1) = [0;0;0];
for t=time
    u_actuator(i) = f((r(i)*N - K*x_disc(:,i)), u_min, u_max);
    x_disc(:,i+1) = sys_disc.A*x_disc(:,i) + sys_disc.B*u_actuator(i);
    y_disc(i) = sys_disc.C*x_disc(:,i);
    i = i + 1;
end

fig = figure;
fig.Position = [100 100 1000 500]; 
subplot(2,1,1)
plot(time, u_actuator, 'LineWidth', 3)
xlabel("Time (sec)")
ylabel("Actuating value (rad)")
title("Actuator with [-1.5;1.5] saturation on input")

subplot(2,1,2)
plot(time, y_disc, 'LineWidth', 3)
xlabel("Time (sec)")
ylabel("Theta (rad)")
title("Output with saturated actuator")
%saveas(fig, path2fig+"lqr_saturation.png")

%% Limit input in system, saturation
% different saturation values example
calculate_diff_saturation = 0;
if calculate_diff_saturation
    f = @(u, u_min, u_max) min(max(u, u_min), u_max); 
    fig = figure;
    fig.Position = [100 100 1000 500]; 
    u = zeros(1, length(time));
    r = u;
    r(100:end) = 1;
    for k = 0.5:0.2:1.5
        u_min = -k;
        u_max = k;

        u_actuator = zeros(1, length(time));
        x_disc(:,1) = [0;0;0];
        i = 1;
        for t=time
            u_actuator(i) = f((r(i)*N - K*x_disc(:,i)), u_min, u_max);
            x_disc(:,i+1) = sys_disc.A*x_disc(:,i) + sys_disc.B*u_actuator(i);
            y_disc(i) = sys_disc.C*x_disc(:,i);
            i = i + 1;
        end
        hold on
        subplot(2,1,1)
        plot(time, u_actuator, 'LineWidth', 3)
        xlabel("Time (sec)")
        ylabel("Actuating value elevator deflection (rad)")
        title("Actuator with different saturation values")
        hold on
        subplot(2,1,2)
        plot(time, y_disc, 'LineWidth', 3)
        xlabel("Time (sec)")
        ylabel("Theta (rad)")
        title("Theta for different saturation values")
    end
    hold off
    %%saveas(fig, path2fig+"lqr_saturation_diff.png")
end

%% Kalman Filter implementation
Sw = .01;
Sv = .5;

[kalmf, Kf, P] = kalman(sys_ss, Sw, Sv); 

f = @(u, u_min, u_max) min(max(u, u_min), u_max); 
time = 0:Ts:60;
x_disc = zeros(3, length(time));
u = zeros(1, length(time));
y_disc = zeros(1, length(time));
y_disc_kf = zeros(1, length(time));

a = -Sv;
b = Sv;


u_min = -1.5;
u_max = 1.5;
r = u;
r(1/Ts:end) = 1;
u_actuator = zeros(1, length(time));
i = 1;
x_disc(:,1) = [0;0;0];
x_disc_kf(:,1) = [0;0;0];

for t=time
    u_actuator(i) = f((r(i)*N - K*x_disc_kf(:,i)), u_min, u_max);
    x_disc(:,i+1) = sys_disc.A*x_disc(:,i) + sys_disc.B*u_actuator(i);
    y_disc(i) = sys_disc.C*x_disc(:,i) + a+(b-a)*rand;
    x_disc_kf(:,i+1) = sys_disc.A*x_disc_kf(:,i) + sys_disc.B*u_actuator(i) + Kf*(y_disc(i) - sys_disc.C*x_disc_kf(:, i));
    y_disc_kf(i) = sys_disc.C*x_disc_kf(:,i);
    i = i + 1;
end
%%

fig = figure;
fig.Position = [100 100 1000 600]; 
subplot(3,1,1)
hold on
plot(time,y_disc','r', 'LineWidth', 1.0)
plot(time, x_disc(3,1:end-1), 'b','LineWidth', 3)
plot(time, x_disc_kf(3,1:end-1), 'g', 'LineWidth',3)
title('Measured and estimated pitch angle')
xlabel('Time (sec)')
ylabel('Pitch angle theta (rad)')
legend('measured','true', 'estimated')
hold off
subplot(3,1,2)
hold on
plot(time, x_disc(2,1:end-1), 'b','LineWidth', 3)
plot(time, x_disc_kf(2,1:end-1), 'g', 'LineWidth',3)
title('True and estimated pitch rate q by Kalman filter')
xlabel('Time (sec)')
ylabel('Pitch rate q (rad/s)')
legend('true', 'estimated')
hold off

subplot(3,1,3)
hold on
plot(time, x_disc(1,1:end-1), 'b','LineWidth', 3)
plot(time, x_disc_kf(1,1:end-1), 'g', 'LineWidth',3)
title('True and estimated vertical velocity omega by Kalman filter')
xlabel('Time (sec)')
ylabel('Vertical velocity  omega (rad/s)')
legend('true', 'estimated')
hold off

%%saveas(fig, path2fig+"lqg_saturated_noise.png")
