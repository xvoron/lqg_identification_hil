%% Main file LQG
% Artyom Voronin
% Brno, 2021
clc
clear all
close all

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

%% Open loop dynamic 
% System dynamic

pols = eig(A);
control = rank(ctrb(A, B));
observ= rank(obsv(A, C));
fprintf("Pols: %d\n", pols)
fprintf("Controlability %d\n",  control)
fprintf("Observability %d\n",  observ)

%% Open-Loop Impulse response
%  respone simulation

t = 0:0.1:20;

[y,t] = impulse(sys_ss, t);
plot(t,y)
title('Open-Loop Impulse Response')
xlabel('Time (sec)')
ylabel('Pitch angle theta (rad)')

%% Open-Loop Step response
figure
[y,t] = step(sys_ss,t);

plot(t,y)
title('Open-Loop Step Response')
xlabel('Time (sec)')
ylabel('Pitch angle theta (rad)')



%% Close-loop impulse Response different Q and R matrix
figure
for i=1:50:501  
    Q_ = diag([0, 0, i]);
    R_ = .1;
    K = lqr(A,B,Q_,R_);
    N = rscale(A,B,C,D,K);
    sys_cl = ss(A-B*K,B*N,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
    [y,t] = impulse(sys_cl,t);
    plot(t,y)
    title('Close-Loop Impulse Response for different Q matrices')
    xlabel('Time (sec)')
    ylabel('Pitch angle theta (rad)')
    hold on
end
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
%% Close-loop impulse Response
figure
[y,t] = impulse(sys_cl,t);
plot(t,y)
title('Close-Loop Impuls Response')
xlabel('Time (sec)')
ylabel('Pitch angle theta (rad)')
%% Close-Loop Step Response
figure
[y,t] = step(sys_cl,t);
plot(t,y)
title('Close-Loop Step Response')
xlabel('Time (sec)')
ylabel('Pitch angle theta (rad)')


%% System with noise

Vd = .01*eye(3); 
Vn = 1;
Bf = [B Vd 0*B];
sys_noise = ss(A,Bf,C,[0 0 0 0 Vn]);
sys_full_output = ss(A,Bf,eye(3),zeros(3,size(Bf,2)));

%% Kalman filter
Sw = 1;
Sv = .1;

Kf = (lqr(A',C',Vd,Vn))';
% [kalmf, Kf, P] = kalman(sys_ss, Sw, Sv); %another solution
Akf = A-Kf*C;
Bkf = [B Kf];
Ckf = eye(3);
Dkf = 0*[B Kf];
sys_kf = ss(Akf, Bkf, Ckf, Dkf);


dt = 0.01;
t = 0:dt:50;

%
% Prepaire input with noise
%

u_dist = randn(3,size(t,2));
u_noise = randn(size(t));
u = 0*t;
u(100:200) = 100;
u(1500:1600) = -100;
u_aug = [u; 0.1*eye(3)*u_dist; 1*u_noise];

%
% Simulate and plot
%

figure
[y,t] = lsim(sys_noise,u_aug,t);
plot(t,y);
[x_true,t] = lsim(sys_full_output,u_aug,t);
hold on
plot(t, x_true(:,3), 'k', 'LineWidth' ,2.0)
[x,t] = lsim(sys_kf, [u; y'],t);
plot(t,x(:,3),'r--', 'LineWidth', 2.0)
title('LQG with noise')
xlabel('Time (sec)')
ylabel('Pitch angle theta (rad)')
legend('y','real theta', 'estimated theta')
hold off

figure
plot(t,x_true(:,2),'-', t, x(:,2), '--', 'LineWidth', 2)
title('Real and estimated pitch rate q')
xlabel('Time (sec)')
ylabel('Pitch rate q (rad/s)')
legend('real q', 'estimated q')
