%%Helicopter azimut script
% author: Artyom Voronin
clc
clear all
close all
% Model equation:
% phi_dot_dot = p_0*w_prop^2 + p_1*phi_dot + p_3*phi_dot^2

p_0 = 1;
p_1 = 1;
p_2 = 1;
p_3 = 1;

rho = 1.2;
Cx = 1.28;

load('heli_data.mat');

figure
subplot(2,1,1)
plot(output(:,1),output(:,2))
title('Output data')
xlabel('Time (s)')
ylabel('Phi (rad)')

subplot(2,1,2)
plot(input(:,1),input(:,2))
title('Input data')
xlabel('Time (s)')
ylabel('Omega (rad/s)')

figure
subplot(2,1,1)
plot(output1(:,1),output1(:,2))
title('Output data')
xlabel('Time (s)')
ylabel('Phi (rad)')

subplot(2,1,2)
plot(input1(:,1),input1(:,2))
title('Input data')
xlabel('Time (s)')
ylabel('Omega (rad/s)')

figure
subplot(2,1,1)
plot(output2(:,1),output2(:,2))
title('Output data')
xlabel('Time (s)')
ylabel('Phi (rad)')

subplot(2,1,2)
plot(input2(:,1),input2(:,2))
title('Input data')
xlabel('Time (s)')
ylabel('Omega (rad/s)')