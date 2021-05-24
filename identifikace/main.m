%%Helicopter azimut script
% author: Artyom Voronin
clc
close all
clear all
path2fig = "../doc/img/";
% Model equation:
% phi_dot_dot = p_0*w_prop^2 p_4*w_prop - p_1*phi_dot - p_2*sign(phi) - p_3*phi_dot^2

load("data_random.mat")
load("data_imp.mat")
load("data_step.mat")
load("data_chirp.mat")

rho = 1.2;
Cx = 1.15;

p_0 = 1.1584e-06;
p_1 = 0.077289;
p_2 = 0.0079642;
p_3 = 0.00012029;
p_4 = 0.010321;

p_0 = -2.9998e-06;
p_1 = 0.32977;
p_2 = -0.0054782;
p_3 = -0.0022903;
p_4 = 0.033036;

open("model.slx")
ident("ident_session.sid")
    
f = figure;
f.Position = [10 10 1000 500]; 
title("System response step input")
subplot(2,1,1)
plot(phi_step.Time, phi_step.Data, 'LineWidth', 3)
xlabel("Time (sec)")
ylabel("Phi (rad)")
title("Output phi")
grid on
 
subplot(2,1,2)
plot(u_step.Time, u_step.Data, 'LineWidth', 3)
xlabel("Time (sec)")
ylabel("Omega (rad/s)")
title("Input omega")
grid on
saveas(f, path2fig+"ident_step.png")


f = figure;
f.Position = [10 10 1000 500]; 
title("System response impulse input")
subplot(2,1,1)
plot(phi_imp.Time, phi_imp.Data, 'LineWidth', 3)
xlabel("Time (sec)")
ylabel("Phi (rad)")
title("Output phi")
grid on
 
subplot(2,1,2)
plot(u_imp.Time, u_imp.Data, 'LineWidth', 3)
xlabel("Time (sec)")
ylabel("Omega (rad/s)")
title("Input omega")
grid on
saveas(f, path2fig+"ident_imp.png")

f = figure;
f.Position = [10 10 1000 500]; 
title("System response random input")
subplot(2,1,1)
plot(phi_random.Time, phi_random.Data, 'LineWidth', 3)
xlabel("Time (sec)")
ylabel("Phi (rad)")
title("Output phi")
grid on
 
subplot(2,1,2)
plot(u_random.Time, u_random.Data, 'LineWidth', 3)
xlabel("Time (sec)")
ylabel("Omega (rad/s)")
title("Input omega")
grid on
saveas(f, path2fig+"ident_random.png")

