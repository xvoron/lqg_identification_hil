%% HIL onewheel
% Author: Artyom Voronin
clc
clear all
close all

%% Parameters
U = 36;     %[V] Voltage Terminal Voltage
freq = 1e3;  %[Hz] Freq
Ipn = 2;    %[A] Rated Current
Ts = 1e-5; %[s] Sample Time
%Motor parameters
R = 4;      %[Ohm] Resistance
L = 5.5e-3;     %[H] Inductance
J = 1.55e-5;    %[kg.m^2] Inertia

b = 9.55e-5;     %[N*m/(rad/s)] Damping coefficient
K = .03;    %[V/(rad/s)] Constant of Proportionality
Fc = 0.005;


M = 1024;    %Encoder
I_max = 10; % [A] Max current
phi_max = 2;   %[rad] Phi max angle 

phi_p = pi/M;

m = 200 * 1e-3; % [kg] Mass
l = 0.1;    %[m] Length
g = 9.81;
I = m*l^2;

JI = J + I;

A = [ -R/L 0 -K/L;
     0  0  1;
     K/JI -m*g*l/JI -b/JI];
 
B = [1/L; 0; 0];

C = [0 1 0];

D = [0];

sys = ss(A, B, C, D);
% step(sys)

