%% HIL onewheel
% Author: Artyom Voronin
clc
clear all
close all

%% Parameters
U = 48;     %[V] Voltage
freq = 20;  %[Hz] Freq
Ipn = 2;    %[A]
Ts = 1e-4; %[s] Sample Time
%Motor parameters
R = 1.45;      %[Ohm] Resistance
L = 5.4e-3;     %[H] Inductance
J = 21890.7e-6;    %[kg.m^2] Inertia
%J = 1;
b = .071;     %[N*m/(rad/s)] Damping coefficient
K = .573;    %[V/(rad/s)] Constant of Proportionality

M = 250;    %Encoder
U_I_max = 4;   %[V] LEM max Voltage 
phi_max = 10;   %[rad] Phi max angle 

phi_p = 2*pi/M;

I = 1;
l = 1;
B = 0.1;
m = 1;
g = 9.81;

current_max = 5;