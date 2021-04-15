%% HIL onewheel
% Author: Artyom Voronin
clc
clear all
close all
%% Parameters
U = 48;     %[V] Voltage
freq = 20;  %[Hz] Freq
Ipn = 2;    %[A]
Ts = 0.001; %[s] Sample Time
%Motor parameters
R = 1.45;      %[Ohm] Resistance
L = 5.4e-3;     %[H] Inductance
J = 21890.7e-6;    %[kg.m^2] Inertia
%J = 1;
b = .071;     %[N*m/(rad/s)] Damping coefficient
K = .573;    %[V/(rad/s)] Constant of Proportionality

M = 171;    %Encoder
U_I_max = 4;   %[V] LEM max Voltage 
phi_max = 10;   %[rad] Phi max angle 

data = load('dataOneWheel.mat'); %Data from accelerometer

dataX = data.Acceleration.X;
dataY = data.Acceleration.Y;
dataZ = data.Acceleration.Z;
t = 0:0.1:11.1;
t = t';