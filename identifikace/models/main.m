%%Helicopter azimut script
% author: Artyom Voronin
clc
close all
clear all
% Model equation:
% phi_dot_dot = p_0*w_prop^2 + p_1*phi_dot + p_3*phi_dot^2
p_0 = 1;
p_1 = 1;
p_2 = 1;
p_3 = 1;

rho = 1.2;
Cx = 1.15;



% sim('model_data_mesuare')
% 
% input_data_time = ans.input_data.Time;
% input_data = ans.input_data.Data;
% input = [input_data_time, input_data];
% output_data_time = ans.output_data.Time;
% output_data = ans.output_data.Data;
% output = [output_data_time, output_data];
% 
% input_data_time1 = ans.input_data1.Time;
% input_data1 = ans.input_data1.Data;
% input1 = [input_data_time1, input_data1];
% output_data_time1 = ans.output_data1.Time;
% output_data1 = ans.output_data1.Data;
% output1 = [output_data_time1, output_data1];
% 
% input_data_time2 = ans.input_data2.Time;
% input_data2 = ans.input_data2.Data;
% input2 = [input_data_time2, input_data2];
% output_data_time2 = ans.output_data2.Time;
% output_data2 = ans.output_data2.Data;
% output2 = [output_data_time2, output_data2];

%%Helicopter azimut script
% author: Artyom Voronin
% Model equation:
% phi_dot_dot = p_0*w_prop^2 + p_1*phi_dot + p_3*phi_dot^2



% load('heli_data.mat');
% 
% figure
% subplot(2,1,1)
% plot(output(:,1),output(:,2))
% title('Output data')
% xlabel('Time (s)')
% ylabel('Phi (rad)')
% 
% subplot(2,1,2)
% plot(input(:,1),input(:,2))
% title('Input data')
% xlabel('Time (s)')
% ylabel('Omega (rad/s)')
% 
% figure
% subplot(2,1,1)
% plot(output1(:,1),output1(:,2))
% title('Output data')
% xlabel('Time (s)')
% ylabel('Phi (rad)')
% 
% subplot(2,1,2)
% plot(input1(:,1),input1(:,2))
% title('Input data')
% xlabel('Time (s)')
% ylabel('Omega (rad/s)')
% 
% figure
% subplot(2,1,1)
% plot(output2(:,1),output2(:,2))
% title('Output data')
% xlabel('Time (s)')
% ylabel('Phi (rad)')
% 
% subplot(2,1,2)
% plot(input2(:,1),input2(:,2))
% title('Input data')
% xlabel('Time (s)')
% ylabel('Omega (rad/s)')