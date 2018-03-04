clc;
close all;
clear;
 
str1 = '/home/vasu536/river_ws/src/bagfiles/orientus/02-28-2018/tb_s_';
str3 = '.bag';

i = 2;

g = 9.80665;

file = strcat(str1, int2str(i), str3);
    
[x_lin_acc, y_lin_acc] = data_analysis(file);

mean_x_acc = mean(x_lin_acc);
mean_y_acc = mean(y_lin_acc);

x_acc_wobias = x_lin_acc - mean_x_acc;
y_acc_wobias = y_lin_acc - mean_y_acc;

x_vel = cumtrapz(x_acc_wobias);
y_vel = cumtrapz(y_acc_wobias);

x_disp_inst = cumtrapz(x_vel);
y_disp_inst = cumtrapz(y_vel);

x_disp_full = [0 diff(x_disp_inst)];
y_disp_full = [0 diff(y_disp_inst)];

figure;
plot(x_disp_full, y_disp_full);



