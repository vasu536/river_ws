clc;
close all;
clear;
 
str1 = '/home/vasu536/river_ws/src/bagfiles/orientus/03-01-2018/tb_s_';
str3 = '.bag';

g = 9.80665;

i = 1;


file = strcat(str1, int2str(i), str3);

[x_lin_pos, y_lin_pos, x_lin_vel, y_lin_vel, ... 
          x_lin_acc, y_lin_acc, z_lin_acc, ...
          gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, time_data] = pos_data_analysis(file);

figure;
plot(x_lin_pos, y_lin_pos);

mean_x_acc = mean(x_lin_acc(1:500));
mean_y_acc = mean(y_lin_acc(1:500));

x_acc_wobias = x_lin_acc - mean_x_acc;
y_acc_wobias = y_lin_acc - mean_y_acc;

data_time = [0 0.02*(1:length(x_lin_acc)-1)];

x_vel = cumtrapz(data_time, x_acc_wobias);
y_vel = cumtrapz(data_time, y_acc_wobias);

x_disp_inst = cumtrapz(data_time, x_vel);
y_disp_inst = cumtrapz(data_time, y_vel);

x_disp_full = [0 diff(x_disp_inst)];
y_disp_full = [0 diff(y_disp_inst)];

figure;
plot(x_disp_full, y_disp_full);


