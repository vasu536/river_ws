clc;
close all;
clear;
 
str1 = '/home/vasu536/river_ws/src/bagfiles/orientus_and_vectornav/03-26-2018/cart_m_';
str3 = '.bag';

g = 9.80665;

i = 1;


file = strcat(str1, int2str(i), str3);

[x_lin_pos, y_lin_pos, x_lin_vel, y_lin_vel, ... 
 x_lin_acc_vn, y_lin_acc_vn, z_lin_acc_vn, ...
 x_lin_acc_or, y_lin_acc_or, z_lin_acc_or, ...
 x_lin_acc_wobias, y_lin_acc_wobias, ...
 gyro_x_vn, gyro_y_vn, gyro_z_vn, mag_x_vn, mag_y_vn, mag_z_vn, ...
 gyro_x_or, gyro_y_or, gyro_z_or, mag_x_or, mag_y_or, mag_z_or] = pos_data_analysis(file);

%%

data_time_or = [0 0.02*(1:length(x_lin_acc_or)-1)];

data_time_vn = [0 0.02*(1:length(x_lin_acc_vn)-1)];

pitch_vn = cumtrapz(data_time_vn, gyro_x_vn);

pitch_or = cumtrapz(data_time_or, gyro_x_or);

roll_vn = cumtrapz(data_time_vn, gyro_y_vn);

roll_or = cumtrapz(data_time_or, gyro_y_or);

g_imu = mean(z_lin_acc_or(200:500));

g_comp_in_x_or = -g_imu * sin(pitch_or);

%x_lin_acc_corr_or = x_lin_acc_or - g_comp_in_x_or;

%mean_x_acc_or = mean(x_lin_acc_corr_or(1:500));

%x_lin_acc_wobias_or = x_lin_acc_corr_or - mean_x_acc_or;

mean_x_acc_or = mean(x_lin_acc_or(1:500));
x_lin_acc_wobias_or = x_lin_acc_or - mean_x_acc_or;

plot(data_time_or, x_lin_acc_or); hold on; 
%plot(data_time_or, x_lin_acc_corr_or); hold on;
plot(data_time_or, x_lin_acc_wobias_or);

title('Linear acceleration in x');
%legend('Before pitch correction', 'After pitch correction', 'After removing bias');
grid on;

x_vel_or = cumtrapz(data_time_or, x_lin_acc_wobias_or);

x_disp_or = cumtrapz(data_time_or, x_vel_or);

figure;
plot(x_disp_or);
title('Position in x calculated in matlab');

figure;
plot(x_vel_or);
title('Velocity in x calculated in matlab');

figure;
plot(pitch_or);
title('Pitch in orientus calculated in Matlab');