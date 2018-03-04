clc;
close all;
clear;
 
str1 = '/home/vasu536/river_ws/src/bagfiles/orientus/03-02-2018/tb_m_';
str3 = '.bag';

g = 9.80665;

i = 5;


file = strcat(str1, int2str(i), str3);

[x_lin_pos, y_lin_pos, x_lin_vel, y_lin_vel, ... 
          x_lin_acc, y_lin_acc, z_lin_acc, ...
          gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z] = pos_data_analysis(file);

%figure;
%plot(x_lin_pos, y_lin_pos);

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

%%
x = fit_ellipse(mag_x, mag_y);

rot_matrix = [cos(-x.phi) sin(-x.phi); -sin(-x.phi) cos(-x.phi)]; % for soft iron
h = [mag_x - x.X0_in; mag_y - x.Y0_in]; % for hard iron

calib_matrix = rot_matrix*h;

mag_x_corrected = calib_matrix(1,:);

mag_y_corrected = calib_matrix(2,:);

if x.a > x.b
    mag_x_corrected = (x.b/x.a)*mag_x_corrected;
elseif x.a < x.b
    mag_y_corrected = (x.a/x.b)*mag_y_corrected; 
end

yaw_from_mag = atan2(-mag_y_corrected, mag_x_corrected);

yaw_from_gyro = cumtrapz(data_time, gyro_z);

yaw_from_compl_filter = 0.01*yaw_from_gyro + 0.99*yaw_from_mag(1:length(yaw_from_gyro));

%%

y_dot_omega = yaw_from_compl_filter.*y_vel;

x_dot_omega = yaw_from_compl_filter.*x_vel;

x_est = x_disp_full - y_dot_omega;

y_est = y_disp_full + x_dot_omega;

