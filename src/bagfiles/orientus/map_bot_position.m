clc;
close all;
clear;
 
str1 = '/home/vasu536/river_ws/src/bagfiles/vectornav/03-14-2018/cart_m_';
str3 = '.bag';

g = 9.80665;

i = 4;


file = strcat(str1, int2str(i), str3);

[x_lin_pos, y_lin_pos, x_lin_vel, y_lin_vel, ... 
          x_lin_acc, y_lin_acc, z_lin_acc, ...
          x_lin_acc_wobias, y_lin_acc_wobias, ...
          gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z] = pos_data_analysis(file);

%figure;
%plot(x_lin_pos, y_lin_pos);
plot(x_lin_acc_wobias);
figure;
plot(y_lin_acc_wobias);

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

set = min(length(yaw_from_gyro), length(yaw_from_mag));
yaw_from_compl_filter = 0.01*yaw_from_gyro(1:set) + 0.99*yaw_from_mag(1:set);

pitch_from_gyro = cumtrapz(data_time, gyro_x);

%%

y_dot_omega = yaw_from_compl_filter(1:set).*y_vel(1:set);

x_dot_omega = yaw_from_compl_filter(1:set).*x_vel(1:set);

x_est = x_disp_full(1:set) - y_dot_omega;

y_est = y_disp_full(1:set) + x_dot_omega;

%%

[rc_x_inst, rc_y_inst] = pol2cart(yaw_from_gyro(1:set), x_disp_full(1:set));

rc_x_cum = cumsum(rc_x_inst);
rc_y_cum = cumsum(rc_y_inst);

figure;
plot(rc_x_cum, rc_y_cum);
%%

figure;
plot(x_lin_vel); hold on; plot(x_vel);
legend('Algorithm', 'Matlab');
title('X velocity');

figure;
plot(y_lin_vel); hold on; plot(y_vel);
legend('Algorithm', 'Matlab');
title('Y velocity');

figure;
plot(x_lin_pos); hold on; plot(x_disp_inst);
legend('Algorithm', 'Matlab');
title('disp in x');

figure;
plot(y_lin_pos); hold on; plot(y_disp_inst);
legend('Algorithm', 'Matlab');
title('disp in y');


%%

x_lin_pos_diff = [0 diff(x_lin_pos)];
y_lin_pos_diff = [0 diff(y_lin_pos)];

min_len = min(length(yaw_from_gyro), length(x_lin_pos_diff));

[x_lin_rc, y_lin_rc] = pol2cart(yaw_from_gyro(1:min_len), x_lin_pos_diff(1:min_len));

x_lin_cum = cumsum(x_lin_rc);
y_lin_cum = cumsum(y_lin_rc);

figure;
plot(x_lin_cum, y_lin_cum);

%%
Fs = 50;
NFFT = length(gyro_z);

fft_gyro_z = fft(gyro_z, NFFT);

F = ((0:1/NFFT:1-1/NFFT)*Fs).';

mag_fft_gyro_z = abs(fft_gyro_z);
phase_fft_gyro_z = unwrap(angle(fft_gyro_z));

helperFrequencyAnalysisPlot1(F, mag_fft_gyro_z, phase_fft_gyro_z, NFFT);

fft_gyro_z_shift = fftshift(fft_gyro_z);
%plot(abs(fft_gyro_z_shift));
