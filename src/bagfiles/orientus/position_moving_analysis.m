clc;
close all;
clear;
 
str1 = '/home/vasu536/river_ws/src/bagfiles/orientus/02-27-2018/tb_m_';
str3 = '.bag';

g = 9.80665;

n = 4;

for i = n:n
    
    file = strcat(str1, int2str(i), str3);
    
    [x_pos, y_pos, x_vel, y_vel, x_lin_acc, y_lin_acc] = pos_data_analysis(file);
    
    figure;
    plot(x_pos);
    title('Displacement of X over time');
    
    figure;
    plot(y_pos);
    title('Displacement of Y over time');
    
    figure;   
    plot(x_vel, '*');
    title('Velocity in X direction calc');

    figure;
    plot(y_lin_acc, '*');
    title('Velocity in Y direction calc');
    
end

figure;
plot(x_pos, y_pos);
title('Position of moving bot over time');