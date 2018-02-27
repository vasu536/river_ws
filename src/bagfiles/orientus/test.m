clc;
close all;
clear;
 
str1 = '/home/vasu536/river_ws/src/bagfiles/orientus/02-26-2018/position_3/tb_s_';
str3 = '.bag';

g = 9.80665;

n = 6; %number of files in folder in order

for i = 1:n
    
    file = strcat(str1, int2str(i), str3);
    
    [x_lin_acc, y_lin_acc] = data_analysis(file);
    
%     figure;
%     plot(x_lin_acc);
%     title('linear acceleration in X direction');
% 
%     figure;
%     plot(y_lin_acc);
%     title('linear acceleration in Y direction');

    x_mean = mean(x_lin_acc);
    y_mean = mean(y_lin_acc);
    
    x_mean_array(i) = x_mean;
    y_mean_array(i) = y_mean;

    x_lin_acc_wobias = x_lin_acc - mean(x_mean);
    y_lin_acc_wobias = y_lin_acc - mean(y_mean);
    
%     figure;   
%     plot(x_lin_acc_wobias);
%     title('linear acceleration in X direction without bias');
% 
%     figure;
%     plot(y_lin_acc_wobias);
%     title('linear acceleration in Y direction without bias');

    std_x_wobias = std(x_lin_acc_wobias);
    std_y_wobias = std(y_lin_acc_wobias);
    
    std_x_wobias_array(i) = std_x_wobias;
    std_y_wobias_array(i) = std_y_wobias;
    
end

figure;
plot(x_mean_array, '*');
title('variation of mean of linear acceleration in x');

figure;
plot(y_mean_array, '*');
title('variation of mean of linear acceleration in y');

range_x_mean = max(x_mean_array) - min(x_mean_array);
range_y_mean = max(y_mean_array) - min(y_mean_array);

theta = rad2deg(-mean(x_mean_array)/g);

