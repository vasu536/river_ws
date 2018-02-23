clc;
close;
clear all;

[lin_acc_x, lin_acc_y] = data_analysis('2018-02-20-16-59-16.bag');

x_mean = mean(lin_acc_x);
y_mean = mean(lin_acc_y);

plot(lin_acc_x);
title('x acc');
figure;

plot(lin_acc_y);
title('y acc');

x_std = std(lin_acc_x);
y_std = std(lin_acc_y);

x_var = var(lin_acc_x);

gauss = sqrt(x_var)*randn(1,3828);

b(1) = x_mean;
for i = 2:length(lin_acc_x)
    
    b(i) = b(i-1) + gauss(i);

end

figure;
plot(b)

