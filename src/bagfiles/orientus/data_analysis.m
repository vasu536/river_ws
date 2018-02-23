function [x_lin_acc, y_lin_acc] = data_analysis(filename)

bag_data = rosbag(filename);

imu_data = select(bag_data, 'Topic', '/imu/data');

imu_data = readMessages(imu_data);

for i = 1:length(imu_data)
    
    x_lin_acc(i) = imu_data{i}.LinearAcceleration.X;
    y_lin_acc(i) = imu_data{i}.LinearAcceleration.Y;

end
