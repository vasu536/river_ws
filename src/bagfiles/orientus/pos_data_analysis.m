function [x_lin_pos, y_lin_pos, x_lin_vel, y_lin_vel, ... 
          x_lin_acc, y_lin_acc, z_lin_acc, ...
          x_lin_acc_wobias, y_lin_acc_wobias, ...
          gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z] = pos_data_analysis(filename)

bag_data = rosbag(filename);

lin_acc_data_wobias = select(bag_data, 'Topic', '/bot/linear_acceleration_wobias');
lin_pos_data = select(bag_data, 'Topic', '/bot/lin_position');
lin_vel_data = select(bag_data, 'Topic', '/bot/lin_velocity');
imu_data = select(bag_data, 'Topic', '/imu/imu');
mag_data = select(bag_data, 'Topic', '/imu/magnetic_field');
timestamp = select(bag_data, 'Topic', '/bot/timestamp');

lin_acc_data_wobias = readMessages(lin_acc_data_wobias);
lin_pos_data = readMessages(lin_pos_data);
lin_vel_data = readMessages(lin_vel_data);
imu_data = readMessages(imu_data);
mag_data = readMessages(mag_data);
timestamp = readMessages(timestamp);

for i = 1:length(lin_pos_data)
    
    x_lin_pos(i) = lin_pos_data{i}.X;
    y_lin_pos(i) = lin_pos_data{i}.Y;
    
end

for i = 1:length(lin_vel_data)
    
    x_lin_vel(i) = lin_vel_data{i}.X;
    y_lin_vel(i) = lin_vel_data{i}.Y;

end

for i = 1:length(imu_data)
    
    x_lin_acc(i) = imu_data{i}.LinearAcceleration.X;
    y_lin_acc(i) = imu_data{i}.LinearAcceleration.Y;
    z_lin_acc(i) = imu_data{i}.LinearAcceleration.Z;
    
    gyro_x(i) = imu_data{i}.AngularVelocity.X;
    gyro_y(i) = imu_data{i}.AngularVelocity.Y;
    gyro_z(i) = imu_data{i}.AngularVelocity.Z;
    
end

for i = 1:length(mag_data)
    
    mag_x(i) = mag_data{i}.MagneticField_.X;
    mag_y(i) = mag_data{i}.MagneticField_.Y;
    mag_z(i) = mag_data{i}.MagneticField_.Z;
    
end

for i = 1:length(lin_acc_data_wobias)
    
    x_lin_acc_wobias(i) = lin_acc_data_wobias{i}.X;
    y_lin_acc_wobias(i) = lin_acc_data_wobias{i}.Y;
    
end
    
    