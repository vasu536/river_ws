function [x_lin_pos, y_lin_pos, x_lin_vel, y_lin_vel, ... 
          x_lin_acc_vn, y_lin_acc_vn, z_lin_acc_vn, ...
          x_lin_acc_or, y_lin_acc_or, z_lin_acc_or, ...
          x_lin_acc_wobias, y_lin_acc_wobias, ...
          gyro_x_vn, gyro_y_vn, gyro_z_vn, mag_x_vn, mag_y_vn, mag_z_vn, ...
          gyro_x_or, gyro_y_or, gyro_z_or, mag_x_or, mag_y_or, mag_z_or] = pos_data_analysis(filename)

bag_data = rosbag(filename);

lin_acc_data_wobias = select(bag_data, 'Topic', '/bot/linear_acceleration_wobias');
lin_pos_data = select(bag_data, 'Topic', '/bot/lin_position');
lin_vel_data = select(bag_data, 'Topic', '/bot/lin_velocity');
imu_data_vectornav = select(bag_data, 'Topic', '/imu/imu');
imu_data_orientus = select(bag_data, 'Topic', '/imu/data');
mag_data_vectornav = select(bag_data, 'Topic', '/imu/magnetic_field');
mag_data_orientus = select(bag_data, 'Topic', '/imu/mag');
timestamp = select(bag_data, 'Topic', '/bot/timestamp');

lin_acc_data_wobias = readMessages(lin_acc_data_wobias);
lin_pos_data = readMessages(lin_pos_data);
lin_vel_data = readMessages(lin_vel_data);
imu_data_vectornav = readMessages(imu_data_vectornav);
imu_data_orientus = readMessages(imu_data_orientus);
mag_data_vectornav = readMessages(mag_data_vectornav);
mag_data_orientus = readMessages(mag_data_orientus);
timestamp = readMessages(timestamp);

for i = 1:length(lin_pos_data)
    
    x_lin_pos(i) = lin_pos_data{i}.X;
    y_lin_pos(i) = lin_pos_data{i}.Y;
    
end

for i = 1:length(lin_vel_data)
    
    x_lin_vel(i) = lin_vel_data{i}.X;
    y_lin_vel(i) = lin_vel_data{i}.Y;

end

for i = 1:length(imu_data_vectornav)
    
    x_lin_acc_vn(i) = imu_data_vectornav{i}.LinearAcceleration.X;
    y_lin_acc_vn(i) = imu_data_vectornav{i}.LinearAcceleration.Y;
    z_lin_acc_vn(i) = imu_data_vectornav{i}.LinearAcceleration.Z;
    
    gyro_x_vn(i) = imu_data_vectornav{i}.AngularVelocity.X;
    gyro_y_vn(i) = imu_data_vectornav{i}.AngularVelocity.Y;
    gyro_z_vn(i) = imu_data_vectornav{i}.AngularVelocity.Z;
    
end

for i = 1:length(imu_data_orientus)
    
    x_lin_acc_or(i) = imu_data_orientus{i}.LinearAcceleration.X;
    y_lin_acc_or(i) = imu_data_orientus{i}.LinearAcceleration.Y;
    z_lin_acc_or(i) = imu_data_orientus{i}.LinearAcceleration.Z;
    
    gyro_x_or(i) = imu_data_orientus{i}.AngularVelocity.X;
    gyro_y_or(i) = imu_data_orientus{i}.AngularVelocity.Y;
    gyro_z_or(i) = imu_data_orientus{i}.AngularVelocity.Z;
    
end

for i = 1:length(mag_data_vectornav)
    
    mag_x_vn(i) = mag_data_vectornav{i}.MagneticField_.X;
    mag_y_vn(i) = mag_data_vectornav{i}.MagneticField_.Y;
    mag_z_vn(i) = mag_data_vectornav{i}.MagneticField_.Z;
    
end

for i = 1:length(mag_data_orientus)
    
    mag_x_or(i) = mag_data_orientus{i}.MagneticField_.X;
    mag_y_or(i) = mag_data_orientus{i}.MagneticField_.Y;
    mag_z_or(i) = mag_data_orientus{i}.MagneticField_.Z;
    
end

for i = 1:length(lin_acc_data_wobias)
    
    x_lin_acc_wobias(i) = lin_acc_data_wobias{i}.X;
    y_lin_acc_wobias(i) = lin_acc_data_wobias{i}.Y;
    
end
    
    