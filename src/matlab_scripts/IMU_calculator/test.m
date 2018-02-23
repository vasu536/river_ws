clc;
close;
clear all;

bag = rosbag('2018-02-20-12-14-57.bag');

imu_data = select(bag, 'Topic', '/imu/data_raw');
imu_data = readMessages(imu_data);

data= [];

for i = 1:length(imu_data)
    
    %time
    sec = imu_data{i}.Header.Stamp.Sec;
    nsec = imu_data{i}.Header.Stamp.Nsec;
    data(1,i) = sec + (nsec * 10^(-9));

    %accelerations
    data(2,i) = imu_data{i}.LinearAcceleration.X;
    data(3,i) = imu_data{i}.LinearAcceleration.Y;
    data(4,i) = imu_data{i}.LinearAcceleration.Z;
    
    %angular rates
    data(5,i) = imu_data{i}.AngularVelocity.X;
    data(6,i) = imu_data{i}.AngularVelocity.Y;
    data(7,i) = imu_data{i}.AngularVelocity.Z;
    
    
end

initial = zeros(3,3);
find_position(data, initial);