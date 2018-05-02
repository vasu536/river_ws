#define M_PI           3.14159265358979323846  /* pi */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <exception>
#include <vector>

#include "ros/ros.h"
#include <ros/console.h>

#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "time.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

const static float g = 9.81;

class PositionCalculatorNode {

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Rate loop_rate_;
    std::string frame_id_;

    geometry_msgs::Vector3 linear_acceleration_data;
    geometry_msgs::Quaternion quaternion_data;
    geometry_msgs::Vector3 rpy_data;
    nav_msgs::Odometry bot_odom_data;
    nav_msgs::Path bot_path_data;

    std_msgs::Float32 timestamp;

    std_msgs::Header time_data;

    sensor_msgs::Imu latest_imu_msg;
    bool imu_msg_received;

    /* Statically allocating the offset values which is not a good practice */
    //static const double lin_acc_x_offset = -0.4524;
    //static const double lin_acc_y_offset = 0.0873;

    double lin_acc_x_offset;
    double lin_acc_y_offset;

    double lin_acc_x_cumm;
    double lin_acc_y_cumm;
    double g_cumm;

    double roll, pitch, yaw;
    double roll_cumm, pitch_cumm, yaw_cumm;
    double roll_pres, pitch_pres, yaw_pres;
    double roll_bias, pitch_bias, yaw_bias;

    double lin_acc_x_data;
    double lin_acc_y_data;
    double lin_acc_z_data;
    double g_data;

    double lin_acc_x_net;
    double lin_acc_y_net;
    double lin_acc_z_net;

    double lin_pos_x_present;
    double lin_pos_y_present;
    double lin_pos_z_present;

    double lin_pos_x_next;
    double lin_pos_y_next;
    double lin_pos_z_next;

    double lin_vel_x_present;
    double lin_vel_y_present;
    double lin_vel_z_present;

    double lin_vel_x_next;
    double lin_vel_y_next;
    double lin_vel_z_next;

    float theta_present;
    float theta_next;

    float theta_dot_present;
    float theta_dot_next;

    int counter;

    static const int no_of_samples = 500;

    /* Rate of imu publishing is 50Hz. So delta_t is 0.02sec */
    static const double delta_t = 0.02;

public:

    PositionCalculatorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh), loop_rate_(50)
    {
        imu_msg_received = true;
        imu_sub = nh.subscribe("imu/data", 100, &PositionCalculatorNode::imu_callback, this);

        // Setting up ROS topics to publish after calculations
        linear_position_data_pub = nh.advertise<geometry_msgs::Vector3>("bot/lin_position", 10);
        linear_velocity_data_pub = nh.advertise<geometry_msgs::Vector3>("bot/lin_velocity", 10);
        angular_position_data_pub = nh.advertise<std_msgs::Float32>("bot/angular_position", 10);
        angular_velocity_data_pub = nh.advertise<std_msgs::Float32>("bot/angular_velocity", 10);
        linear_acceleration_wobias_pub = nh.advertise<geometry_msgs::Vector3>("bot/linear_acceleration_wobias", 10);
        rpy_data_pub = nh.advertise<geometry_msgs::Vector3>("bot/rpy", 10);
        bot_odom_pub = nh.advertise<nav_msgs::Odometry>("bot/odom", 10);
        bot_path_pub = nh.advertise<nav_msgs::Path>("bot/path", 10);

        //timestamp_pub = nh.advertise<std_msgs::Float32>("bot/timestamp", 10);

        counter = 0;

        roll = 0;
        pitch = 0;
        yaw = 0;

        roll_cumm = 0;
        pitch_cumm = 0;
        yaw_cumm = 0;

        roll_pres = 0;
        pitch_pres = 0;
        yaw_pres = 0;

        roll_bias = 0;
        pitch_bias = 0;
        yaw_bias = 0;

        lin_acc_x_cumm = 0;
        lin_acc_y_cumm = 0;

        g_cumm = 0;
        g_data = 0;

        lin_acc_x_net = 0;
        lin_acc_y_net = 0;
        lin_acc_z_net = 0;

        lin_acc_x_offset = 0;
        lin_acc_y_offset = 0;

        lin_pos_x_present = 0;
        lin_pos_y_present = 0;
        lin_pos_z_present = 0;

        lin_vel_x_present = 0;
        lin_vel_y_present = 0;
        lin_vel_z_present = 0;

        theta_present = 0;
        theta_dot_present = 0;

        lin_pos_x_next = 0;
        lin_pos_y_next = 0;
        lin_pos_z_next = 0;

        lin_vel_x_next = 0;
        lin_vel_y_next = 0;
        lin_vel_z_next = 0;

        theta_next = 0;
        theta_dot_next = 0;
    }

    void imu_callback(const sensor_msgs::ImuConstPtr& imu_data)
    {
        latest_imu_msg = *imu_data;
        imu_msg_received = true;
        //ROS_INFO("Message recieved");
        linear_acceleration_data = latest_imu_msg.linear_acceleration;
        quaternion_data = latest_imu_msg.orientation;

        time_data = latest_imu_msg.header;

        tf::Quaternion q(quaternion_data.x, quaternion_data.y, quaternion_data.z, quaternion_data.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        /* TO-DO  at present roll pitch and yaw are calculated from magnetometer data
           Need to use gyroscope data and fuse it with the mag data to estimate rpy */
        rpy_data.x = roll * 180 / M_PI;
        if (rpy_data.x < 0)
        {
          rpy_data.x = 360 + rpy_data.x;
        }

        rpy_data.y = pitch * 180 / M_PI;
        if (rpy_data.y < 0)
        {
          rpy_data.y = 360 + rpy_data.y;
        }

        rpy_data.z = yaw * 180 / M_PI;
        if (rpy_data.z < 0)
        {
          rpy_data.z = 360 + rpy_data.z;
        }

        //ROS_INFO("time is -> %f", time_data.seq);

        //std::cout << "linear_acceleration data is " << linear_acceleration_data.x << ", " << linear_acceleration_data.y << ", "
        //          << linear_acceleration_data.z << std::endl;
        //std::cout << "Data received" << std::endl;

        /* Now data manipulation
         * Step 1 : Getting rid of the offset while the sensor is in static state
         * Step 2 : Calcualating the position and velocity after receiving each sample which is 50 samples
         *          from the looks of it - Even though the published loop rate is 100 looking at the driver only 50 samples
         *          are being publsihed per second. So I reduced the subscribing looprate to 50. Now, for our calculations
         *          the difference in time between two samples (delta_t) is 0.02 sec
         * Step 3 : Calculation of position and velocity for n=1 assuming the initial position and velocity (at n=0) is 0
         * */

        /* Step 1 */

        lin_acc_x_data = linear_acceleration_data.x - lin_acc_x_offset;
        lin_acc_y_data = linear_acceleration_data.y - lin_acc_y_offset;
        lin_acc_z_data = linear_acceleration_data.z + g_data;

        //ROS_INFO("Linear accelerations are %f, %f", lin_acc_x_data, lin_acc_y_data);
        //ROS_INFO("Linear offset is %f, %f", lin_acc_x_offset, lin_acc_y_offset);
        //ROS_INFO("Linear position is %f, %f", lin_pos_x_present, lin_pos_y_present);

        //lin_acc_x_data = linear_acceleration_data.x;

        //std::cout<<"Linear accelerations after removing offset are "<< lin_acc_x_data<< ", "<< lin_acc_y_data<< std::endl;

        /* Step 2 and 3 */
        if (counter < no_of_samples)
        {
            roll_cumm = roll_cumm + rpy_data.x;
            pitch_cumm = pitch_cumm + rpy_data.y;
            yaw_cumm = yaw_cumm + rpy_data.z;

            lin_acc_x_cumm = lin_acc_x_cumm + lin_acc_x_data;
            lin_acc_y_cumm = lin_acc_y_cumm + lin_acc_y_data;

            g_cumm = g_cumm + pow((pow(lin_acc_x_data, 2) + pow(lin_acc_y_data, 2) + pow(lin_acc_z_data, 2)), 0.5);

            counter++;

            if (counter == no_of_samples - 1)
            {
                roll_bias = (roll_cumm + rpy_data.x) / no_of_samples;
                pitch_bias = (pitch_cumm + rpy_data.y) / no_of_samples;
                yaw_bias = (yaw_cumm + rpy_data.z) / no_of_samples;

                lin_acc_x_offset = (lin_acc_x_cumm + lin_acc_x_data) / no_of_samples;
                lin_acc_y_offset = (lin_acc_y_cumm + lin_acc_y_data) / no_of_samples;
                g_data = (g_cumm + pow((pow(lin_acc_x_data, 2) + pow(lin_acc_y_data, 2) + pow(lin_acc_z_data, 2)), 0.5)) / no_of_samples;

                ROS_INFO("acceleration due to gravity of IMU = %f", g_data);

              //  ROS_INFO("Linear offset is -> x = %f, y = %f", lin_acc_x_offset, lin_acc_y_offset);
            }
        }

        else if (counter == no_of_samples)
        {
            /* Calculate the net acceleration in X and Y direction to get the absolute value of movement
               in X and Y with respect to roll, pitch and yaw. Use those values to calculate the linear position data */
            roll_pres = rpy_data.x - roll_bias;
            pitch_pres = rpy_data.y - pitch_bias;
            yaw_pres = rpy_data.z - yaw_bias;

            lin_acc_x_net = lin_acc_x_data;
            lin_acc_y_net = lin_acc_y_data;
            lin_acc_z_net = lin_acc_z_data;

            lin_pos_x_next = 0.5 * lin_acc_x_net * pow(delta_t, 2);
            lin_vel_x_next = lin_acc_x_net * delta_t;

            lin_pos_y_next = 0.5 * lin_acc_y_net * pow(delta_t, 2);
            lin_vel_y_next = lin_acc_y_net * delta_t;

            lin_pos_z_next = 0.5 * lin_acc_z_net * pow(delta_t, 2);
            lin_vel_z_next = lin_acc_z_net * delta_t;

            theta_next = lin_pos_y_next / lin_pos_x_next;
            theta_dot_next = theta_next / delta_t;

            counter++;
        }

        else if (counter > no_of_samples)
        {
            roll_pres = rpy_data.x - roll_bias;
            pitch_pres = rpy_data.y - pitch_bias;
            yaw_pres = rpy_data.z - yaw_bias;

            lin_acc_x_net = lin_acc_x_data * cos(pitch_pres * M_PI/180) + lin_acc_x_data * cos(yaw_pres * M_PI/180) \
                                                                        - lin_acc_y_data * sin(yaw_pres * M_PI/180);

            lin_acc_y_net = lin_acc_y_data * cos(roll_pres * M_PI/180) + lin_acc_x_data * sin(yaw_pres * M_PI/180) \
                                                                       - lin_acc_y_data * cos(yaw_pres * M_PI/180);

            lin_acc_z_net = lin_acc_x_data * sin(pitch_pres * M_PI/180) + lin_acc_y_data * sin(roll_pres * M_PI/180) \
                                                                        + lin_acc_z_data;


            lin_pos_x_present = lin_pos_x_next;
            lin_pos_y_present = lin_pos_y_next;
            lin_pos_z_present = lin_pos_z_next;

            lin_vel_x_present = lin_vel_x_next;
            lin_vel_y_present = lin_vel_y_next;
            lin_vel_z_present = lin_vel_z_next;

            lin_vel_x_next = lin_vel_x_present + lin_acc_x_net * delta_t;
            lin_vel_y_next = lin_vel_y_present + lin_acc_y_net * delta_t;
            lin_vel_z_next = lin_vel_z_present + lin_acc_z_net * delta_t;

            lin_pos_x_next = lin_pos_x_present + lin_vel_x_present * delta_t + 0.5 * lin_acc_x_net * pow(delta_t, 2);
            lin_pos_y_next = lin_pos_y_present + lin_vel_y_present * delta_t + 0.5 * lin_acc_y_net * pow(delta_t, 2);
            lin_pos_z_next = lin_pos_z_present + lin_vel_z_present * delta_t + 0.5 * lin_acc_z_net * pow(delta_t, 2);

            theta_present = theta_next;
            theta_dot_present = theta_dot_next;

            theta_next = lin_pos_y_next/lin_pos_x_next;
            theta_dot_next = (theta_next-theta_present)/delta_t;

            counter++;
        }

       publish_data();
        /* Publish the linear position, velocity, angle and angular velocity data for calibration */

    }

    void publish_data()
    {
        //ROS_INFO("Entering publish data method");
        geometry_msgs::Vector3 linear_position_data;
        geometry_msgs::Vector3 linear_velocity_data;
        geometry_msgs::Vector3 linear_acceleration_wobias;
        geometry_msgs::PoseStamped bot_pose;

        std_msgs::Float32 angular_position_data;
        std_msgs::Float32 angular_velocity_data;

        linear_position_data.x = lin_pos_x_present;
        linear_position_data.y = lin_pos_y_present;
        linear_position_data.z = lin_pos_x_present;

        /* data */

        linear_velocity_data.x = lin_vel_x_present;
        linear_velocity_data.y = lin_vel_y_present;
        linear_velocity_data.z = lin_vel_z_present;

        linear_acceleration_wobias.x = lin_acc_x_data;
        linear_acceleration_wobias.y = lin_acc_y_data;
        linear_acceleration_wobias.z = lin_acc_z_data;

        angular_position_data.data = theta_present;
        angular_velocity_data.data = theta_dot_present;



        bot_odom_data.header.stamp = ros::Time::now();
        bot_odom_data.pose.pose.position.x = lin_pos_x_present;
        bot_odom_data.pose.pose.position.y = lin_pos_y_present;
        bot_odom_data.pose.pose.position.z = lin_pos_z_present;
        bot_odom_data.pose.pose.orientation.x = quaternion_data.x;
        bot_odom_data.pose.pose.orientation.y = quaternion_data.y;
        bot_odom_data.pose.pose.orientation.z = quaternion_data.z;
        bot_odom_data.pose.pose.orientation.w = quaternion_data.w;
        bot_odom_data.twist.twist.linear = linear_velocity_data;
        bot_odom_data.twist.twist.angular = latest_imu_msg.angular_velocity;

        //ROS_INFO("Started to put path data on bot_path");

        bot_path_data.header.stamp = ros::Time::now();

        bot_pose.pose.position.x = lin_pos_x_present;
        bot_pose.pose.position.y = lin_pos_y_present;
        bot_pose.pose.position.z = lin_pos_z_present;
        bot_pose.pose.orientation.x = quaternion_data.x;
        bot_pose.pose.orientation.y = quaternion_data.y;
        bot_pose.pose.orientation.z = quaternion_data.z;
        bot_pose.pose.orientation.w = quaternion_data.w;

        bot_path_data.poses.push_back(bot_pose);

        //timestamp.data = ros::Time::now().toSec();

        //ROS_INFO("Time stamp is %f", timestamp.data);
        //ROS_INFO("Started publishing");

        rpy_data_pub.publish(rpy_data);

        //ROS_INFO("published rpy data");

        linear_position_data_pub.publish(linear_position_data);
        linear_velocity_data_pub.publish(linear_velocity_data);
        linear_acceleration_wobias_pub.publish(linear_acceleration_wobias);

        angular_position_data_pub.publish(angular_position_data);
        angular_velocity_data_pub.publish(angular_velocity_data);

        bot_odom_pub.publish(bot_odom_data);
        bot_path_pub.publish(bot_path_data);

        //timestamp_pub.publish(timestamp);
    }

    void spin()
    {
        while (ros::ok())
        {
            if (imu_msg_received)
            {
                imu_msg_received = false;
            }

            ros::spinOnce();
            loop_rate_.sleep();
        }
    }

protected:

    ros::Subscriber imu_sub;

    ros::Publisher linear_position_data_pub;
    ros::Publisher linear_velocity_data_pub;
    ros::Publisher angular_position_data_pub;
    ros::Publisher angular_velocity_data_pub;
    ros::Publisher linear_acceleration_wobias_pub;
    ros::Publisher rpy_data_pub;
    ros::Publisher bot_odom_pub;
    ros::Publisher bot_path_pub;
    ros::Publisher timestamp_pub;

};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "position_calculator_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        PositionCalculatorNode node(nh, pnh);
        //node.setZero();
        node.spin();
    }

    catch(std::exception& e)
    {
        ROS_FATAL_STREAM("Exception thrown: " << e.what());
    }

    return 0;
}
