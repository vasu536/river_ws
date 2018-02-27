#define M_PI           3.14159265358979323846  /* pi */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <exception>

#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"


class PositionCalculatorNode {

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Rate loop_rate_;
    std::string frame_id_;

    geometry_msgs::Vector3 linear_acceleration_data;


    sensor_msgs::Imu latest_imu_msg;
    bool imu_msg_received;

    /* Statically allocating the offset values which is not a good practice */
    //static const double lin_acc_x_offset = -0.4524;
    //static const double lin_acc_y_offset = 0.0873;

    double lin_acc_x_offset;
    double lin_acc_y_offset;

    double lin_acc_x_cumm;
    double lin_acc_y_cumm;

    double lin_acc_x_data;
    double lin_acc_y_data;

    double lin_pos_x_present;
    double lin_pos_y_present;

    double lin_pos_x_next;
    double lin_pos_y_next;

    double lin_vel_x_present;
    double lin_vel_y_present;

    double lin_vel_x_next;
    double lin_vel_y_next;

    float theta_present;
    float theta_next;

    float theta_dot_present;
    float theta_dot_next;

    int counter;

    static const int no_of_samples = 25;

    /* Rate of imu publishing is 50Hz. So delta_t is 0.02sec */
    static const double delta_t = 0.02;

public:

    PositionCalculatorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh), loop_rate_(50)
    {
        imu_msg_received = false;
        imu_sub = nh.subscribe("imu/data", 10, &PositionCalculatorNode::imu_callback, this);

        // Setting up ROS topics to publish after calculations
        linear_position_data_pub = nh.advertise<geometry_msgs::Vector3>("bot/lin_position", 10);
        linear_velocity_data_pub = nh.advertise<geometry_msgs::Vector3>("bot/lin_velocity", 10);
        angular_position_data_pub = nh.advertise<std_msgs::Float32>("bot/angular_position", 10);
        angular_velocity_data_pub = nh.advertise<std_msgs::Float32>("bot/angular_velocity", 10);

        counter = 0;

        lin_acc_x_cumm = 0;
        lin_acc_y_cumm = 0;

        lin_pos_x_present = 0;
        lin_pos_y_present = 0;

        lin_vel_x_present = 0;
        lin_vel_y_present = 0;

        theta_present = 0;
        theta_dot_present = 0;

        lin_pos_x_next = 0;
        lin_pos_y_next = 0;

        lin_vel_x_next = 0;
        lin_vel_y_next = 0;

        theta_next = 0;
        theta_dot_next = 0;
    }

    void imu_callback(const sensor_msgs::ImuConstPtr& imu_data)
    {
        latest_imu_msg = *imu_data;
        imu_msg_received = true;
        linear_acceleration_data = latest_imu_msg.linear_acceleration;

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

        //lin_acc_x_data = linear_acceleration_data.x;

        //std::cout<<"Linear accelerations after removing offset are "<< lin_acc_x_data<< ", "<< lin_acc_y_data<< std::endl;

        /* Step 2 and 3 */
        if (counter < no_of_samples)
        {
            lin_acc_x_cumm = lin_acc_x_cumm + lin_acc_x_data;
            lin_acc_y_cumm = lin_acc_y_cumm + lin_acc_y_data;

            counter++;

            if (counter == no_of_samples)
            {
                lin_acc_x_offset = lin_acc_x_cumm / counter;
                lin_acc_y_offset = lin_acc_y_cumm / counter;
            }
        }

        if (counter == no_of_samples)
        {
            lin_pos_x_next = 0.5 * lin_acc_x_data * pow(delta_t, 2);
            lin_vel_x_next = lin_acc_x_data * delta_t;

            lin_pos_y_next = 0.5 * lin_acc_y_data * pow(delta_t, 2);
            lin_vel_y_next = lin_acc_y_data * delta_t;

            theta_next = lin_pos_y_next / lin_pos_x_next;
            theta_dot_next = theta_next / delta_t;

            counter++;
        }

        else
        {
            lin_pos_x_present = lin_pos_x_next;
            lin_pos_y_present = lin_pos_y_next;

            lin_vel_x_present = lin_vel_x_next;
            lin_vel_y_present = lin_vel_y_next;

            lin_vel_x_next = lin_vel_x_present + lin_acc_x_data * delta_t;
            lin_vel_y_next = lin_vel_y_present + lin_acc_y_data * delta_t;

            lin_pos_x_next = lin_pos_x_present + lin_vel_x_present * delta_t;
            lin_pos_y_next = lin_pos_y_present + lin_vel_y_present * delta_t;

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
        geometry_msgs::Vector3 linear_position_data;
        geometry_msgs::Vector3 linear_velocity_data;

        std_msgs::Float32 angular_position_data;
        std_msgs::Float32 angular_velocity_data;

        linear_position_data.x = lin_pos_x_present;
        linear_position_data.y = lin_pos_y_present;
        linear_position_data.z = 0;

        linear_velocity_data.x = lin_vel_x_present;
        linear_velocity_data.y = lin_vel_y_present;
        linear_velocity_data.z = 0;

        angular_position_data.data = theta_present;
        angular_velocity_data.data = theta_dot_present;

        linear_position_data_pub.publish(linear_position_data);
        linear_velocity_data_pub.publish(linear_velocity_data);

        angular_position_data_pub.publish(angular_position_data);
        angular_velocity_data_pub.publish(angular_velocity_data);
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
