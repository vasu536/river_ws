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
#include "sensor_msgs/MagneticField.h"
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

class PositionCalculatorNode
{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Rate loop_rate_;
    std::string frame_id_;

    geometry_msgs::Vector3 linear_acceleration_data;
    geometry_msgs::Quaternion quaternion_data;
    geometry_msgs::Vector3 rpy_mag_data;
    geometry_msgs::Vector3 rpy_gyro_data;
    geometry_msgs::Vector3 rpy_accel_data;
    nav_msgs::Odometry bot_odom_data;
    nav_msgs::Path bot_path_data;

    //std_msgs::Float32 timestamp;

    //std_msgs::Header time_data;

    sensor_msgs::Imu latest_imu_msg;
    geometry_msgs::Vector3Stamped latest_mag_msg;

    geometry_msgs::Vector3 mag_data;

    bool imu_msg_received;
    bool mag_msg_received;

    /* Statically allocating the offset values which is not a good practice */
    //static const double lin_acc_x_offset = -0.4524;
    //static const double lin_acc_y_offset = 0.0873;

    double lin_acc_x_offset;
    double lin_acc_y_offset;
    double lin_acc_z_offset;

    double lin_acc_x_cumm;
    double lin_acc_y_cumm;
    double lin_acc_z_cumm;
    double g_cumm;

    double roll_mag, pitch_mag, yaw_mag;
    double roll_mag_cumm, pitch_mag_cumm, yaw_mag_cumm;
    double roll_mag_pres, pitch_mag_pres, yaw_mag_pres;
    double roll_mag_bias, pitch_mag_bias, yaw_mag_bias;

    double roll_gyro, pitch_gyro, yaw_gyro;
    double roll_gyro_cumm, pitch_gyro_cumm, yaw_gyro_cumm;
    double roll_gyro_pres, pitch_gyro_pres, yaw_gyro_pres;
    double roll_gyro_bias, pitch_gyro_bias, yaw_gyro_bias;

    double roll_accel, pitch_accel, yaw_accel;
    double roll_accel_cumm, pitch_accel_cumm, yaw_accel_cumm;
    double roll_accel_pres, pitch_accel_pres, yaw_accel_pres;
    double roll_accel_bias, pitch_accel_bias, yaw_accel_bias;

    double Hx, Hy, Hz, Hx_prime, Hy_prime;

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
          mag_msg_received = true;
          mag_sub = nh.subscribe("imu/mag", 100, &PositionCalculatorNode::mag_callback, this);
          imu_sub = nh.subscribe("imu/data", 100, &PositionCalculatorNode::imu_callback, this);

          // Setting up ROS topics to publish after calculations
          linear_position_data_pub = nh.advertise<geometry_msgs::Vector3>("bot/lin_position", 10);
          linear_velocity_data_pub = nh.advertise<geometry_msgs::Vector3>("bot/lin_velocity", 10);
          angular_position_data_pub = nh.advertise<std_msgs::Float32>("bot/angular_position", 10);
          angular_velocity_data_pub = nh.advertise<std_msgs::Float32>("bot/angular_velocity", 10);
          linear_acceleration_wobias_pub = nh.advertise<geometry_msgs::Vector3>("bot/linear_acceleration_wobias", 10);
          rpy_mag_data_pub = nh.advertise<geometry_msgs::Vector3>("bot/rpy_mag", 10);
          rpy_accel_data_pub = nh.advertise<geometry_msgs::Vector3>("bot/rpy_accel", 10);
          bot_odom_pub = nh.advertise<nav_msgs::Odometry>("bot/odom", 10);
          bot_path_pub = nh.advertise<nav_msgs::Path>("bot/path", 10);

          counter = 0;

          roll_mag = 0; pitch_mag = 0; yaw_mag = 0;
          roll_mag_cumm = 0; pitch_mag_cumm = 0; yaw_mag_cumm = 0;
          roll_mag_pres = 0; pitch_mag_pres = 0; yaw_mag_pres = 0;
          roll_mag_bias = 0; pitch_mag_bias = 0; yaw_mag_bias = 0;

          roll_gyro = 0; pitch_gyro = 0; yaw_gyro = 0;
          roll_gyro_cumm = 0; pitch_gyro_cumm = 0; yaw_gyro_cumm = 0;
          roll_gyro_pres = 0; pitch_gyro_pres = 0; yaw_gyro_pres = 0;
          roll_gyro_bias = 0; pitch_gyro_bias = 0; yaw_gyro_bias = 0;

          roll_accel = 0; pitch_accel = 0; yaw_accel = 0;
          roll_accel_cumm = 0; pitch_accel_cumm = 0; yaw_accel_cumm = 0;
          roll_accel_pres = 0; pitch_accel_pres = 0; yaw_accel_pres = 0;
          roll_accel_bias = 0; pitch_accel_bias = 0; yaw_accel_bias = 0;

          Hx = 0; Hy = 0; Hz = 0; Hx_prime = 0; Hy_prime = 0;

          g_cumm = 0;
          g_data = 0;

          lin_acc_x_cumm = 0; lin_acc_y_cumm = 0; lin_acc_z_cumm = 0;

          lin_acc_x_net = 0; lin_acc_y_net = 0; lin_acc_z_net = 0;

          lin_acc_x_offset = 0; lin_acc_y_offset = 0; lin_acc_z_offset = 0;

          lin_pos_x_present = 0; lin_pos_y_present = 0; lin_pos_z_present = 0;

          lin_vel_x_present = 0; lin_vel_y_present = 0; lin_vel_z_present = 0;

          theta_present = 0;
          theta_dot_present = 0;

          lin_pos_x_next = 0; lin_pos_y_next = 0; lin_pos_z_next = 0;

          lin_vel_x_next = 0; lin_vel_y_next = 0; lin_vel_z_next = 0;

          theta_next = 0;
          theta_dot_next = 0;
      }

      void mag_callback(const geometry_msgs::Vector3StampedConstPtr& mag_data)
      {
        latest_mag_msg = *mag_data;
        mag_msg_received = true;

        mag_data = latest_mag_msg.magnetic_field;

        Hx = mag_data.x;
        Hy = mag_data.y;
        Hz = mag_data.z;
      }

      void imu_callback(const sensor_msgs::ImuConstPtr& imu_data)
      {
          latest_imu_msg = *imu_data;
          imu_msg_received = true;
          //ROS_INFO("Message recieved");
          linear_acceleration_data = latest_imu_msg.linear_acceleration;
          quaternion_data = latest_imu_msg.orientation;

          //time_data = latest_imu_msg.header;

          tf::Quaternion q(quaternion_data.x, quaternion_data.y, quaternion_data.z, quaternion_data.w);
          tf::Matrix3x3 m(q);
          m.getRPY(roll_mag, pitch_mag, yaw_mag);

          /* TO-DO  at present roll pitch and yaw are calculated from magnetometer data
             Need to use gyroscope data and fuse it with the mag data to estimate rpy */
          rpy_mag_data.x = roll_mag * 180 / M_PI;
          if (rpy_mag_data.x < 0)
          {
            rpy_mag_data.x = 360 + rpy_mag_data.x;
          }

          rpy_mag_data.y = pitch_mag * 180 / M_PI;
          if (rpy_mag_data.y < 0)
          {
            rpy_mag_data.y = 360 + rpy_mag_data.y;
          }

          rpy_mag_data.z = yaw_mag * 180 / M_PI;
          if (rpy_mag_data.z < 0)
          {
            rpy_mag_data.z = 360 + rpy_mag_data.z;
          }

          /* roll pitch and yaw from gyroscope data */



          /* roll pitch and yaw from accelerometer data */
          roll_accel = atan(linear_acceleration_data.x/(pow((pow(linear_acceleration_data.y, 2) + pow(linear_position_data.z, 2)), 0.5)));
          pitch_accel = atan(linear_acceleration_data.y/(pow((pow(linear_acceleration_data.x, 2) + pow(linear_position_data.z, 2)), 0.5)));

          Hx_prime = Hx * cos(roll_accel * 180/M_PI) - Hy * sin(pitch_accel * 180/M_PI) \
                                                     - Hz * sin(roll_accel * 180/M_PI) * cos(pitch_accel * 180/M_PI);
          Hy_prime = Hy * cos(pitch_accel * 180/M_PI) - Hz * sin(pitch_accel * 180/M_PI);

          yaw_accel = atan2(Hy_prime, Hx_prime);

          rpy_accel_data.x = roll_accel * 180 / M_PI;
          if (rpy_accel_data.x < 0)
          {
            rpy_accel_data.x = 360 + rpy_accel_data.x;
          }

          rpy_accel_data.y = pitch_accel * 180 / M_PI;
          if (rpy_accel_data.y < 0)
          {
            rpy_accel_data.y = 360 + rpy_accel_data.y;
          }

          rpy_accel_data.z = yaw_accel * 180 / M_PI;
          if (rpy_accel_data.z < 0)
          {
            rpy_accel_data.z = 360 + rpy_accel_data.z;
          }

          lin_acc_x_data = linear_acceleration_data.x - lin_acc_x_offset;
          lin_acc_y_data = linear_acceleration_data.y - lin_acc_y_offset;
          lin_acc_z_data = linear_acceleration_data.z + lin_acc_z_offset;

          publish_data();

      }

      void publish_data()
      {

      }


  void spin()
  {
      while (ros::ok())
      {
          if (mag_msg_received)
          {
              mag_msg_received = false;
          }
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
  ros::Subscriber mag_sub;

  ros::Publisher linear_position_data_pub;
  ros::Publisher linear_velocity_data_pub;
  ros::Publisher angular_position_data_pub;
  ros::Publisher angular_velocity_data_pub;
  ros::Publisher linear_acceleration_wobias_pub;
  ros::Publisher rpy_mag_data_pub;
  ros::Publisher rpy_accel_data_pub;
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
