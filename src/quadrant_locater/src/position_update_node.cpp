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
const static float S_bias = 0.0155;
const static float S_scaling = 0.0009;
const static float S_arw = 0.0575;
const static float R_covariance = 0.03;

const static float alpha_compl_yaw;


class PositionCalculatorNode
{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Rate loop_rate_;
    std::string frame_id_;

    geometry_msgs::Vector3 linear_acceleration_data;
    geometry_msgs::Quaternion quaternion_data;
    geometry_msgs::Vector3 rate_data;
    geometry_msgs::Vector3 rpy_mag_data;
    geometry_msgs::Vector3 rpy_gyro_data;
    geometry_msgs::Vector3 rpy_accel_data;
    geometry_msgs::Vector3 rpy_accel_data_norm;
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

    double yaw_compl;

    double gyro_z_present;
    double gyro_z_previous;

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

    float delta_x_cap_gyro[3][1];
    float x_cap_gyro[3][1];
    float P_matrix_gyro[3][3];
    float H_matrix_gyro[1][3];

    static const int no_of_samples = 500;

    /* Rate of imu publishing is 50Hz. So delta_t is 0.02sec */
    static const double delta_t = 0.02;

public:

    PositionCalculatorNode()
    {
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

        yaw_compl = 0;

        g_cumm = 0;
        g_data = 0;

        gyro_z_present = 0; gyro_z_previous = 0;

        lin_acc_x_data = 0; lin_acc_y_data = 0; lin_acc_z_data = 0;

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
        rpy_accel_data_norm_pub = nh.advertise<geometry_msgs::Vector3>("bot/rpy_accel_norm", 10);
        bot_odom_pub = nh.advertise<nav_msgs::Odometry>("bot/odom", 10);
        bot_path_pub = nh.advertise<nav_msgs::Path>("bot/path", 10);
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
        rate_data = latest_imu_msg.angular_velocity;

        updateAccel();

        //time_data = latest_imu_msg.header;

        /* roll pitch and yaw from magnetometer data */
        updateRPYMag();

        /* roll pitch and yaw from gyroscope data */
        /* gyro data is taken after the initial estimation of the bias.
           So, for first 500 samples, rpy is not calculated from the gyro data.
           Yaw rate obtained from the gyro data is used to caluclate the yaw from
           gyro by taking the initial value as the yaw from magnetometer after removing bias.  */


        /* roll pitch and yaw from accelerometer data */
        updateRPYAccel();

        if (counter < no_of_samples)
        {
            cummulativeCalc();
            if (counter == no_of_samples-1)
            {
                averageCalc();
            }
            counter++;
        }

        else if (counter == no_of_samples)
        {
            lin_acc_x_net = lin_acc_x_data;
            lin_acc_y_net = lin_acc_y_data;
            lin_acc_z_net = lin_acc_z_data;

            lin_pos_x_next = 0.5 * lin_acc_x_net * pow(delta_t, 2);
            lin_vel_x_next = lin_acc_x_net * delta_t;

            lin_pos_y_next = 0.5 * lin_acc_y_net * pow(delta_t, 2);
            lin_vel_y_next = lin_acc_y_net * delta_t;

            lin_pos_z_next = 0.5 * lin_acc_z_net * pow(delta_t, 2);
            lin_vel_z_next = lin_acc_z_net * delta_t;

            updateRPYWOBias();
            yaw_gyro = rpy_mag_data.z;
            gyro_z_present = rate_data.z;
        }

        else if (counter > no_of_samples)
        {
            updateYawGyro();
            estimateYawError();
            updateYawAfterErrorEstimate();
            estimatePosition();
        }
        /* publish necessary data */
        publishData();
    }

    void updateAccel()
    {
        lin_acc_x_data = linear_acceleration_data.x - lin_acc_x_offset;
        lin_acc_y_data = linear_acceleration_data.y - lin_acc_y_offset;
        lin_acc_z_data = linear_acceleration_data.z - lin_acc_z_offset;
    }

    void updateRPYMag()
    {
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
    }

    void updateRPYAccel()
    {
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
    }

    void cummulativeCalc()
    {
        roll_mag_cumm = roll_mag_cumm + rpy_mag_data.x;
        pitch_mag_cumm = pitch_mag_cumm + rpy_mag_data.y;
        yaw_mag_cumm = yaw_mag_cumm + rpy_mag_data.z;

        roll_accel_cumm = roll_accel_cumm + rpy_accel_data.x;
        pitch_accel_cumm = pitch_accel_cumm + rpy_accel_data.y;
        yaw_accel_cumm = yaw_accel_cumm + rpy_accel_data.z;

        lin_acc_x_cumm = lin_acc_x_cumm + lin_acc_x_data;
        lin_acc_y_cumm = lin_acc_y_cumm + lin_acc_y_data;
        lin_acc_z_cumm = lin_acc_z_cumm + lin_acc_z_data;

        g_cumm = g_cumm + pow((pow(lin_acc_x_data, 2) + pow(lin_acc_y_data, 2) + pow(lin_acc_z_data, 2)), 0.5);
    }

    void averageCalc()
    {
        roll_mag_bias = roll_mag_cumm / no_of_samples;
        pitch_mag_bias = pitch_mag_cumm / no_of_samples;
        yaw_mag_bias = yaw_mag_cumm / no_of_samples;

        roll_accel_bias = roll_accel_cumm / no_of_samples;
        pitch_accel_bias = pitch_accel_cumm / no_of_samples;
        yaw_accel_bias = yaw_accel_cumm / no_of_samples;

        lin_acc_x_offset = lin_acc_x_cumm / no_of_samples;
        lin_acc_y_offset = lin_acc_y_cumm / no_of_samples;
        lin_acc_z_offset = lin_acc_z_cumm / no_of_samples;

        g_data = g_cumm / no_of_samples;
    }

    void updateRPYWOBias()
    {
        rpy_mag_data.x = rpy_mag_data.x - roll_mag_bias;
        rpy_mag_data.y = rpy_mag_data.y - pitch_mag_bias;
        rpy_mag_data.z = rpy_mag_data.z - yaw_mag_bias;

        rpy_accel_data_norm.x = rpy_accel_data.x - roll_accel_bias;
        rpy_accel_data_norm.y = rpy_accel_data.y - pitch_accel_bias;
        rpy_accel_data_norm.z = rpy_accel_data.z - yaw_accel_bias;
    }

    void updateYawGyro()
    {
        gyro_z_previous = gyro_z_present;
        gyro_z_present = rate_data.z;
        delta_yaw_gyro = (gyro_z_present - gyro_z_previous) * delta_t;
        yaw_gyro = yaw_gyro + delta_yaw_gyro;
    }

    void estimateYawError()
    {
      /* Predict Step */
      float phi_k[3][3] = {};
      float phi_k_tr[3][3] = {};

      /* phi_k matrix */
      phi_k[0][0] = 1;
      phi_k[0][1] = delta_yaw_gyro;
      phi_k[0][2] = delta_t;
      phi_k[1][0] = 0; phi_k[1][1] = 1; phi_k[1][2] = 0;
      phi_k[2][0] = 0; phi_k[2][1] = 0; phi_k[2][2] = 1;

      /* phi_k_tr matrix */
      phi_k_tr[0][0] = 1;
      phi_k_tr[1][0] = delta_yaw_gyro;
      phi_k_tr[2][0] = delta_t;
      phi_k_tr[0][1] = 0; phi_k[1][1] = 1; phi_k[2][1] = 0;
      phi_k_tr[0][2] = 0; phi_k[1][2] = 0; phi_k[2][2] = 1;

      /* Q_k matrix */
      float Q_k[3][3] = {};

      Q_k[0][0] = (S_bias*pow(delta_t, 3)/3) + (S_scaling*pow(delta_yaw_gyro, 2)*delta_t)/2 + S_arw*detla_t;
      Q_k[0][1] = (delta_yaw_gyro*S_scaling*delta_t)/2;
      Q_k[0][2] = (S_bias*pow(delta_t, 2))/2;
      Q_k[1][0] = (delta_yaw_gyro*S_scaling*detla_t)/2;
      Q_k[1][1] = S_scaling*delta_t;
      Q_k[1][2] = 0;
      Q_k[2][0] = (S_bias*pow(detla_t, 2))/2;
      Q_k[2][1] = 0;
      Q_k[2][2] = S_bias*delta_t;

      float x_cap_priori[3][1] = {};

      /* Priori State vector prediction */

      x_cap_priori[0][0] = phi_k[0][0]*x_cap_gyro[0][0] + phi_k[0][1]*x_cap_gyro[1][0] + \
                           phi_k[0][2]*x_cap_gyro[2][0] - delta_x_cap_gyro[0][0];
      x_cap_priori[1][0] = phi_k[1][0]*x_cap_gyro[0][0] + phi_k[1][1]*x_cap_gyro[1][0] + \
                           phi_k[1][2]*x_cap_gyro[2][0] - delta_x_cap_gyro[1][0];
      x_cap_priori[2][0] = phi_k[2][0]*x_cap_gyro[0][0] + phi_k[2][1]*x_cap_gyro[1][0] + \
                           phi_k[2][2]*x_cap_gyro[2][0] - delta_x_cap_gyro[2][0];

      float temp_matrix[3][3] = {};
      float P_matrix_priori[3][3] = {};

      /* Priori Covariance matrix calculation */

      for (int i = 0; i < 3; i++)
      {
          for (int j = 0; j < 3; j++)
          {
              temp_matrix[i][j] = phi_k[i][0]*P_matrix_gyro[0][j] + phi_k[i+1][1]*P_matrix_gyro[1][j] + phi_k[i][2]*P_matrix_gyro[2][j];
          }
      }

      for (int i = 0; i < 3; i++)
      {
          for (int j = 0; j < 3; j++)
          {
              P_matrix_priori[i][j] = temp_matrix[i][0]*phi_k_tr[0][j] + temp_matrix[i][1]*phi_k_tr[1][j] + temp_matrix[i][2]*phi_k_tr[2][j] + Q_k[i][j];
          }
      }

      /* Update Step */

      float kalman_gain[3][1] = {};
      float scaling_factor = 0;

      scaling_factor = H_matrix_gyro[0][0] * (H_matrix_gyro[0][0]*P_matrix_priori[0][0] + H_matrix_gyro[0][1]*P_matrix_priori[1][0] + H_matrix_gyro[0][2]*P_matrix_priori[2][0]) \
                     + H_matrix_gyro[0][1] * (H_matrix_gyro[0][0]*P_matrix_priori[0][1] + H_matrix_gyro[0][1]*P_matrix_priori[1][1] + H_matrix_gyro[0][2]*P_matrix_priori[2][1]) \
                     + H_matrix_gyro[0][2] * (H_matrix_gyro[0][0]*P_matrix_priori[0][2] + H_matrix_gyro[0][1]*P_matrix_priori[1][2] + H_matrix_gyro[0][2]*P_matrix_priori[2][2]);
      scaling_factor = scaling_factor + R_covariance;

      /* Calcuation of Kalman Gain */

      for (int i = 0; i < 3; i++)
      {
          kalman_gain[i][0] = (P_matrix_priori[i][0]*H_matrix_gyro[0][0] + P_matrix_priori[i][1]*H_matrix_gyro[0][1] + P_matrix_priori[i][2]*H_matrix_gyro[0][2]) * scaling_factor
      }

      //float z = yaw_gyro - delta_x_cap_gyro[0][0] - yaw_mag_pres.z;
      float z = delta_x_cap_gyro[0][0] + delta_yaw_gyro*delta_x_cap_gyro[1][0] + delta_t*delta_x_cap_gyro[2][0];
      float innovation_temp = H_matrix_gyro[0][0]*x_cap_priori[0][0] + H_matrix_gyro[0][1]*x_cap_priori[1][0] + H_matrix_gyro[0][2]*x_cap_priori[2][0];
      float x_cap_posteriori[3][1] = {};

      /* Posteriori state vector prediction */
      for (int i = 0; i < 3; i++)
      {
          x_cap_posteriori[i][0] = x_cap_priori[i][0] + kalman_gain[i][0] * (z - innovation_temp);
      }

      /* Posteriori covariance matrix calculation. */

      float P_matrix_posteriori[3][3] = {};
      float Left_matrix[3][3] = {};

      for (int i = 0; i < 3; i++)
      {
          for (int j = 0; j < 3; j++)
          {
              Left_matrix[i][j] = (1 - kalman_gain[i][0]*H_matrix_gyro[0][j]);
          }
      }

      float Right_matrix[3][3] = {};

      for (int i = 0; i < 3; i++)
      {
          for (int j = 0; j < 3; j++)
          {
              Right_matrix[i][j] = (1 - H_matrix_gyro[0][i]*kalman_gain[j][0]);
          }
      }

      float a_temp_matrix[3][3] = {};

      for (int i = 0; i < 3; i++)
      {
          for (int j = 0; j < 3; j++)
          {
              a_temp_matrix[i][j] = 0;
              for (k = 0; k < 3; k++)
              {
                  a_temp_matrix[i][j] += Left_matrix[i][k]*P_matrix_priori[k][j];
              }
          }
      }

      float b_temp_matrix[3][3] = {};

      for (int i = 0; i < 3; i++)
      {
          for (int j = 0; j < 3; j++)
          {
              b_temp_matrix[i][j] = 0;
              for (k = 0; k < 3; k++)
              {
                  b_temp_matrix[i][j] += a_temp_matrix[i][k]*Right_matrix[k][j];
              }
          }
      }

      float c_temp_matrix[3][3] = {};

      for (int i = 0; i < 3; i++)
      {
          for (int j = 0; j < 3; j++)
          {
              c_temp_matrix[i][j] = kalman_gain[i][0]*R_covariance*kalman_gain[j][0];
          }
      }

      for (int i = 0; i < 3; i++)
      {
          for (int j = 0; j < 3; j++)
          {
              P_matrix_posteriori[i][j] = b_temp_matrix[i][j] + c_temp_matrix[i][j];
          }
      }

      /* Error updater for next step */

      delta_x_cap_gyro = delta_x_cap_gyro + x_cap_posteriori;

      P_matrix_gyro = P_matrix_posteriori;

      x_cap_gyro = x_cap_posteriori;
    }

    void updateYawAfterErrorEstimate()
    {
        yaw_gyro = yaw_gyro - delta_x_cap_gyro[0][0];

        /* TODO Complimentary filter for gyro yaw */
        //yaw_compl = alpha_compl_yaw * yaw_gyro + (1 - alpha_compl_yaw) * rpy_mag_data.z;
    }

    void estimatePosition()
    {
        float accel_net = pow((pow(lin_acc_x_data, 2) + pow(lin_acc_y_data, 2) + pow(lin_acc_z_data, 2)), 0.5);
        if ((accel_net < g_data + g_thresh) && (accel_net > g_data - g_thresh))
        {
            lin_acc_x_net = lin_acc_x_data * cos(rpy_accel_data.y * M_PI/180) + lin_acc_x_data * cos(yaw_gyro * M_PI/180) \
                                                                       - lin_acc_y_data * sin(yaw_gyro * M_PI/180);

            lin_acc_y_net = lin_acc_y_data * cos(rpy_accel_data.x * M_PI/180) + lin_acc_x_data * sin(yaw_gyro * M_PI/180) \
                                                                       - lin_acc_y_data * cos(yaw_gyro * M_PI/180);

            lin_acc_z_net = lin_acc_x_data * sin(rpy_accel_data.y * M_PI/180) + lin_acc_y_data * sin(rpy_accel_data.x * M_PI/180) \
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
        }
        else
        {
            lin_pos_x_present = lin_pos_x_next;
            lin_pos_y_present = lin_pos_y_next;
            lin_pos_z_present = lin_pos_z_next;

            lin_vel_x_present = lin_vel_x_next;
            lin_vel_y_present = lin_vel_y_next;
            lin_vel_z_present = lin_vel_z_next;
        }

    }

    void publishData()
    {
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

        bot_path_data.header.stamp = ros::Time::now();

        bot_pose.pose.position.x = lin_pos_x_present;
        bot_pose.pose.position.y = lin_pos_y_present;
        bot_pose.pose.position.z = lin_pos_z_present;
        bot_pose.pose.orientation.x = quaternion_data.x;
        bot_pose.pose.orientation.y = quaternion_data.y;
        bot_pose.pose.orientation.z = quaternion_data.z;
        bot_pose.pose.orientation.w = quaternion_data.w;

        bot_path_data.poses.push_back(bot_pose);

        rpy_mag_data_pub.publish(rpy_mag_data);
        rpy_accel_data_pub.publish(rpy_accel_data);
        rpy_accel_data_norm_pub.publish(rpy_accel_data_norm);

        //ROS_INFO("published rpy data");

        linear_position_data_pub.publish(linear_position_data);
        linear_velocity_data_pub.publish(linear_velocity_data);
        linear_acceleration_wobias_pub.publish(linear_acceleration_wobias);

        angular_position_data_pub.publish(angular_position_data);
        angular_velocity_data_pub.publish(angular_velocity_data);

        bot_odom_pub.publish(bot_odom_data);
        bot_path_pub.publish(bot_path_data);
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
    ros::Publisher rpy_accel_data_norm_pub;
    ros::Publisher bot_odom_pub;
    ros::Publisher bot_path_pub;

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
