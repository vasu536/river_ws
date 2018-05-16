#include "quadrant_locater/position_calculator_node.hpp"

PositionCalculatorNode::PositionCalculatorNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh), loop_rate_(50)
{
    ROS_INFO("Constructor called");
    imu_msg_received = true;
    mag_msg_received = true;
    mag_sub = nh.subscribe("imu/magnetic_field", 10, &PositionCalculatorNode::mag_callback, this);
    imu_sub = nh.subscribe("imu/imu", 10, &PositionCalculatorNode::imu_callback, this);


    // Setting up ROS topics to publish after calculations
    linear_position_pub = nh.advertise<geometry_msgs::Vector3>("bot/lin_position", 10);
    linear_velocity_pub = nh.advertise<geometry_msgs::Vector3>("bot/lin_velocity", 10);
    angular_position_pub = nh.advertise<std_msgs::Float32>("bot/angular_position", 10);
    angular_velocity_pub = nh.advertise<std_msgs::Float32>("bot/angular_velocity", 10);
    linear_acceleration_wobias_pub = nh.advertise<geometry_msgs::Vector3>("bot/linear_acceleration_wobias", 10);
    linear_acceleration_net_pub = nh.advertise<geometry_msgs::Vector3>("bot/linear_acceleration_net", 10);
    orientation_mag_pub = nh.advertise<geometry_msgs::Vector3>("bot/rpy_mag", 10);
    orientation_gyro_pub = nh.advertise<geometry_msgs::Vector3>("bot/rpy_gyro", 10);
    orientation_accel_pub = nh.advertise<geometry_msgs::Vector3>("bot/rpy_accel", 10);
    orientation_accel_norm_pub = nh.advertise<geometry_msgs::Vector3>("bot/rpy_accel_norm", 10);
    bot_odom_pub = nh.advertise<nav_msgs::Odometry>("bot/odom", 10);
    bot_path_pub = nh.advertise<nav_msgs::Path>("bot/path", 10);

    counter = 0;
    seq_count = 0;

    Hx = 0; Hy = 0; Hz = 0; Hx_prime = 0; Hy_prime = 0;

    g_cumm = 0;
    g_data = 0;

    theta_present = 0;
    theta_next = 0;

    theta_dot_present = 0;
    theta_dot_next = 0;

    H_matrix_gyro[0][0] = 1; H_matrix_gyro[0][1] = 0; H_matrix_gyro[0][2] = 0;

    P_matrix_gyro[0][0] = 1; P_matrix_gyro[0][1] = 0; P_matrix_gyro[0][2] = 0;
    P_matrix_gyro[1][0] = 0; P_matrix_gyro[1][1] = 1; P_matrix_gyro[1][2] = 0;
    P_matrix_gyro[2][0] = 0; P_matrix_gyro[2][1] = 0; P_matrix_gyro[2][2] = 1;

    delta_x_cap_gyro[0][0] = 0; delta_x_cap_gyro[1][0] = 0; delta_x_cap_gyro[2][0] = 0;
    x_cap_gyro[0][0] = 0; x_cap_gyro[1][0] = 0; x_cap_gyro[2][0] = 0;
};

void PositionCalculatorNode::mag_callback(const sensor_msgs::MagneticFieldConstPtr& mag_d)
{
    latest_mag_msg = *mag_d;
    mag_msg_received = true;

    mag_field = latest_mag_msg.magnetic_field;

    Hx = mag_field.x;
    Hy = mag_field.y;
    Hz = mag_field.z;
}

void PositionCalculatorNode::imu_callback(const sensor_msgs::ImuConstPtr& imu_data)
{
    latest_imu_msg = *imu_data;
    imu_msg_received = true;
    //ROS_INFO("Message recieved");
    linear_acceleration = latest_imu_msg.linear_acceleration;
    quaternion = latest_imu_msg.orientation;
    angular_rate = latest_imu_msg.angular_velocity;

    updateAccel();
    /* roll pitch and yaw from magnetometer data */
    updateRPYMag();

    /* roll pitch and yaw from gyroscope data */
    /* gyro data is taken after the initial estimation of the bias.
       So, for first 500 samples, rpy is not calculated from the gyro data.
       Yaw rate obtained from the gyro data is used to caluclate the yaw from
       gyro by taking the initial value as the yaw from magnetometer after removing bias.  */

    /* no_of_samples is the total number of samples collected at the beginning
       with intital pose of IMU while stationary */
    if (counter < no_of_samples)
    {
        cummulativeCalc();
        if (counter == no_of_samples-1)
        {
            averageCalc();
        }
        counter++;

        initPosCal();
    }

    else
    {

        updateRPYAccel();
        //ROS_INFO("lin_pos_x_next is %f", lin_pos_x_next);
        //ROS_INFO("Starting to estimate");
        updateRPYWOBias();
        updateRPYGyro();

        /* roll pitch and yaw from accelerometer data */
        //estimateYawError();
        //updateYawAfterErrorEstimate();
        estimatePosition();

    }
    /* publish necessary data */
    publishData();
}

void PositionCalculatorNode::initPosCal()
{
    lin_acc_net.x = lin_acc.x;
    lin_acc_net.y = lin_acc.y;
    lin_acc_net.z = lin_acc.z;

    /* lin_pos_next.x = 0.5 * lin_acc_net.x * pow(delta_t, 2);
    lin_vel_next.x = lin_acc_net.x * delta_t;

    lin_pos_next.y = 0.5 * lin_acc_net.y * pow(delta_t, 2);
    lin_vel_next.y = lin_acc_net.y * delta_t;

    lin_pos_next.z = 0.5 * lin_acc_net.z * pow(delta_t, 2);
    lin_vel_next.z = lin_acc_net.z * delta_t; */

    gyro_present.x = angular_rate.x;
    gyro_present.y = angular_rate.y;
    gyro_present.z = angular_rate.z;
}

void PositionCalculatorNode::updateAccel()
{
    //ROS_INFO("Entering updateAccel");
    lin_acc.x = linear_acceleration.x - lin_acc_offset.x;
    lin_acc.y = linear_acceleration.y - lin_acc_offset.y;
    lin_acc.z = linear_acceleration.z - lin_acc_offset.z;
}

void PositionCalculatorNode::updateRPYMag()
{
    //ROS_INFO("Entering updateRPYMag");
    tf::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf::Matrix3x3 m(q);
    m.getRPY(orientation_mag.z, orientation_mag.y, orientation_mag.x);

}

void PositionCalculatorNode::updateRPYAccel()
{
    orientation_accel.x = atan(linear_acceleration.x/(pow((pow(linear_acceleration.y, 2) + pow(linear_acceleration.z, 2)), 0.5)));
    orientation_accel.y = atan(linear_acceleration.y/(pow((pow(linear_acceleration.x, 2) + pow(linear_acceleration.z, 2)), 0.5)));

    Hx_prime = Hx * cos(orientation_accel.y) - Hy * sin(orientation_accel.x) - Hz * sin(orientation_accel.y) * cos(orientation_accel.x);
    Hy_prime = Hy * cos(orientation_accel.x) - Hz * sin(orientation_accel.x);

    orientation_accel.z = 2*M_PI - atan2(Hy_prime, Hx_prime);

}

void PositionCalculatorNode::cummulativeCalc()
{
    mag_cumm.x = mag_cumm.x + orientation_mag.x;
    mag_cumm.y = mag_cumm.y + orientation_mag.y;
    mag_cumm.z = mag_cumm.z + orientation_mag.z;

    accel_cumm.x = accel_cumm.x + orientation_accel.x;
    accel_cumm.y = accel_cumm.y + orientation_accel.y;
    accel_cumm.z = accel_cumm.z + orientation_accel.z;

    lin_acc_cumm.x = lin_acc_cumm.x + lin_acc.x;
    lin_acc_cumm.y = lin_acc_cumm.y + lin_acc.y;
    lin_acc_cumm.z = lin_acc_cumm.z + lin_acc.z;

    g_cumm = g_cumm + pow((pow(lin_acc.x, 2) + pow(lin_acc.y, 2) + pow(lin_acc.z, 2)), 0.5);
}

void PositionCalculatorNode::averageCalc()
{
    mag_bias.x = mag_cumm.x / no_of_samples;
    mag_bias.y = mag_cumm.y / no_of_samples;
    mag_bias.z = mag_cumm.z / no_of_samples;

    //ROS_INFO("Mag bias data is %f, %f, %f", roll_mag_bias, pitch_mag_bias, yaw_mag_bias);

    accel_bias.x = accel_cumm.x / no_of_samples;
    accel_bias.y = accel_cumm.y / no_of_samples;
    accel_bias.z = accel_cumm.z / no_of_samples;

    lin_acc_offset.x = lin_acc_cumm.x / no_of_samples;
    lin_acc_offset.y = lin_acc_cumm.y / no_of_samples;
    lin_acc_offset.z = lin_acc_cumm.z / no_of_samples;

    g_data = g_cumm / no_of_samples;
    ROS_INFO("g for vn100 imu is %f", g_data);
}

void PositionCalculatorNode::updateRPYWOBias()
{
    //ROS_INFO("Mag bias data is %f, %f, %f", roll_mag_bias, pitch_mag_bias, yaw_mag_bias);
    orientation_mag.x = orientation_mag.x - mag_bias.x;
    orientation_mag.y = orientation_mag.y - mag_bias.y;
    orientation_mag.z = orientation_mag.z - mag_bias.z;

    orientation_accel_norm.x = orientation_accel.x - accel_bias.x;
    orientation_accel_norm.y = orientation_accel.y - accel_bias.y;
    orientation_accel_norm.z = orientation_accel.z - accel_bias.z;

}

void PositionCalculatorNode::updateRPYGyro()
{
    gyro_previous.x = gyro_present.x;
    gyro_present.x = angular_rate.x;
    delta_gyro.x = gyro_present.x * delta_t;
    orientation_gyro.x += delta_gyro.x;

    gyro_previous.y = gyro_present.y;
    gyro_present.y = angular_rate.y;
    delta_gyro.y = gyro_present.y * delta_t;
    orientation_gyro.y += delta_gyro.y;

    gyro_previous.z = gyro_present.z;
    gyro_present.z = angular_rate.z;
    delta_gyro.z = gyro_present.z * delta_t;
    //ROS_INFO("yaw before is %f", yaw_gyro);
    orientation_gyro.z += delta_gyro.z;
    //ROS_INFO("yaw after is %f", yaw_gyro);

    if (orientation_gyro.x < 0)
    {
        orientation_gyro.x = 2*M_PI + orientation_gyro.x;
    }
    else if (orientation_gyro.x > 2*M_PI)
    {
        orientation_gyro.x = orientation_gyro.x - 2*M_PI;
    }

    if (orientation_gyro.y < 0)
    {
        orientation_gyro.y = 2*M_PI + orientation_gyro.y;
    }
    else if (orientation_gyro.y > 2*M_PI)
    {
        orientation_gyro.y = orientation_gyro.y - 2*M_PI;
    }

    if (orientation_gyro.z < 0)
    {
        orientation_gyro.z = 2*M_PI + orientation_gyro.z;
    }
    else if (orientation_gyro.z > 2*M_PI)
    {
        orientation_gyro.z = orientation_gyro.z - 2*M_PI;
    }
}

void PositionCalculatorNode::estimateYawError()
{
  //ROS_INFO("Delta yaw estimate is %f", delta_x_cap_gyro[0][0]);
  /* Predict Step */
  float phi_k[3][3] = {};
  float phi_k_tr[3][3] = {};

  /* phi_k matrix */
  phi_k[0][0] = 1;
  phi_k[0][1] = delta_gyro.z;
  phi_k[0][2] = delta_t;
  phi_k[1][0] = 0; phi_k[1][1] = 1; phi_k[1][2] = 0;
  phi_k[2][0] = 0; phi_k[2][1] = 0; phi_k[2][2] = 1;

  /* phi_k_tr matrix */
  phi_k_tr[0][0] = 1;
  phi_k_tr[1][0] = delta_gyro.z;
  phi_k_tr[2][0] = delta_t;
  phi_k_tr[0][1] = 0; phi_k[1][1] = 1; phi_k[2][1] = 0;
  phi_k_tr[0][2] = 0; phi_k[1][2] = 0; phi_k[2][2] = 1;

  /* Q_k matrix */
  float Q_k[3][3] = {};

  Q_k[0][0] = (S_bias*pow(delta_t, 3)/3) + (S_scaling*pow(delta_gyro.z, 2)*delta_t)/2 + S_arw*delta_t;
  Q_k[0][1] = (delta_gyro.z*S_scaling*delta_t)/2;
  Q_k[0][2] = (S_bias*pow(delta_t, 2))/2;
  Q_k[1][0] = (delta_gyro.z*S_scaling*delta_t)/2;
  Q_k[1][1] = S_scaling*delta_t;
  Q_k[1][2] = 0;
  Q_k[2][0] = (S_bias*pow(delta_t, 2))/2;
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
          temp_matrix[i][j] = phi_k[i][0]*P_matrix_gyro[0][j] + phi_k[i][1]*P_matrix_gyro[1][j] + phi_k[i][2]*P_matrix_gyro[2][j];
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
  float accel_net = pow((pow(lin_acc_net.x, 2) + pow(lin_acc_net.y, 2) + pow(lin_acc_net.z, 2)), 0.5);
  //ROS_INFO("Net acceleration is %f", accel_net);
  if (accel_net < g_thresh)
  {

      float kalman_gain[3][1] = {};
      float scaling_factor = 0;

      scaling_factor = H_matrix_gyro[0][0] * (H_matrix_gyro[0][0]*P_matrix_priori[0][0] + H_matrix_gyro[0][1]*P_matrix_priori[1][0] + H_matrix_gyro[0][2]*P_matrix_priori[2][0]) \
                     + H_matrix_gyro[0][1] * (H_matrix_gyro[0][0]*P_matrix_priori[0][1] + H_matrix_gyro[0][1]*P_matrix_priori[1][1] + H_matrix_gyro[0][2]*P_matrix_priori[2][1]) \
                     + H_matrix_gyro[0][2] * (H_matrix_gyro[0][0]*P_matrix_priori[0][2] + H_matrix_gyro[0][1]*P_matrix_priori[1][2] + H_matrix_gyro[0][2]*P_matrix_priori[2][2]);
      scaling_factor = scaling_factor + R_covariance;

      /* Calcuation of Kalman Gain */

      for (int i = 0; i < 3; i++)
      {
          kalman_gain[i][0] = (P_matrix_priori[i][0]*H_matrix_gyro[0][0] + P_matrix_priori[i][1]*H_matrix_gyro[0][1] + P_matrix_priori[i][2]*H_matrix_gyro[0][2]) * scaling_factor;
      }

      //float z = yaw_gyro - delta_x_cap_gyro[0][0] - yaw_mag_pres.z;
      float z = delta_x_cap_gyro[0][0] + delta_gyro.z*delta_x_cap_gyro[1][0] + delta_t*delta_x_cap_gyro[2][0] + R_covariance;
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
              for (int k = 0; k < 3; k++)
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
              for (int k = 0; k < 3; k++)
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

      for (int i = 0; i < 3; i++)
      {
          delta_x_cap_gyro[i][0] = delta_x_cap_gyro[i][0] + x_cap_posteriori[i][0];
          x_cap_gyro[i][0] = x_cap_posteriori[i][0];
      }

      for (int i = 0; i < 3; i++)
      {
          for (int j = 0; j < 3; j++)
          {
              P_matrix_gyro[i][j] = P_matrix_posteriori[i][j];
          }
      }
  }

}

void PositionCalculatorNode::updateYawAfterErrorEstimate()
{
    orientation_gyro.z = orientation_gyro.z - delta_x_cap_gyro[0][0];
    //rpy_gyro_data.x = 0;
    //rpy_gyro_data.y = 0;
    if (orientation_gyro.z < 0)
    {
        orientation_gyro.z = 2*M_PI + orientation_gyro.z;
    }
    else if (orientation_gyro.z > 2*M_PI)
    {
        orientation_gyro.z = orientation_gyro.z - 2*M_PI;
    }
    //ROS_INFO("Delta yaw estimate is %f", delta_x_cap_gyro[0][0]);
    //ROS_INFO("yaw after estimate is %f", yaw_gyro);

    /* TODO Complimentary filter for gyro yaw */
    //yaw_compl = alpha_compl_yaw * yaw_gyro + (1 - alpha_compl_yaw) * rpy_mag_data.z;
}

void PositionCalculatorNode::estimatePosition()
{
    //ROS_INFO("Net acceleration and g are %f, %f", accel_net, g_data);
    /* float a_xr = lin_acc.x;
        //ROS_INFO("a_xr = %f", lin_acc_x_data);
    float a_yr = lin_acc.y * cos(orientation_gyro.x) + lin_acc.z * sin(orientation_gyro.x);
    float a_zr = lin_acc.z * cos(orientation_gyro.x) - lin_acc.y * sin(orientation_gyro.x);

    float a_xrp = a_xr * cos(orientation_gyro.y) + a_zr * sin(orientation_gyro.y);
    float a_yrp = a_yr;
    float a_zrp = a_zr * cos(orientation_gyro.y) - a_xr * sin(orientation_gyro.y);

    lin_acc_net.x = a_xrp * cos(orientation_gyro.z) + a_yrp * sin(orientation_gyro.z);
    lin_acc_net.y = a_yrp * cos(orientation_gyro.z) - a_xrp * sin(orientation_gyro.z);
    lin_acc_net.z = 0; */

    float accel_net = pow((pow(linear_acceleration.x, 2) + pow(linear_acceleration.y, 2) + pow(linear_acceleration.z, 2)), 0.5);
    if (std::abs(accel_net - g_data) <  g_thresh)
    {
        lin_acc.x = 0; lin_acc.y = 0; lin_acc.z = 0;
    }

    tf::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf::Vector3 lin_acc_tf(lin_acc.x, lin_acc.y, lin_acc.z);
    tf::Vector3 lin_acc_net_tf = tf::quatRotate(q, lin_acc_tf);


    lin_acc_net.x = lin_acc_net_tf.getX();
    lin_acc_net.y = lin_acc_net_tf.getY();
    lin_acc_net.z = lin_acc_net_tf.getZ();
    ROS_INFO("net lin acc are %f, %f, %f", lin_acc_net.x, lin_acc_net.y, lin_acc_net.z);
    //float accel_net = pow((pow(lin_acc_net.x, 2) + pow(lin_acc_net.y, 2) + pow(lin_acc_net.z, 2)), 0.5);

    //ROS_INFO("Net acceleration is %f", accel_net - g_data);
    if (std::abs(accel_net - g_data) >  g_thresh)
    {
        //ROS_INFO("Gyro yaw is %f", yaw_gyro);
        //ROS_INFO("Updating postion");
        lin_pos_present = lin_pos_next;
        lin_vel_present = lin_vel_next;

        lin_vel_next.x = lin_vel_present.x + lin_acc_net.x * delta_t;
        lin_vel_next.y = lin_vel_present.y + lin_acc_net.y * delta_t;
        lin_vel_next.z = 0; //lin_vel_present.z + lin_acc_net.z * delta_t;

        lin_pos_next.x = lin_pos_present.x + lin_vel_present.x * delta_t + 0.5 * lin_acc_net.x * pow(delta_t, 2);
        lin_pos_next.y = lin_pos_present.y + lin_vel_present.y * delta_t + 0.5 * lin_acc_net.y * pow(delta_t, 2);
        lin_pos_next.z = 0; //lin_pos_present.z + lin_vel_present.z * delta_t + 0.5 * lin_acc_net.z * pow(delta_t, 2);
        //ROS_INFO("lin_pos_x_next is %f", lin_pos_x_next);
    }
    else
    {
        //ROS_INFO("Not updating position");
        //ROS_INFO("lin_pos_x_next is %f", lin_pos_x_next);
        lin_pos_present = lin_pos_next;
        lin_vel_present = lin_vel_next;
    }
}

void PositionCalculatorNode::publishData()
{
    geometry_msgs::Point linear_position;
    geometry_msgs::Vector3 linear_velocity;
    geometry_msgs::Vector3 linear_acceleration_wobias;
    geometry_msgs::PoseStamped bot_pose;

    std_msgs::Float32 angular_position_data;
    std_msgs::Float32 angular_velocity_data;

    linear_position = lin_pos_present;
    linear_velocity = lin_vel_present;
    linear_acceleration_wobias = lin_acc;

    bot_odom.header.stamp = ros::Time::now();
    bot_odom.pose.pose.position = lin_pos_present;
    bot_odom.pose.pose.orientation = quaternion;
    bot_odom.twist.twist.linear = linear_velocity;
    bot_odom.twist.twist.angular = latest_imu_msg.angular_velocity;

    ros::Time now = ros::Time::now();

    bot_path.header.stamp = now;
    bot_path.header.seq = seq_count;
    bot_path.header.frame_id = "bot";

    bot_pose.pose.position = lin_pos_present;
    bot_pose.pose.orientation = quaternion;
    bot_pose.header.stamp = now;
    bot_pose.header.frame_id = "bot";
    bot_pose.header.seq = seq_count;

    bot_path.poses.clear();
    bot_path.poses.push_back(bot_pose);
    //ROS_INFO("bot path size is %d", bot_path_data.poses.size());
    seq_count++;

    orientation_mag_pub.publish(orientation_mag);
    orientation_accel_pub.publish(orientation_accel);
    orientation_accel_norm_pub.publish(orientation_accel_norm);
    orientation_gyro_pub.publish(orientation_gyro);

    //ROS_INFO("published rpy data");

    linear_position_pub.publish(linear_position);
    linear_velocity_pub.publish(linear_velocity);
    linear_acceleration_wobias_pub.publish(linear_acceleration_wobias);
    linear_acceleration_net_pub.publish(lin_acc_net);

    angular_position_pub.publish(angular_position_data);
    angular_velocity_pub.publish(angular_velocity_data);

    bot_odom_pub.publish(bot_odom);
    bot_path_pub.publish(bot_path);
}

void PositionCalculatorNode::spin()
{
    while (ros::ok())
    {
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "world"));
        transform.setOrigin(tf::Vector3(lin_pos_present.x, lin_pos_present.y, lin_pos_present.z));

        tf::Quaternion q_rot = tf::createQuaternionFromRPY(orientation_gyro.x, orientation_gyro.y, orientation_gyro.z);
        q_rot.normalize();
        transform.setRotation(q_rot);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "bot"));
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


int main(int argc, char *argv[])
{
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
