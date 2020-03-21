/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mellinger_control_node.h"

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h> 
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/crazyflie_complementary_filter.h"

#define ATTITUDE_UPDATE_DT 0.0004  /* ATTITUDE UPDATE RATE [s] - 500Hz */
#define RATE_UPDATE_DT 0.0002      /* RATE UPDATE RATE [s] - 250Hz */
#define SAMPLING_TIME  0.001       /* SAMPLING CONTROLLER TIME [s] - 100Hz */

namespace rotors_control {

MellingerControlNode::MellingerControlNode() {

    ROS_INFO_ONCE("Started position controller");

    InitializeParams();

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,  &MellingerControlNode::MultiDofJointTrajectoryCallback, this);

    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &MellingerControlNode::OdometryCallback, this);

    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    // The subscription to the IMU topic is made only if it is available, when simulating the Crazyflie dynamics considering the also IMU values

    hover_active = nh.subscribe("hover_active",1,&MellingerControlNode::HoverCallback,this);
    path_active = nh.subscribe("path_active",1,&MellingerControlNode::PathCallback,this);

    if(pnh_node.getParam("enable_state_estimator", enable_state_estimator_)){
      ROS_INFO("Got param 'enable_state_estimator': %d", enable_state_estimator_);
    }


    if (enable_state_estimator_){
        imu_sub_ = nh.subscribe(mav_msgs::default_topics::IMU, 1, &MellingerControlNode::IMUCallback, this);

        //Timers allow to set up the working frequency of the control system
        timer_Attitude_ = n_.createTimer(ros::Duration(ATTITUDE_UPDATE_DT), &MellingerControlNode::CallbackAttitudeEstimation, this, false, true);

        timer_highLevelControl = n_.createTimer(ros::Duration(SAMPLING_TIME), &MellingerControlNode::CallbackHightLevelControl, this, false, true);

        timer_IMUUpdate = n_.createTimer(ros::Duration(RATE_UPDATE_DT), &MellingerControlNode::CallbackIMUUpdate, this, false, true);

     }


}

MellingerControlNode::~MellingerControlNode(){}


void MellingerControlNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  // We can trigger the first command immediately.
  mellinger_controller_.SetTrajectoryPoint(eigen_reference);
  commands_.pop_front();

  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO("PositionController got first MultiDOFJointTrajectory message.");
  }
}


void MellingerControlNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Parameters reading from rosparam.
  GetRosParameter(pnh, "hover_stiff_kp/x",
                  mellinger_controller_.controller_parameters_.hover_xyz_stiff_kp_.x(),
                  &mellinger_controller_.controller_parameters_.hover_xyz_stiff_kp_.x());
  GetRosParameter(pnh, "hover_stiff_kp/y",
                  mellinger_controller_.controller_parameters_.hover_xyz_stiff_kp_.y(),
                  &mellinger_controller_.controller_parameters_.hover_xyz_stiff_kp_.y());
  GetRosParameter(pnh, "hover_stiff_kp/z",
                  mellinger_controller_.controller_parameters_.hover_xyz_stiff_kp_.z(),
                  &mellinger_controller_.controller_parameters_.hover_xyz_stiff_kp_.z());
  GetRosParameter(pnh, "hover_stiff_ki/x",
                  mellinger_controller_.controller_parameters_.hover_xyz_stiff_ki_.x(),
                  &mellinger_controller_.controller_parameters_.hover_xyz_stiff_ki_.x());
  GetRosParameter(pnh, "hover_stiff_ki/y",
                    mellinger_controller_.controller_parameters_.hover_xyz_stiff_ki_.y(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_stiff_ki_.y());
  GetRosParameter(pnh, "hover_stiff_ki/z",
                    mellinger_controller_.controller_parameters_.hover_xyz_stiff_ki_.z(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_stiff_ki_.z());
  GetRosParameter(pnh, "hover_stiff_kd/x",
                    mellinger_controller_.controller_parameters_.hover_xyz_stiff_kd_.x(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_stiff_kd_.x());
  GetRosParameter(pnh, "hover_stiff_kd/y",
                    mellinger_controller_.controller_parameters_.hover_xyz_stiff_kd_.y(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_stiff_kd_.y());
  GetRosParameter(pnh, "hover_stiff_kd/z",
                    mellinger_controller_.controller_parameters_.hover_xyz_stiff_kd_.z(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_stiff_kd_.z());

  GetRosParameter(pnh, "hover_stiff_angle_kp/phi",
                    mellinger_controller_.controller_parameters_.hover_xyz_stiff_angle_kp_.x(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_stiff_angle_kp_.x());
    GetRosParameter(pnh, "hover_stiff_angle_kp/theta",
                    mellinger_controller_.controller_parameters_.hover_xyz_stiff_angle_kp_.y(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_stiff_angle_kp_.y());
    GetRosParameter(pnh, "hover_stiff_angle_kp/psi",
                    mellinger_controller_.controller_parameters_.hover_xyz_stiff_angle_kp_.z(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_stiff_angle_kp_.z());
    GetRosParameter(pnh, "hover_stiff_angle_kd/phi",
                    mellinger_controller_.controller_parameters_.hover_xyz_stiff_angle_kd_.x(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_stiff_angle_kd_.x());
    GetRosParameter(pnh, "hover_stiff_angle_kd/theta",
                    mellinger_controller_.controller_parameters_.hover_xyz_stiff_angle_kd_.y(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_stiff_angle_kd_.y());
    GetRosParameter(pnh, "hover_stiff_angle_kd/psi",
                    mellinger_controller_.controller_parameters_.hover_xyz_stiff_angle_kd_.z(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_stiff_angle_kd_.z());

    GetRosParameter(pnh, "hover_soft_angle_kp/phi",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_angle_kp_.x(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_angle_kp_.x());
    GetRosParameter(pnh, "hover_soft_angle_kp/theta",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_angle_kp_.y(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_angle_kp_.y());
    GetRosParameter(pnh, "hover_soft_angle_kp/psi",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_angle_kp_.z(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_angle_kp_.z());
    GetRosParameter(pnh, "hover_soft_angle_kd/phi",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_angle_kd_.x(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_angle_kd_.x());
    GetRosParameter(pnh, "hover_soft_angle_kd/theta",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_angle_kd_.y(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_angle_kd_.y());
    GetRosParameter(pnh, "hover_soft_angle_kd/psi",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_angle_kd_.z(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_angle_kd_.z());


    GetRosParameter(pnh, "hover_soft_kp/x",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_kp_.x(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_kp_.x());
    GetRosParameter(pnh, "hover_soft_kp/y",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_kp_.y(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_kp_.y());
    GetRosParameter(pnh, "hover_soft_kp/z",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_kp_.z(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_kp_.z());
    GetRosParameter(pnh, "hover_soft_ki/x",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_ki_.x(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_ki_.x());
    GetRosParameter(pnh, "hover_soft_ki/y",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_ki_.y(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_ki_.y());
    GetRosParameter(pnh, "hover_soft_ki/z",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_ki_.z(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_ki_.z());
    GetRosParameter(pnh, "hover_soft_kd/x",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_kd_.x(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_kd_.x());
    GetRosParameter(pnh, "hover_soft_kd/y",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_kd_.y(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_kd_.y());
    GetRosParameter(pnh, "hover_soft_kd/z",
                    mellinger_controller_.controller_parameters_.hover_xyz_soft_kd_.z(),
                    &mellinger_controller_.controller_parameters_.hover_xyz_soft_kd_.z());

   GetRosParameter(pnh, "attitude_kp/phi",
                  mellinger_controller_.controller_parameters_.attitude_kp_.x(),
                  &mellinger_controller_.controller_parameters_.attitude_kp_.x());
  GetRosParameter(pnh, "attitude_kp/theta",
                  mellinger_controller_.controller_parameters_.attitude_kp_.y(),
                  &mellinger_controller_.controller_parameters_.attitude_kp_.y());
  GetRosParameter(pnh, "attitude_kp/psi",
                    mellinger_controller_.controller_parameters_.attitude_kp_.z(),
                    &mellinger_controller_.controller_parameters_.attitude_kp_.z());
    GetRosParameter(pnh, "attitude_kd/phi",
                    mellinger_controller_.controller_parameters_.attitude_kd_.x(),
                    &mellinger_controller_.controller_parameters_.attitude_kd_.x());
    GetRosParameter(pnh, "attitude_kd/theta",
                    mellinger_controller_.controller_parameters_.attitude_kd_.y(),
                    &mellinger_controller_.controller_parameters_.attitude_kd_.y());
    GetRosParameter(pnh, "attitude_kd/psi",
                    mellinger_controller_.controller_parameters_.attitude_kd_.z(),
                    &mellinger_controller_.controller_parameters_.attitude_kd_.z());

    GetRosParameter(pnh, "path_angle_kp/phi",
                    mellinger_controller_.controller_parameters_.path_angle_kp_.x(),
                    &mellinger_controller_.controller_parameters_.path_angle_kp_.x());
    GetRosParameter(pnh, "path_angle_kp/theta",
                    mellinger_controller_.controller_parameters_.path_angle_kp_.y(),
                    &mellinger_controller_.controller_parameters_.path_angle_kp_.y());
    GetRosParameter(pnh, "path_angle_kp/psi",
                    mellinger_controller_.controller_parameters_.path_angle_kp_.z(),
                    &mellinger_controller_.controller_parameters_.path_angle_kp_.z());

    GetRosParameter(pnh, "path_angle_kd/phi",
                    mellinger_controller_.controller_parameters_.path_angle_kd_.x(),
                    &mellinger_controller_.controller_parameters_.path_angle_kd_.x());
    GetRosParameter(pnh, "path_angle_kd/theta",
                    mellinger_controller_.controller_parameters_.path_angle_kd_.y(),
                    &mellinger_controller_.controller_parameters_.path_angle_kd_.y());
    GetRosParameter(pnh, "path_angle_kd/psi",
                    mellinger_controller_.controller_parameters_.path_angle_kd_.z(),
                    &mellinger_controller_.controller_parameters_.path_angle_kd_.z());

    GetRosParameter(pnh, "path_kp/x",
                    mellinger_controller_.controller_parameters_.path_kp_.x(),
                    &mellinger_controller_.controller_parameters_.path_kp_.x());
    GetRosParameter(pnh, "path_kp/y",
                    mellinger_controller_.controller_parameters_.path_kp_.y(),
                    &mellinger_controller_.controller_parameters_.path_kp_.y());
    GetRosParameter(pnh, "path_kp/z",
                    mellinger_controller_.controller_parameters_.path_kp_.z(),
                    &mellinger_controller_.controller_parameters_.path_kp_.z());
    GetRosParameter(pnh, "path_kd/x",
                    mellinger_controller_.controller_parameters_.path_kd_.x(),
                    &mellinger_controller_.controller_parameters_.path_kd_.x());
    GetRosParameter(pnh, "path_kd/y",
                    mellinger_controller_.controller_parameters_.path_kd_.y(),
                    &mellinger_controller_.controller_parameters_.path_kd_.y());
    GetRosParameter(pnh, "path_kd/z",
                    mellinger_controller_.controller_parameters_.path_kd_.z(),
                    &mellinger_controller_.controller_parameters_.path_kd_.z());
    GetRosParameter(pnh, """bf",
                    mellinger_controller_.controller_parameters_.bf,
                    &mellinger_controller_.controller_parameters_.bf);
    GetRosParameter(pnh, "bm",
                    mellinger_controller_.controller_parameters_.bm,
                    &mellinger_controller_.controller_parameters_.bm);
    GetRosParameter(pnh, "l",
                    mellinger_controller_.controller_parameters_.l,
                    &mellinger_controller_.controller_parameters_.l);


    double mass, Ixx,Iyy, Izz, Ixy, Ixz, Iyz;
    GetRosParameter(pnh, "inertia/xx",
                    Ixx, &Ixx);
    GetRosParameter(pnh, "inertia/yy",
                    Iyy, &Iyy);
    GetRosParameter(pnh, "inertia/zz",
                    Izz, &Izz);
    GetRosParameter(pnh, "inertia/xy",
                    Ixy, &Ixy);
    GetRosParameter(pnh, "inertia/xz",
                    Ixz, &Ixz);
    GetRosParameter(pnh, "inertia/yz",
                    Iyz, &Iyz);
    GetRosParameter(pnh, "mass",
                    mass, &mass);


    mellinger_controller_.controller_parameters_.Inertia << Ixx, Ixy, Ixz,
                                                            Ixy, Iyy, Iyz,
                                                            Ixz, Iyz, Izz ;
    mellinger_controller_.controller_parameters_.mass = mass;
    mellinger_controller_.SetControllerGains();


 //if (enable_state_estimator_)
   // mellinger_controller_.crazyflie_onboard_controller_.SetControllerGains(mellinger_controller_.controller_parameters_);

}

void MellingerControlNode::Publish(){
}

void MellingerControlNode::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

    ROS_INFO_ONCE("PositionController got first imu message.");
    
    // Angular velocities data
    sensors_.gyro.x = imu_msg->angular_velocity.x;
    sensors_.gyro.y = imu_msg->angular_velocity.y;
    sensors_.gyro.z = imu_msg->angular_velocity.z;
    
    // Linear acceleration data
    sensors_.acc.x = imu_msg->linear_acceleration.x;
    sensors_.acc.y = imu_msg->linear_acceleration.y;
    sensors_.acc.z = imu_msg->linear_acceleration.z;

    imu_msg_head_stamp_ = imu_msg->header.stamp;	

}

void MellingerControlNode::PathCallback(const std_msgs::BoolConstPtr& path_active){
 if (path_active->data)
   mellinger_controller_.setPathFollow();
 else
   mellinger_controller_.resetPathFollow();
}
void MellingerControlNode::HoverCallback(const std_msgs::BoolConstPtr& hover_active){
  if(hover_active->data)
    mellinger_controller_.setHover();
  else
   mellinger_controller_.resetHover();
}

void MellingerControlNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("PositionController got first odometry message.");

    if (waypointHasBeenPublished_ && enable_state_estimator_){

	    //This functions allows us to put the odometry message into the odometry variable--> _position,
 	    //_orientation,_velocit_body,_angular_velocity
	    EigenOdometry odometry;
	    eigenOdometryFromMsg(odometry_msg, &odometry);
	    mellinger_controller_.SetOdometryWithStateEstimator(odometry);

    }

    if(waypointHasBeenPublished_){

      //This functions allows us to put the odometry message into the odometry variable--> _position,
      //_orientation,_velocit_body,_angular_velocity
      EigenOdometry odometry;
      eigenOdometryFromMsg(odometry_msg, &odometry);
      mellinger_controller_.SetOdometryWithoutStateEstimator(odometry);

      Eigen::Vector4d ref_rotor_velocities;
      mellinger_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

      //creating a new mav message. actuator_msg is used to send the velocities of the propellers.
      mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

      //we use clear because we later want to be sure that we used the previously calculated velocity.
      actuator_msg->angular_velocities.clear();
      //for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
      for (int i = 0; i < ref_rotor_velocities.size(); i++)
         actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
      actuator_msg->header.stamp = odometry_msg->header.stamp;

      motor_velocity_reference_pub_.publish(actuator_msg);

    }
   
}

// The attitude is estimated only if the waypoint has been published
void MellingerControlNode::CallbackAttitudeEstimation(const ros::TimerEvent& event){

    if (waypointHasBeenPublished_)
            mellinger_controller_.CallbackAttitudeEstimation();

}

// The high level control is run only if the waypoint has been published
void MellingerControlNode::CallbackHightLevelControl(const ros::TimerEvent& event){

    if (waypointHasBeenPublished_)
            mellinger_controller_.CallbackHightLevelControl();

}

// IMU messages are sent to the controller with a frequency of 500Hz. In other words, with a sampling time of 0.002 seconds
void MellingerControlNode::CallbackIMUUpdate(const ros::TimerEvent& event){

    mellinger_controller_.SetSensorData(sensors_);

    ROS_INFO_ONCE("IMU Message sent to position controller");

    if (waypointHasBeenPublished_){

	    Eigen::Vector4d ref_rotor_velocities;
	    mellinger_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

	    // A new mav message, actuator_msg, is used to send to Gazebo the propellers angular velocities.
	    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

	    // The clear method makes sure the actuator_msg is empty (there are no previous values of the propellers angular velocities).
	    actuator_msg->angular_velocities.clear();
	    // for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
	    for (int i = 0; i < ref_rotor_velocities.size(); i++)
         actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
	    actuator_msg->header.stamp = imu_msg_head_stamp_;
	    
	    motor_velocity_reference_pub_.publish(actuator_msg);

    }

}


}

int main(int argc, char** argv){
    ros::init(argc, argv, "mellinger_controller_node_with_stateEstimator");
    
    ros::NodeHandle nh2;
    
    rotors_control::MellingerControlNode mellinger_controller_node;

    ros::spin();

    return 0;
}
