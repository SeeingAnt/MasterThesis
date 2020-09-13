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

#include "aggressive_control_node.h"

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include "rotors_comm/GainMSG.h"

#include <ros/console.h> 
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>


#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/crazyflie_complementary_filter.h"

#define ATTITUDE_UPDATE_DT 0.004  /* ATTITUDE UPDATE RATE [s] - 250Hz */
#define RATE_UPDATE_DT 0.002      /* RATE UPDATE RATE [s] - 500Hz */
#define SAMPLING_TIME  0.001      /* SAMPLING CONTROLLER TIME [s] - 100Hz */

namespace rotors_control {

AggressiveControlNode::AggressiveControlNode() {

    ROS_INFO_ONCE("Started position controller");

    InitializeParams();

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,  &AggressiveControlNode::MultiDofJointTrajectoryCallback, this);

    active_controller_sub_ = nh.subscribe("control_flag",1,&AggressiveControlNode::ActiveCallback, this);

    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 30, &AggressiveControlNode::OdometryCallback, this);

    init_sub_ = nh.advertiseService("init_model",&AggressiveControlNode::InitService,this);

    gains_sub_ = nh.subscribe("ControlGains", 1, &AggressiveControlNode::CallbackGainsControl,this);

    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    forces_pub_ = nh.advertise<mav_msgs::Actuators>("forces", 1);

    // The subscription to the IMU topic is made only if it is available, when simulating the Crazyflie dynamics considering the also IMU values

    hover_active = nh.subscribe("hover_active",1,&AggressiveControlNode::HoverCallback,this);
    path_active = nh.subscribe("path_active",1,&AggressiveControlNode::PathCallback,this);

    if(pnh_node.getParam("enable_state_estimator", enable_state_estimator_)){
      ROS_INFO("Got param 'enable_state_estimator': %d", enable_state_estimator_);
    }

    if (enable_state_estimator_){
        imu_sub_ = nh.subscribe(mav_msgs::default_topics::IMU, 1, &AggressiveControlNode::IMUCallback, this);

        //Timers allow to set up the working frequency of the control system
        timer_Attitude_ = n_.createTimer(ros::Duration(ATTITUDE_UPDATE_DT), &AggressiveControlNode::CallbackAttitudeEstimation, this, false, true);

        timer_highLevelControl = n_.createTimer(ros::Duration(SAMPLING_TIME), &AggressiveControlNode::CallbackHightLevelControl, this, false, true);

        timer_IMUUpdate = n_.createTimer(ros::Duration(RATE_UPDATE_DT), &AggressiveControlNode::CallbackIMUUpdate, this, false, true);

     }


}


AggressiveControlNode::~AggressiveControlNode(){}

void AggressiveControlNode::ActiveCallback(const std_msgs::BoolConstPtr& msg){

  aggressive_controller_.active = msg->data;

}

void AggressiveControlNode::CallbackGainsControl(const rotors_comm::GainMSG &control_gains){

     aggressive_controller_.SetGainP(control_gains.Gain_p.data());
     aggressive_controller_.SetGainD(control_gains.Gain_d.data());
     aggressive_controller_.SetGainPT(control_gains.Gain_pt.data());
     aggressive_controller_.SetGainDT(control_gains.Gain_dt.data());
     aggressive_controller_.SetGainPP(control_gains.Gain_pp.data());
     aggressive_controller_.SetGainDD(control_gains.Gain_dd.data());
     aggressive_controller_.SetGainPeta(control_gains.Gain_peta.data());
     aggressive_controller_.SetGainDom(control_gains.Gain_dom.data());

}


void AggressiveControlNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
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
  aggressive_controller_.SetTrajectoryPoint(eigen_reference);
  commands_.pop_front();

  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO("PositionController got first MultiDOFJointTrajectory message.");
  }
}

bool AggressiveControlNode::InitService(rotors_comm::init_service::Request  &req,
                                            rotors_comm::init_service::Response &res){

  InitializeParams();
  if (aggressive_controller_.hover_is_active == true)
    aggressive_controller_.setHover();
  if (aggressive_controller_.path_is_active == true)
    aggressive_controller_.setPathFollow();
  res.success = true;
  return true;

}

void AggressiveControlNode::InitializeParams(){
  ros::NodeHandle pnh("~");

   ROS_INFO("Parameters initialize.");
  // Parameters reading from rosparam.
  GetRosParameter(pnh, "hover_stiff_kp/x",
                  aggressive_controller_.controller_parameters_.hover_xyz_stiff_kp_.x(),
                  &aggressive_controller_.controller_parameters_.hover_xyz_stiff_kp_.x());
  GetRosParameter(pnh, "hover_stiff_kp/y",
                  aggressive_controller_.controller_parameters_.hover_xyz_stiff_kp_.y(),
                  &aggressive_controller_.controller_parameters_.hover_xyz_stiff_kp_.y());
  GetRosParameter(pnh, "hover_stiff_kp/z",
                  aggressive_controller_.controller_parameters_.hover_xyz_stiff_kp_.z(),
                  &aggressive_controller_.controller_parameters_.hover_xyz_stiff_kp_.z());
  GetRosParameter(pnh, "hover_stiff_ki/x",
                  aggressive_controller_.controller_parameters_.hover_xyz_stiff_ki_.x(),
                  &aggressive_controller_.controller_parameters_.hover_xyz_stiff_ki_.x());
  GetRosParameter(pnh, "hover_stiff_ki/y",
                    aggressive_controller_.controller_parameters_.hover_xyz_stiff_ki_.y(),
                    &aggressive_controller_.controller_parameters_.hover_xyz_stiff_ki_.y());
  GetRosParameter(pnh, "hover_stiff_ki/z",
                    aggressive_controller_.controller_parameters_.hover_xyz_stiff_ki_.z(),
                    &aggressive_controller_.controller_parameters_.hover_xyz_stiff_ki_.z());
  GetRosParameter(pnh, "hover_stiff_kd/x",
                    aggressive_controller_.controller_parameters_.hover_xyz_stiff_kd_.x(),
                    &aggressive_controller_.controller_parameters_.hover_xyz_stiff_kd_.x());
  GetRosParameter(pnh, "hover_stiff_kd/y",
                    aggressive_controller_.controller_parameters_.hover_xyz_stiff_kd_.y(),
                    &aggressive_controller_.controller_parameters_.hover_xyz_stiff_kd_.y());
  GetRosParameter(pnh, "hover_stiff_kd/z",
                    aggressive_controller_.controller_parameters_.hover_xyz_stiff_kd_.z(),
                    &aggressive_controller_.controller_parameters_.hover_xyz_stiff_kd_.z());

  GetRosParameter(pnh, "hover_stiff_angle_kp/phi",
                    aggressive_controller_.controller_parameters_.hover_xyz_stiff_angle_kp_.x(),
                    &aggressive_controller_.controller_parameters_.hover_xyz_stiff_angle_kp_.x());
    GetRosParameter(pnh, "hover_stiff_angle_kp/theta",
                    aggressive_controller_.controller_parameters_.hover_xyz_stiff_angle_kp_.y(),
                    &aggressive_controller_.controller_parameters_.hover_xyz_stiff_angle_kp_.y());
    GetRosParameter(pnh, "hover_stiff_angle_kp/psi",
                    aggressive_controller_.controller_parameters_.hover_xyz_stiff_angle_kp_.z(),
                    &aggressive_controller_.controller_parameters_.hover_xyz_stiff_angle_kp_.z());
    GetRosParameter(pnh, "hover_stiff_angle_kd/phi",
                    aggressive_controller_.controller_parameters_.hover_xyz_stiff_angle_kd_.x(),
                    &aggressive_controller_.controller_parameters_.hover_xyz_stiff_angle_kd_.x());
    GetRosParameter(pnh, "hover_stiff_angle_kd/theta",
                    aggressive_controller_.controller_parameters_.hover_xyz_stiff_angle_kd_.y(),
                    &aggressive_controller_.controller_parameters_.hover_xyz_stiff_angle_kd_.y());
    GetRosParameter(pnh, "hover_stiff_angle_kd/psi",
                    aggressive_controller_.controller_parameters_.hover_xyz_stiff_angle_kd_.z(),
                    &aggressive_controller_.controller_parameters_.hover_xyz_stiff_angle_kd_.z());

    GetRosParameter(pnh, "path_angle_kp/phi",
                    aggressive_controller_.controller_parameters_.path_angle_kp_.x(),
                    &aggressive_controller_.controller_parameters_.path_angle_kp_.x());
    GetRosParameter(pnh, "path_angle_kp/theta",
                    aggressive_controller_.controller_parameters_.path_angle_kp_.y(),
                    &aggressive_controller_.controller_parameters_.path_angle_kp_.y());
    GetRosParameter(pnh, "path_angle_kp/psi",
                    aggressive_controller_.controller_parameters_.path_angle_kp_.z(),
                    &aggressive_controller_.controller_parameters_.path_angle_kp_.z());

    GetRosParameter(pnh, "path_angle_kd/phi",
                    aggressive_controller_.controller_parameters_.path_angle_kd_.x(),
                    &aggressive_controller_.controller_parameters_.path_angle_kd_.x());
    GetRosParameter(pnh, "path_angle_kd/theta",
                    aggressive_controller_.controller_parameters_.path_angle_kd_.y(),
                    &aggressive_controller_.controller_parameters_.path_angle_kd_.y());
    GetRosParameter(pnh, "path_angle_kd/psi",
                    aggressive_controller_.controller_parameters_.path_angle_kd_.z(),
                    &aggressive_controller_.controller_parameters_.path_angle_kd_.z());

    GetRosParameter(pnh, "path_kp/x",
                    aggressive_controller_.controller_parameters_.path_kp_.x(),
                    &aggressive_controller_.controller_parameters_.path_kp_.x());
    GetRosParameter(pnh, "path_kp/y",
                    aggressive_controller_.controller_parameters_.path_kp_.y(),
                    &aggressive_controller_.controller_parameters_.path_kp_.y());
    GetRosParameter(pnh, "path_kp/z",
                    aggressive_controller_.controller_parameters_.path_kp_.z(),
                    &aggressive_controller_.controller_parameters_.path_kp_.z());
    GetRosParameter(pnh, "path_kd/x",
                    aggressive_controller_.controller_parameters_.path_kd_.x(),
                    &aggressive_controller_.controller_parameters_.path_kd_.x());
    GetRosParameter(pnh, "path_kd/y",
                    aggressive_controller_.controller_parameters_.path_kd_.y(),
                    &aggressive_controller_.controller_parameters_.path_kd_.y());
    GetRosParameter(pnh, "path_kd/z",
                    aggressive_controller_.controller_parameters_.path_kd_.z(),
                    &aggressive_controller_.controller_parameters_.path_kd_.z());
    GetRosParameter(pnh, """bf",
                    aggressive_controller_.controller_parameters_.bf,
                    &aggressive_controller_.controller_parameters_.bf);
    GetRosParameter(pnh, "bm",
                    aggressive_controller_.controller_parameters_.bm,
                    &aggressive_controller_.controller_parameters_.bm);
    GetRosParameter(pnh, "l",
                    aggressive_controller_.controller_parameters_.l,
                    &aggressive_controller_.controller_parameters_.l);


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


    aggressive_controller_.controller_parameters_.Inertia << Ixx, Ixy, Ixz,
                                                            Ixy, Iyy, Iyz,
                                                            Ixz, Iyz, Izz ;
    aggressive_controller_.controller_parameters_.mass = mass;
    aggressive_controller_.SetControllerGains();


 //if (enable_state_estimator_)
   // mellinger_controller_.crazyflie_onboard_controller_.SetControllerGains(mellinger_controller_.controller_parameters_);

}

void AggressiveControlNode::Publish(){
}

void AggressiveControlNode::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

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

void AggressiveControlNode::PathCallback(const std_msgs::BoolConstPtr& path_active){
 if (path_active->data)
   aggressive_controller_.setPathFollow();
 else
   aggressive_controller_.resetPathFollow();
}
void AggressiveControlNode::HoverCallback(const std_msgs::BoolConstPtr& hover_active){
  if(hover_active->data)
    aggressive_controller_.setHover();
  else
   aggressive_controller_.resetHover();
}

void AggressiveControlNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("PositionController got first odometry message.");

    if (waypointHasBeenPublished_ && enable_state_estimator_){

	    //This functions allows us to put the odometry message into the odometry variable--> _position,
 	    //_orientation,_velocit_body,_angular_velocity
	    EigenOdometry odometry;
	    eigenOdometryFromMsg(odometry_msg, &odometry);
      aggressive_controller_.SetOdometryWithStateEstimator(odometry);

    }

    if(waypointHasBeenPublished_){

      //This functions allows us to put the odometry message into the odometry variable--> _position,
      //_orientation,_velocit_body,_angular_velocity
      EigenOdometry odometry;
      eigenOdometryFromMsg(odometry_msg, &odometry);
      aggressive_controller_.SetOdometryWithoutStateEstimator(odometry);

      Eigen::Vector4d ref_rotor_velocities;
      Eigen::Vector4d forces;
      aggressive_controller_.CalculateRotorVelocities(&ref_rotor_velocities,&forces);

      mav_msgs::ActuatorsPtr forces_msg(new mav_msgs::Actuators);

      //we use clear because we later want to be sure that we used the previously calculated velocity.
      forces_msg->angular_velocities.clear();
      //for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
      for (int i = 0; i < ref_rotor_velocities.size(); i++)
         forces_msg->angular_velocities.push_back(forces[i]);
      forces_msg->header.stamp = odometry_msg->header.stamp;

      forces_pub_.publish(forces_msg);


      //creating a new mav message. actuatros::Publisher motor_velocity_reference_pub_;ros::Publisher motor_velocity_reference_pub_;ros::Publisher motor_velocity_reference_pub_;or_msg is used to send the velocities of the propellers.
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
void AggressiveControlNode::CallbackAttitudeEstimation(const ros::TimerEvent& event){

    if (waypointHasBeenPublished_)
            aggressive_controller_.CallbackAttitudeEstimation();

}

// The high level control is run only if the waypoint has been published
void AggressiveControlNode::CallbackHightLevelControl(const ros::TimerEvent& event){

    if (waypointHasBeenPublished_)
            aggressive_controller_.CallbackHightLevelControl();

}

// IMU messages are sent to the controller with a frequency of 500Hz. In other words, with a sampling time of 0.002 seconds
void AggressiveControlNode::CallbackIMUUpdate(const ros::TimerEvent& event){

    aggressive_controller_.SetSensorData(sensors_);

    ROS_INFO_ONCE("IMU Message sent to position controller");

}


}

int main(int argc, char** argv){
    ros::init(argc, argv, "aggressive_controller_node_with_stateEstimator");
    
    ros::NodeHandle nh2;
    
    rotors_control::AggressiveControlNode aggressive_controller_node;

    ros::spin();

    return 0;
}
