#include "ros/ros.h"
#include <Eigen/Geometry> 
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <ros/package.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <vector>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_msgs/RollPitchYawrateThrust.h>
#include <std_msgs/Bool.h>

using namespace std;

 Eigen::Quaterniond quat_from_euler(double yaw, double pitch, double roll);
void build_reference(vector<string> line, 
          trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);
void FlagCallback(const std_msgs::BoolConstPtr& flag_msg);
void build_eight(vector<string> line, 
          trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);


bool flip_flag = false;

int main(int argc, char **argv) {

  ros::init(argc, argv, "reference_trajectory");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>
                        (mav_msgs::default_topics::COMMAND_TRAJECTORY, 1000);
  
  ros::Subscriber flag_sub_ = n.subscribe("flip_flag",10,&FlagCallback);
  
  ros::Rate loop_rate1(5);

  ifstream input_file;
  string row;

  //input_file.open((ros::package::getPath("rotors_gazebo")+"/src/traj.txt").c_str());

  input_file.open((ros::package::getPath("rotors_gazebo")+"/src/eight_traj.txt").c_str());

  if (!input_file) {
    ROS_INFO("Unable to open file!");
    exit(1);
  }




  while (ros::ok()) {
/*
     if (!flip_flag){
      trajectory_msg.header.stamp = ros::Time::now();

      build_reference(split_start, trajectory_msg, attitude_msg);

      pub1.publish(attitude_msg);
      pub.publish(trajectory_msg);

     }

*/
    if (flip_flag && getline(input_file,row)) {

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

    trajectory_msg.header.stamp = ros::Time::now();

    istringstream iss(row);
    vector<string> split((istream_iterator<string>(iss)),istream_iterator<string>());
    //build_reference(split, trajectory_msg);
    build_eight(split, trajectory_msg);

    pub.publish(trajectory_msg);

    }

    ros::spinOnce();
    loop_rate1.sleep();

  }

  input_file.close();

  return 0;
}

 Eigen::Quaterniond quat_from_euler(double yaw, double pitch, double roll) {

  
  Eigen::Quaterniond q;

  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  q.w() = cy * cp * cr + sy * sp * sr;
  q.x() = cy * cp * sr - sy * sp * cr;
  q.y() = sy * cp * sr + cy * sp * cr;
  q.z() = sy * cp * cr - cy * sp * sr;


  return q;

}

void FlagCallback(const std_msgs::BoolConstPtr& flag_msg) {
  
  flip_flag = flag_msg->data;

}

void build_reference(vector<string> line, 
          trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg) {
  
  Eigen::Quaterniond q;
  mav_msgs::EigenTrajectoryPoint point;
  Eigen::Vector3d desired_position(0.0, atof(line[0].c_str()), atof(line[1].c_str()));

  q = quat_from_euler(0,atof(line[2].c_str()),0);

  point.position_W=desired_position;
  point.orientation_W_B=q;

  mav_msgs::msgMultiDofJointTrajectoryFromEigen(point, &trajectory_msg);
/*
   attitude_msg.roll = 0;
   attitude_msg.pitch = atof(line[2].c_str());
   attitude_msg.yaw_rate = 0;
*/

}


void build_eight(vector<string> line, 
          trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg) {
  
  Eigen::Quaterniond q;
  mav_msgs::EigenTrajectoryPoint point;
  Eigen::Vector3d desired_position(atof(line[0].c_str()), atof(line[1].c_str()), atof(line[2].c_str()));

  std::cout << "ok";
  std::cout << atof(line[0].c_str()) << std::endl;

  q = quat_from_euler(atof(line[3].c_str()),atof(line[4].c_str()),atof(line[5].c_str()));

  point.position_W=desired_position;
  point.orientation_W_B=q;

  mav_msgs::msgMultiDofJointTrajectoryFromEigen(point, &trajectory_msg);

  // fill the message for the attitude
    /*attitude_msg.roll = atof(line[3].c_str());
    attitude_msg.pitch = atof(line[4].c_str());
    attitude_msg.yaw_rate = atof(line[5].c_str());
  */
}