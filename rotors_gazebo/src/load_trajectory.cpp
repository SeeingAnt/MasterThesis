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

#define M_PI                                     3.14159265358979323846

 Eigen::Quaterniond quat_from_euler(double yaw, double pitch, double roll);
void build_reference(vector<string> line, 
          trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);
void build_eight(vector<string> line, 
          trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);
void build_traj(vector<string> line,
          trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);

std_msgs::Bool c_flag;

int main(int argc, char **argv) {

  ros::init(argc, argv, "reference_trajectory");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>
                        (mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
  
  ros::Publisher hover_pub =
     n.advertise<std_msgs::Bool>("hover_active", 1);

  ros::Publisher c_pub = n.advertise<std_msgs::Bool>("control_flag", 1000);
  c_flag.data = true;

  ros::Publisher path_pub =
     n.advertise<std_msgs::Bool>("path_active", 1);

  ros::Rate loop_rate1(20);

  ifstream input_file;
  string row;

  //input_file.open((ros::package::getPath("rotors_gazebo")+"/src/traj.txt").c_str());
  //input_file.open((ros::package::getPath("rotors_gazebo")+"/src/prova.txt").c_str());
  input_file.open((ros::package::getPath("rotors_gazebo")+"/src/eight_traj.txt").c_str());

  if (!input_file) {
    ROS_INFO("Unable to open file!");
    exit(1);
  }

  ros::Duration(2.0).sleep();

  std_msgs::Bool path;
  std_msgs::Bool hover;

  hover.data = true;
  hover_pub.publish(hover);

  ROS_INFO("HOVER!");

  getline(input_file,row);

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

  trajectory_msg.header.stamp = ros::Time::now();

  istringstream iss(row);
  vector<string> split((istream_iterator<string>(iss)),istream_iterator<string>());
  //build_reference(split, trajectory_msg);
  build_eight(split, trajectory_msg);
  //build_traj(split, trajectory_msg);

  pub.publish(trajectory_msg);

  ros::Duration(8.0).sleep();

  hover.data = false;
  hover_pub.publish(hover);

  path.data = true;
  path_pub.publish(path);

  ROS_INFO("AGGRESSIVE!");

  loop_rate1.sleep();
  while (getline(input_file,row)) {

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

    trajectory_msg.header.stamp = ros::Time::now();

    istringstream iss(row);
    vector<string> split((istream_iterator<string>(iss)),istream_iterator<string>());
    //build_reference(split, trajectory_msg);
    build_eight(split, trajectory_msg);
    //build_traj(split, trajectory_msg);

    pub.publish(trajectory_msg);
   // c_pub.publish(c_flag);

    ros::spinOnce();
    loop_rate1.sleep();

  }

 ROS_INFO("HOVER!");
 hover.data = true;
 hover_pub.publish(hover);

 path.data = false;
 path_pub.publish(path);

  input_file.close();

  ros::spin();

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

void build_reference(vector<string> line, 
          trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg) {
  
  Eigen::Quaterniond q;
  mav_msgs::EigenTrajectoryPoint point;
  Eigen::Vector3d desired_position(atof(line[0].c_str()), atof(line[1].c_str()), atof(line[2].c_str()));
  Eigen::Vector3d desired_velocity(atof(line[6].c_str()),atof(line[7].c_str()),atof(line[8].c_str()));
  Eigen::Vector3d desired_acceleration(atof(line[12].c_str()),atof(line[13].c_str()),atof(line[14].c_str()));
  Eigen::Vector3d angular_velocity(atof(line[9].c_str()),atof(line[10].c_str()),atof(line[11].c_str()));
  Eigen::Vector3d angular_acceleration(atof(line[15].c_str()),atof(line[16].c_str()),atof(line[17].c_str()));

  q = quat_from_euler(atof(line[5].c_str()),atof(line[4].c_str()),atof(line[3].c_str()));

  point.angular_acceleration_W = angular_acceleration;
  point.position_W=desired_position;
  point.velocity_W=desired_velocity;
  point.acceleration_W = desired_acceleration;
  point.orientation_W_B=q;
  point.angular_velocity_W = angular_velocity;

  mav_msgs::msgMultiDofJointTrajectoryFromEigen(point, &trajectory_msg);

}


void build_eight(vector<string> line, 
          trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg) {
  
  Eigen::Quaterniond q;
  mav_msgs::EigenTrajectoryPoint point;
  Eigen::Vector3d desired_position(atof(line[0].c_str()), atof(line[1].c_str()), atof(line[2].c_str()));
  Eigen::Vector3d desired_velocity(atof(line[6].c_str()),atof(line[7].c_str()),atof(line[8].c_str()));
  Eigen::Vector3d desired_acceleration(atof(line[9].c_str()),atof(line[10].c_str()),atof(line[11].c_str()));

  std::cout << "ok";
  std::cout << atof(line[0].c_str()) << std::endl;

  q = quat_from_euler(atof(line[3].c_str()),atof(line[4].c_str()),atof(line[5].c_str()));

  point.position_W=desired_position;
  point.velocity_W=desired_velocity;
  point.acceleration_W = desired_acceleration;
  point.orientation_W_B=q;

  mav_msgs::msgMultiDofJointTrajectoryFromEigen(point, &trajectory_msg);

}

void build_traj(vector<string> line,
          trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg) {

  Eigen::Quaterniond q;
  mav_msgs::EigenTrajectoryPoint point;
  Eigen::Vector3d desired_position(atof(line[0].c_str()), atof(line[1].c_str()), atof(line[2].c_str()));
  Eigen::Vector3d desired_velocity(atof(line[4].c_str()),atof(line[5].c_str()),atof(line[6].c_str()));
  Eigen::Vector3d desired_acceleration(atof(line[8].c_str()),atof(line[9].c_str()),atof(line[10].c_str()));

  std::cout << "ok";
  std::cout << atof(line[0].c_str()) << std::endl;

  q = quat_from_euler(atof(line[3].c_str()),0,0);

  point.position_W=desired_position;
  point.velocity_W=desired_velocity;
  point.acceleration_W = desired_acceleration;
  point.orientation_W_B=q;

  mav_msgs::msgMultiDofJointTrajectoryFromEigen(point, &trajectory_msg);


}
