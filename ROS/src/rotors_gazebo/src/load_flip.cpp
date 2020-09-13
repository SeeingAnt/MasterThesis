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
#include "rotors_comm/GainMSG.h"

#include <mav_msgs/RollPitchYawrateThrust.h>
#include <std_msgs/Bool.h>

using namespace std;

#define M_PI                                     3.14159265358979323846

 Eigen::Quaterniond quat_from_euler(double yaw, double pitch, double roll);
void build_reference(vector<string> line, 
          trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);
void Control_Matrix(ifstream &Gain_file, rotors_comm::GainMSG &gains_message);
void fillGainMessage(string Gain,double Gain1[],double Gain2[], double Gain3[], double Gain4[]);


std_msgs::Bool c_flag;

int main(int argc, char **argv) {

  ros::init(argc, argv, "reference_trajectory");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>
                        (mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
  
  ros::Publisher hover_pub =
     n.advertise<std_msgs::Bool>("hover_active", 1);
  ros::Publisher Gains_pub = n.advertise<rotors_comm::GainMSG>("ControlGains", 1);

  ros::Publisher c_pub = n.advertise<std_msgs::Bool>("control_flag", 1000);
  c_flag.data = true;

  ros::Publisher path_pub =
     n.advertise<std_msgs::Bool>("path_active", 1);

  ros::Rate loop_rate1(500);

  ifstream input_file;
  ifstream Gain_file;
  string row;

  input_file.open((ros::package::getPath("rotors_gazebo")+"/src/traj.txt").c_str());
  Gain_file.open((ros::package::getPath("rotors_gazebo")+"/src/Gains.txt").c_str());

  if (!input_file) {
    ROS_INFO("Unable to open file!");
    exit(1);
  }

  if (!Gain_file) {
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

  rotors_comm::GainMSG gains_message;
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

  trajectory_msg.header.stamp = ros::Time::now();

  istringstream iss(row);
  vector<string> split((istream_iterator<string>(iss)),istream_iterator<string>());
  build_reference(split, trajectory_msg);
  Control_Matrix(Gain_file,gains_message);

  Gains_pub.publish(gains_message);

  pub.publish(trajectory_msg);

  ros::Duration(5.0).sleep();

  hover.data = false;
  hover_pub.publish(hover);

  path.data = true;
  path_pub.publish(path);

  ROS_INFO("AGGRESSIVE!");

  loop_rate1.sleep();
  while (getline(input_file,row)) {



    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    rotors_comm::GainMSG gains_message;

    trajectory_msg.header.stamp = ros::Time::now();

    istringstream iss(row);
    vector<string> split((istream_iterator<string>(iss)),istream_iterator<string>());
    build_reference(split, trajectory_msg);
    Control_Matrix(Gain_file,gains_message);

    Gains_pub.publish(gains_message);
    pub.publish(trajectory_msg);


    ros::spinOnce();
    loop_rate1.sleep();


  }

 ROS_INFO("HOVER!");

 path.data = false;
 path_pub.publish(path);

 hover.data = true;
 hover_pub.publish(hover);

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

  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;


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

void fillGainMessage(string Gain,double Gain1[3],double Gain2[3], double Gain3[3], double Gain4[3])
{
  istringstream iss(Gain);
  vector<string> split((istream_iterator<string>(iss)),istream_iterator<string>());

  Gain1[0] = atof(split[0].c_str());
  Gain1[1] = atof(split[1].c_str());
  Gain1[2] = atof(split[2].c_str());

  Gain2[0] = atof(split[3].c_str());
  Gain2[1] = atof(split[4].c_str());
  Gain2[2] = atof(split[5].c_str());

  Gain3[0] = atof(split[6].c_str());
  Gain3[1] = atof(split[7].c_str());
  Gain3[2] = atof(split[8].c_str());

  Gain4[0] = atof(split[9].c_str());
  Gain4[1] = atof(split[10].c_str());
  Gain4[2] = atof(split[11].c_str());

};

void Control_Matrix(ifstream &Gain_file,
                    rotors_comm::GainMSG &gains_message) {
  
  string Gain_line[4];
  getline(Gain_file,Gain_line[0]);
  getline(Gain_file,Gain_line[1]);
  getline(Gain_file,Gain_line[2]);
  getline(Gain_file,Gain_line[3]);

  gains_message.header.stamp = ros::Time::now();

  fillGainMessage(Gain_line[0],&gains_message.Gain_pp[0],&gains_message.Gain_dd[0],
                        &gains_message.Gain_peta[0],&gains_message.Gain_dom[0]);

  for(int i=1; i <= 3; i++)
  {
     fillGainMessage(Gain_line[i],&gains_message.Gain_pt[0]+3*(i-1),&gains_message.Gain_dt[0]+3*(i-1)
                    ,&gains_message.Gain_p[0]+3*(i-1),&gains_message.Gain_d[0]+3*(i-1));

  }

}
