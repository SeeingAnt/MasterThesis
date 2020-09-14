#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <ros/package.h>
#include <vector>

int main(int argc, char **argv) {

  ros::init(argc, argv, "supervisory_controller");
  ros::NodeHandle n;
  ros::Publisher c_pub = n.advertise<std_msgs::Bool>("control_flag", 1000);
  ros::Publisher f_pub = n.advertise<std_msgs::Bool>("flip_flag", 1000);

  std_msgs::Bool c_flag, f_flag;
  ros::Duration hov(12.0);
  //ros::Duration reach(12.77);
  ros::Duration reach(13.09);
  //ros::Duration flip(12.98);
  ros::Duration flip(13.30);
  ros::Duration rec(13.80);

  ros::Rate loop_rate(1000);


  ros::Time begin = ros::Time::now();

  while (ros::ok()) {

    ros::Time t_cur = ros::Time::now();
    double t_elapsed = t_cur.toSec() - begin.toSec();

    ROS_INFO("Current time: %f", t_elapsed);

    if (t_elapsed < hov.toSec() || t_elapsed >= rec.toSec()) {
      c_flag.data = true;
      f_flag.data = false;
      c_pub.publish(c_flag);
      f_pub.publish(f_flag);
      ROS_INFO("Hovering -> F = %d, C = %d.",f_flag.data,c_flag.data);
    }

    if (t_elapsed >= hov.toSec() && t_elapsed < reach.toSec()) {
      c_flag.data = true;
      f_flag.data = true;
      c_pub.publish(c_flag);
      f_pub.publish(f_flag);
      ROS_INFO("Reaching phase -> F = %d, C = %d.",f_flag.data,c_flag.data);
    }

    if (t_elapsed >= reach.toSec() && t_elapsed < flip.toSec()) {
      c_flag.data = true;
      f_flag.data = true;
      c_pub.publish(c_flag);
      f_pub.publish(f_flag);
      ROS_INFO("Flip phase -> F = %d, C = %d.",f_flag.data,c_flag.data);
    }

    if (t_elapsed >= flip.toSec() && t_elapsed < rec.toSec()) {
      c_flag.data = true;
      f_flag.data = true;
      c_pub.publish(c_flag);
      f_pub.publish(f_flag);
      ROS_INFO("Recovery phase -> F = %d, C = %d.",f_flag.data,c_flag.data);
    }

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
