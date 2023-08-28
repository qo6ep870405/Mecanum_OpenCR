#ifndef MECANUM_ROS_H_
#define MECANUM_ROS_H_

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

void initROS(void);
void cmdVelCallback(const geometry_msgs::Twist& cmdVelMsg);
void cmdClearCallback(const std_msgs::Bool& cmdClearMsg);

void publishOdom(void);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Bool> cmdClearSub("cmd_clear", cmdClearCallback);

nav_msgs::Odometry odom;
ros::Publisher odomPub("odom", &odom);

geometry_msgs::TransformStamped odomTrans;
tf::TransformBroadcaster broadcaster;

#endif
