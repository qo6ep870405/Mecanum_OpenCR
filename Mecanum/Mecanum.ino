#include "mecanum.h"
#include "mecanum_ros.h"

Mecanum mecanum;

void setup() 
{ 
  Serial.begin(9600);
  mecanum.initMecanum();
  initROS();
}

void loop()
{
  double pose[3];
  double velocity[3];
  if ((millis() - timer[0]) >= (1000 / CONTROL_FREQUENCY)){
    mecanum.controlMecanum();
    timer[0] = millis();
  }
  
  if ((millis() - timer[1]) >= (1000 / UPDATE_FREQUENCY)){
    mecanum.updatePose();
    mecanum.updatePresentVelocity();
    mecanum.getPose(pose);
    mecanum.getPresentVelocity(velocity);
    publishOdom(pose, velocity);
    timer[1] = millis();
  }

  if ((millis() - timer[2]) >= 2000){
    mecanum.showProperties();
    timer[2] = millis();
  }
  nh.spinOnce();
}

void initROS(void)
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(cmdVelSub);
  nh.subscribe(cmdClearSub);
  nh.advertise(odomPub);
}

void cmdVelCallback(const geometry_msgs::Twist& cmdVelMsg)
{
  double cmd[3];
  cmd[0] = cmdVelMsg.linear.x;
  cmd[1] = cmdVelMsg.linear.y;
  cmd[2] = cmdVelMsg.angular.z;
  mecanum.setGoalVelocity(cmd);
}

void cmdClearCallback(const std_msgs::Bool& cmdClearMsg)
{
  mecanum.clearAll();
}

void publishOdom(double pose[3], double velocity[3])
{
  ros::Time t = nh.now();
  geometry_msgs::Quaternion odomQuat = tf::createQuaternionFromYaw(pose[2]);
  
  //set TF
  odomTrans.header.stamp = t;
  odomTrans.header.frame_id = "/odom";
  odomTrans.child_frame_id = "/base_footprint";
  odomTrans.transform.translation.x = pose[0];
  odomTrans.transform.translation.y = pose[1];
  odomTrans.transform.translation.z = 0.0;
  odomTrans.transform.rotation = odomQuat;
  broadcaster.sendTransform(odomTrans);

  //set odom
  odom.header.stamp = t;
  odom.header.frame_id = "/odom";
  odom.child_frame_id = "/base_footprint";
  odom.pose.pose.position.x = pose[0];
  odom.pose.pose.position.y = pose[1];
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odomQuat;
  odom.twist.twist.linear.x = velocity[0];
  odom.twist.twist.linear.y = velocity[1];
  odom.twist.twist.angular.z = velocity[2];
  odomPub.publish(&odom);
}
