#ifndef KINOVA_TELEOP_H
#define KINOVA_TELEOP_H

#include <ros/ros.h>

#include <kinova_msgs/Stop.h>
#include <kinova_msgs/Start.h>
#include <kinova_msgs/HomeArm.h>

#include <kinova_msgs/FullJointVelocity.h>
#include <kinova_msgs/FullPoseVelocity.h>

#include <sensor_msgs/Joy.h>

//Control modes
#define CARTESIAN_CONTROL 0 
#define JOINT_CONTROL 1


#define MAX_TRANS_VEL .175
#define MAX_ANG_VEL 1.047
#define MAX_ROT_VEL_WRIST 12.000
#define MAX_ROT_VEL_ARM 8.000
#define MAX_FINGER_VEL 1500.0


class kinova_teleop
{
  public:

  kinova_teleop();

  void publish_velocity();

  private:

  void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_, ph_; 

  ros::Publisher joint_cmd_; 
  ros::Publisher cartesian_cmd_; 
  ros::Publisher finger_cmd_;
  ros::Subscriber joy_sub_; 

  ros::ServiceClient stop_client_;
  ros::ServiceClient start_client_;
  ros::ServiceClient homing_client_;
  
  kinova_msgs::FullJointVelocity full_joint_vel_;
  kinova_msgs::FullPoseVelocity full_cartesian_vel_; 

  int mode_;

  double linear_throttle_factor_; 
  double angular_throttle_factor_; 
  double finger_throttle_factor_; 


};


int main(int argc, char **argv);


#endif
