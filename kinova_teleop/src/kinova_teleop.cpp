#include <kinova_teleop/kinova_teleop.h>

using namespace std;

kinova_teleop::kinova_teleop():
  ph_("~")
{	
	joint_cmd_ = ph_.advertise<kinova_msgs::FullJointVelocity>("/joint_velocity",1);
	cartesian_cmd_ = ph_.advertise<kinova_msgs::FullPoseVelocity>("/cartesian_velocity", 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &kinova_teleop::joy_callback, this);
	
	stop_client_ = nh_.serviceClient<kinova_msgs::Stop>("/stop");
	start_client_ = nh_.serviceClient<kinova_msgs::Start>("/start"); 
	homing_client_ = nh_.serviceClient<kinova_msgs::HomeArm>("/home_arm");;
	
	ph_.param<double>("linear_throttle_factor", linear_throttle_factor_, 1.0);
	ph_.param<double>("angular_throttle_factor", angular_throttle_factor_, 1.0);
	ph_.param<double>("finger_throttle_factor", finger_throttle_factor_, 1.0);
	
	mode_ = JOINT_CONTROL;
	
	ROS_INFO("KINOVA joystick started");
}

void kinova_teleop::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	if (joy->buttons.at(6) == 1)
	{
		kinova_msgs::Stop srv_stop;
		srv_stop.request;
		if (stop_client_.call(srv_stop))
		{
			ROS_INFO("%s" , srv_stop.response.stop_result.c_str());
		}
		else 
		{
			ROS_INFO("Failed to call STOP service");
		}
	}
	  
	if (joy->buttons.at(7) == 1)
	{
		kinova_msgs::Start srv_start;
		srv_start.request;
		if (start_client_.call(srv_start))
		{
			ROS_INFO("%s" , srv_start.response.start_result.c_str());
		}
		else 
		{
			ROS_INFO("Failed to call START service");
		}
	}	
		
	if (joy->buttons.at(3) == 1)
	{
		kinova_msgs::HomeArm srv_homing;
		srv_homing.request;
		if (homing_client_.call(srv_homing))
		{
			ROS_INFO("%s" , srv_homing.response.homearm_result.c_str());
		}
		else 
		{
			ROS_INFO("Failed to call HomeArm service");
		}
	}
	   
	switch (mode_)
	{
		case CARTESIAN_CONTROL:
		
		full_cartesian_vel_.twist_linear_x = joy->axes.at(0) * MAX_TRANS_VEL * linear_throttle_factor_;
		full_cartesian_vel_.twist_linear_y = joy->axes.at(1) * MAX_TRANS_VEL * linear_throttle_factor_;
		full_cartesian_vel_.twist_linear_z = joy->axes.at(3) * MAX_TRANS_VEL * linear_throttle_factor_;
		
		full_cartesian_vel_.twist_angular_x = joy->axes.at(6) * MAX_ANG_VEL * angular_throttle_factor_;
		full_cartesian_vel_.twist_angular_y = joy->axes.at(7) * MAX_ANG_VEL * angular_throttle_factor_;
		full_cartesian_vel_.twist_angular_z = joy->axes.at(4) * MAX_ANG_VEL * angular_throttle_factor_;
		    
		if (joy->buttons.at(5) == 0)
		{
			full_cartesian_vel_.finger1 = joy->buttons.at(4) * MAX_FINGER_VEL * finger_throttle_factor_;
			full_cartesian_vel_.finger2 = joy->buttons.at(4) * MAX_FINGER_VEL * finger_throttle_factor_;
			full_cartesian_vel_.finger3 = joy->buttons.at(4) * MAX_FINGER_VEL * finger_throttle_factor_;
		}
		else if (joy->buttons.at(5) ==1)
		{
			full_cartesian_vel_.finger1 = -joy->buttons.at(5) * MAX_FINGER_VEL * finger_throttle_factor_;
			full_cartesian_vel_.finger2 = -joy->buttons.at(5) * MAX_FINGER_VEL * finger_throttle_factor_;
			full_cartesian_vel_.finger3 = -joy->buttons.at(5) * MAX_FINGER_VEL * finger_throttle_factor_;
		}

//		ROS_INFO("Cartesian velocity is : %f, %f, %f, %f, %f, %f", 
//                cartesian_vel.twist.linear.x, cartesian_vel.twist.linear.y, cartesian_vel.twist.linear.z,
//                cartesian_vel.twist.angular.x, cartesian_vel.twist.angular.y, cartesian_vel.twist.angular.z);

		if (joy->buttons.at(JOINT_CONTROL) == 1)
		{
			full_cartesian_vel_.twist_linear_x = 0.0;
			full_cartesian_vel_.twist_linear_y = 0.0;
			full_cartesian_vel_.twist_linear_z = 0.0;
		
			full_cartesian_vel_.twist_angular_x = 0.0;
			full_cartesian_vel_.twist_angular_y = 0.0;
			full_cartesian_vel_.twist_angular_z = 0.0;
			
			full_cartesian_vel_.finger1 = 0.0;
			full_cartesian_vel_.finger2 = 0.0;
			full_cartesian_vel_.finger3 = 0.0;
            
			cartesian_cmd_.publish(full_cartesian_vel_);
			mode_ = JOINT_CONTROL;
			
			ROS_INFO("Activated JOINT control mode");
		}
			
		break;
			
		case JOINT_CONTROL:
		
		full_joint_vel_.joint1 = joy->axes.at(0) * MAX_ROT_VEL_ARM * angular_throttle_factor_;
		full_joint_vel_.joint2 = joy->axes.at(1) * MAX_ROT_VEL_ARM * angular_throttle_factor_;
		full_joint_vel_.joint3 = joy->axes.at(3) * MAX_ROT_VEL_ARM * angular_throttle_factor_;
		full_joint_vel_.joint4 = joy->axes.at(4) * MAX_ROT_VEL_WRIST * angular_throttle_factor_;
		full_joint_vel_.joint5 = joy->axes.at(6) * MAX_ROT_VEL_WRIST * angular_throttle_factor_;
		full_joint_vel_.joint6 = joy->axes.at(7) * MAX_ROT_VEL_WRIST * angular_throttle_factor_;
			
		if (joy->buttons.at(5) == 0 )
		{
			full_joint_vel_.finger1 = joy->buttons.at(4) * MAX_FINGER_VEL * finger_throttle_factor_;
			full_joint_vel_.finger2 = joy->buttons.at(4) * MAX_FINGER_VEL * finger_throttle_factor_;
			full_joint_vel_.finger3 = joy->buttons.at(4) * MAX_FINGER_VEL * finger_throttle_factor_;
		}
		else if (joy->buttons.at(5) == 1)
		{
			full_joint_vel_.finger1 = -joy->buttons.at(5) * MAX_FINGER_VEL * finger_throttle_factor_;
			full_joint_vel_.finger2 = -joy->buttons.at(5) * MAX_FINGER_VEL * finger_throttle_factor_;
			full_joint_vel_.finger3 = -joy->buttons.at(5) * MAX_FINGER_VEL * finger_throttle_factor_;
		}
				
//		ROS_INFO("Joint velocity is : %f, %f, %f, %f, %f, %f", 
//                joint_vel.joint1, joint_vel.joint2, joint_vel.joint3,
//                joint_vel.joint4, joint_vel.joint5, joint_vel.joint6);
	    
		if (joy->buttons.at(CARTESIAN_CONTROL) == 1)
		{
			full_joint_vel_.joint1 = 0.0;
			full_joint_vel_.joint2 = 0.0;
			full_joint_vel_.joint3 = 0.0;
			full_joint_vel_.joint4 = 0.0;
			full_joint_vel_.joint5 = 0.0;
			full_joint_vel_.joint6 = 0.0;
		
			full_joint_vel_.finger1 = 0.0;
			full_joint_vel_.finger2 = 0.0;
			full_joint_vel_.finger3 = 0.0;

			joint_cmd_.publish(full_joint_vel_);
			mode_ = CARTESIAN_CONTROL;
			
			ROS_INFO("Activated CARTESIAN control mode");
		}
		
		break;
			
	}
}

void kinova_teleop::publish_velocity()
{		
	switch (mode_)
	{
		case CARTESIAN_CONTROL:
		cartesian_cmd_.publish(full_cartesian_vel_);
		break;
		
		case JOINT_CONTROL:
		joint_cmd_.publish(full_joint_vel_);
		break;
			
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinova_teleop");
	kinova_teleop joystick;
	ros::Rate loop_rate(60);
	
	while (ros::ok())
	{
		joystick.publish_velocity();
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

