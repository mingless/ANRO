#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "anrobot_description/FwdKinematics.h"

class TrajectoryGenerator
{
	public:
		TrajectoryGenerator();
		~TrajectoryGenerator();
	private:
		bool method;
		ros::Duration d;
		
}
