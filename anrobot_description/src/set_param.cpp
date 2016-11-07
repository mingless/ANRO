#include <ros/ros.h>
#include "anrobot_description/FwdKinematics.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "set_param_client");
	if(argc != 4)
	{
		return 1;
	}
	
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<anrobot_description::FwdKinematics>("simple_fwd");
	anrobot_description::FwdKinematics srv;
	srv.request.theta1 = atoll(argv[1]);
	srv.request.theta2 = atoll(argv[2]);
	srv.request.d3 = atoll(argv[3]);
	if(client.call(srv))
	{
		// do rviza
	}
	else
	{
		ROS_ERROR("Call failed.");
		return 1;
	}

	return 0;
}
