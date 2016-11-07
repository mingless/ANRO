#include "anrobot_description/simple_fwd_kinematics.h"

bool get_pose(anrobot_description::FwdKinematics::Request  &req,
	      anrobot_description::FwdKinematics::Response &res)
{
	double c1 = cos(req.theta1), c2 = cos(req.theta2),
	       s1 = sin(req.theta1), s2 = sin(req.theta2);
	geometry_msgs::Point p;
	p.x = 2*c1 + c1*c2 - s1*s2;
	p.y = 2*s1 + c1*s2 + c2*s1;
	p.z = -req.d3;

	res.end_pose.position = p;
	return true;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "simple_fwd_kinematics");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("simple_fwd", get_pose);
	ros::spin();
	return 0;
}
