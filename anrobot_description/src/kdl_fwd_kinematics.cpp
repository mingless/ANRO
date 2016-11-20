#include "anrobot_description/kdl_fwd_kinematics.h"

bool get_pose(anrobot_description::FwdKinematics::Request  &req,
	      anrobot_description::FwdKinematics::Response &res)
{
	KDL::Chain chain;
	double a1 = 2, a2 = 1;
 	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a1, 0, 0, 0)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a2, M_PI, 0, 0)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ),KDL::Frame::DH(0, 0, 0, 0)));

	KDL::Frame endpos; 
	KDL::ChainFkSolverPos_recursive solver = KDL::ChainFkSolverPos_recursive(chain);
	KDL::JntArray joints = KDL::JntArray(3);
	
	joints(0) = req.theta1;
	joints(1) = req.theta2;
	joints(2) = req.d3;

	solver.JntToCart(joints, endpos);

	geometry_msgs::Point p;
	p.x = endpos.p.x();
	p.y = endpos.p.y();
	p.z = endpos.p.z();

	res.end_pose.position = p;
	return true;
}


int main(int argc,char **argv)
{
	ros::init(argc, argv, "kdl_fwd_kinematics");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("kdl_fwd", get_pose);
	ros::spin();	
		
	return 0;
}
