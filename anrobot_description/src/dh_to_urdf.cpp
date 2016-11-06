#include <ros/ros.h>
#include <iostream>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <math.h>

int main(){
	KDL::Chain chain;
	double pi = M_PI;
	double a1 = 2;
	double a2 = 1;
	double theta1 = pi/3;
	double theta2 = pi/8;
	double d3 = 2;

	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a1, 0, 0, theta1)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a2, pi, 0, theta2)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, 0, d3, 0)));

	double joint[3], tip[3];
	double roll, pitch, yaw;
	for (unsigned int i = 0; i < 3; ++i){
		for(int j = 0; j < 3; j++){ 
			joint[j] = chain.getSegment(i).getJoint().JointAxis().data[j];
			tip[j] = chain.getSegment(i).getFrameToTip().p.data[j];
		}
		chain.getSegment(i).getFrameToTip().M.GetRPY(roll, pitch, yaw);
		
		std::cout << "Joint " << i << " rpy : " << joint[0] << ", " << joint[1] << ", " << joint[2] << std::endl;
		std::cout << "TipFrame " << i << " rpy : " << roll << ", " << pitch << ", " << yaw << std::endl;
		std::cout << "TipFrame " << i << " xyz : " << tip[0] << ", " << tip[1] << ", " << tip[2] << std::endl;
	}

	return 0;
}
