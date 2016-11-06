#include <ros/ros.h>
#include <iostream>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <math.h>
#include <stdlib.h>

int main(int argc,char *argv[]){
	KDL::Chain chain;	
	double pi = M_PI;
	double a1 = 2, a2 = 1, theta1, theta2, d3;
	if(argc == 4) {
		theta1 =  strtol(argv[1], NULL, 10);
		theta2 = strtol(argv[2], NULL, 10);
		d3 = strtol(argv[3], NULL, 10);
	}
	else {
		theta1 = pi/3;
		theta2 = pi/8;
		d3 = 2;
	}

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
