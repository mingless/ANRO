#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include "anrobot/FwdKinematics.h"

bool get_pose(anrobot::FwdKinematics::Request  &req,
	      anrobot::FwdKinematics::Response &res);
