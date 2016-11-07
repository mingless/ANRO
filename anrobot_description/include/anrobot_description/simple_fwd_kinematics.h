#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include "anrobot_description/FwdKinematics.h"

bool get_pose(anrobot_description::FwdKinematics::Request  &req,
	      anrobot_description::FwdKinematics::Response &res);
