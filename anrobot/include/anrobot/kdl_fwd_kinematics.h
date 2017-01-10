#include <ros/ros.h>
#include <iostream>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <math.h>
#include <stdlib.h>
#include "anrobot/FwdKinematics.h"

bool get_pose(anrobot::FwdKinematics::Request  &req,
	      anrobot::FwdKinematics::Response &res);
