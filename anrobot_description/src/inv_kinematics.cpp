#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include "anrobot_description/InvKinematics.h"
#include <math.h>


class States
{
	private:
		ros::NodeHandle n;
		ros::Publisher inv_kin_pub;
		ros::ServiceServer inv_kin_client;
	public:
		States() {
			inv_kin_client = n.advertiseService("inv_kinematics",
                    &States::get_states, this);
		}

		bool get_states(anrobot_description::InvKinematics::Request &req,
		anrobot_description::InvKinematics::Response &res)
		{
            res.success = false;
			sensor_msgs::JointState state;
			state.name.push_back("joint1");
			state.name.push_back("joint2");
			state.name.push_back("joint3");
            for (int i = 0; i < 3; ++i) {
			    state.effort.push_back(0);
			    state.velocity.push_back(0);
            }
			double x = req.point.x;
			double y = req.point.y;
			double z = req.point.z;
			double a = 2;
			double b = 1;

			double eq = 1 - pow((x*x + y*y - a*a - b*b) / (2*a*b), 2);
			if(z < -3. || z > -0.3 || eq < 0) {
				ROS_ERROR_STREAM("Invalid target position\n" << x << " " << y << " " << z <<"\n");
				return false;
			}
			double theta2 = atan2(sqrt(eq),(x*x + y*y - a*a - b*b) / (2*a*b));
			double theta1 = atan2(y,x) - atan2(b*sin(theta2) ,a + b*cos(theta2));
			double d3 = -z;
			state.position.push_back(theta1);
			state.position.push_back(theta2);
			state.position.push_back(d3);
			res.states = state;
			res.success = true;
			return true;
		}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "inv_kinematics");
	States state;
	ros::spin();
	return 0;
}











