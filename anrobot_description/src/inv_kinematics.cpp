#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <anrobot_description/InvKinematics.h>
#include <anrobot_description/SetTarget.h>
#include <math.h>


class States
{
	private:
		ros::NodeHandle n;
		ros::Publisher target_pub;
		ros::ServiceServer inv_kin, set_end_target;
	public:
		States() {
			inv_kin = n.advertiseService("inv_kinematics",
                    &States::get_states, this);
			set_end_target = n.advertiseService("set_end_target",
                    &States::set_target, this);
            target_pub = n.advertise<geometry_msgs::Point>("target_end", 1);
            ROS_WARN_ONCE("Target position not set.");
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
				ROS_ERROR_STREAM_THROTTLE(1, "Invalid target position\n" << x << " " << y << " " << z <<"\n");
				return false;
			}
			double theta2 = atan2(sqrt(eq),(x*x + y*y - a*a - b*b) / (2*a*b));
			double theta1 = atan2(y,x) - atan2(b*sin(theta2), a + b*cos(theta2));
			double d3 = -z;
			state.position.push_back(theta1);
			state.position.push_back(theta2);
			state.position.push_back(d3);
			res.states = state;
			res.success = true;
			return true;
		}

        bool set_target(anrobot_description::SetTarget::Request &req,
                anrobot_description::SetTarget::Response &res) {
            res.success = false;
            target_pub.publish(req.point);
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











