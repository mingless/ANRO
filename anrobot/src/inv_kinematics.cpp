#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <anrobot/InvKinematics.h>
#include <anrobot/SetTarget.h>
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
        }

		bool get_states(anrobot::InvKinematics::Request &req,
		anrobot::InvKinematics::Response &res) {
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

			if(!is_valid(req.point)) {
				ROS_ERROR_STREAM_THROTTLE(5, "Invalid target position\n" << x
                                          << " " << y << " " << z <<"\n");
				return false;
			}
            else {
                ROS_INFO_STREAM("Target set to: [" << x << ", "
                                << y << ", " << z << "].");
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

        bool is_valid(const geometry_msgs::Point &t) {  // target_position
            double a = 2, b = 1;
            double eq = 1 - pow((t.x*t.x + t.y*t.y - a*a - b*b) / (2*a*b), 2);
			if(t.z < -3. || t.z > -0.3 || eq < 0) {
                return false;
            }
            return true;
        }

        bool set_target(anrobot::SetTarget::Request &req,
                anrobot::SetTarget::Response &res) {
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











