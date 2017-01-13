#include <ros/ros.h>
#include <anrobot/SetTarget.h>
#include <std_srvs/SetBool.h>
#include <queue>
#include <fstream>
#include <string>
#include <ros/package.h>

class TrajectoryTest {
	private:
		ros::NodeHandle n;
		ros::ServiceServer send_next;
		ros::ServiceClient next_point;
		std::vector<double> points;

		bool send_next_cb(std_srvs::SetBool::Request &request,
                          std_srvs::SetBool::Response &response) {
			if(!request.data) {
				ROS_ERROR_STREAM_THROTTLE(3, "Invalid point requested. Terminating.\n");
				send_next.shutdown();
                return false;
			}
			else if(!points.empty()) {
				anrobot::SetTarget srv;
				srv.request.point.x = points.at(0);
				srv.request.point.y = points.at(1);
				srv.request.point.z = points.at(2);
				points.erase(points.begin(), points.begin()+3);
				next_point.call(srv);
				if(!srv.response.success) {
					ROS_ERROR_STREAM_THROTTLE(3, "Invalid point requested. Terminating.\n");
					send_next.shutdown();
                    return false;
				}
                return true;
			}
			else {
				ROS_INFO_STREAM_THROTTLE(3, "End of the path reached.\n");
				send_next.shutdown();
                return false;
		    }
		}

	public:
		TrajectoryTest() {
            if(!n.hasParam("points")) {
                ROS_ERROR_STREAM("Failed to retrieve target points. Shutting down.");
                return;
            }
			n.getParam("points", points);
			if(points.size()%3 != 0)
				return;
			send_next = n.advertiseService("trajectory_finished",
                                            &TrajectoryTest::send_next_cb,
                                            this);
			next_point = n.serviceClient<anrobot::SetTarget>("set_end_target");
		}
};



int main(int argc, char** argv) {
	ros::init(argc, argv, "trajectory_test");
	TrajectoryTest t;
	ros::spin();
}
