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
		std::queue<double> q;

		bool send_next_cb(std_srvs::SetBool::Request &request,
                          std_srvs::SetBool::Response &response) {
			if(!request.data) {
				ROS_ERROR_STREAM_THROTTLE(3, "Invalid point requested. Terminating.\n");
				send_next.shutdown();
			}
			else if(!q.empty()) {
				anrobot::SetTarget srv;
				srv.request.point.x = q.front();
				q.pop();
				srv.request.point.y = q.front();
				q.pop();
				srv.request.point.z = q.front();
				q.pop();
				next_point.call(srv);
				if(!srv.response.success) {
					ROS_ERROR_STREAM_THROTTLE(3, "Invalid point requested. Terminating.\n");
					send_next.shutdown();
				}
			}
			else {
				ROS_ERROR_STREAM_THROTTLE(3, "End of the path reached.\n");
				send_next.shutdown();
			}

		}

	public:
		TrajectoryTest() {
			std::string filename;
			n.param<std::string>("filename", filename, "/data/data1");
			std::fstream file;
			filename = ros::package::getPath("anrobot") + filename;
			file.open(filename.c_str(), std::ios::in | std::ios::binary);
			double coord;
			while(file >> coord)
				q.push(coord);
			if(q.size()%3 != 0)
				return;
			file.close();
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
