#include <ros/ros.h>
#include <anrobot/SetTarget.h>
#include <std_srvs/SetBool.h>
#include <fstream>

class TrajectoryTest {
	private:
		
		int POINTS;
		ros::NodeHandle n;
		ros::ServiceServer send_next;
		ros::ServiceClient next_point;
		double target_points [3][3];
		int inc;

		bool send_next_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
			if(!request.data) {
				ROS_ERROR_STREAM_THROTTLE(1, "Invalid point requested. Terminating.\n");
				send_next.shutdown();
			}
			if(inc < POINTS) {
				anrobot::SetTarget srv;
				srv.request.point.x = target_points[inc][0];
				srv.request.point.y = target_points[inc][1];
				srv.request.point.z = target_points[inc][2];
				next_point.call(srv);
				if(!srv.response.success) {
					ROS_ERROR_STREAM_THROTTLE(1, "Invalid point requested. Terminating.\n");
					send_next.shutdown();
				}
				inc++;
			}
			else {
				ROS_ERROR_STREAM_THROTTLE(1, "End of the path reached.\n");
				send_next.shutdown();
			}
				
		}
				
	public:
		TrajectoryTest() {
			inc = 0;
			POINTS = 3;
			target_points[0][0] = 1.;
			target_points[0][1] = 2.;
			target_points[0][2] = -1.;
			target_points[1][0] = 0.;
			target_points[1][1] = 2.;
			target_points[1][2] = -1.;
			target_points[2][0] = -1.;
			target_points[2][1] = 1.5;
			target_points[2][2] = -2.;
			send_next = n.advertiseService("trajectory_finished", &TrajectoryTest::send_next_cb, this);
			next_point = n.serviceClient<anrobot::SetTarget>("set_end_target");
		}
			

};
 


		
	



int main(int argc, char** argv) {
	ros::init(argc, argv, "trajectory_test");
	TrajectoryTest t;

	ros::spin();
}
