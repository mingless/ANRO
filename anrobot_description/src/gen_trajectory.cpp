#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
using namespace std;


class Trajectory {
	private:
		int time;
		int msg_amount;
		int inc;
		double a1, a2, a3, b1, b2, b3,
		       lin_delta_1, lin_delta_2, lin_delta_3;
		sensor_msgs::JointState state;
		ros::Publisher* pub_ptr;
		void init(); 
	public:
		Trajectory(ros::Publisher* pub, double a1, double a2, double a3, double b1, double b2, double b3);
	
		
		void next_step_lin(const ros::TimerEvent& event) 
		{
			inc++;
			pub_ptr->publish(state);
			state.position[0] += lin_delta_1;
			state.position[1] += lin_delta_2;	
			state.position[2] += lin_delta_3;
			if(inc > msg_amount) {
				inc = 0;
				lin_delta_1 = -1 * lin_delta_1;
				lin_delta_2 = -1 * lin_delta_3;
				lin_delta_3 = -1 * lin_delta_3;
			}
		}
		

};

void Trajectory::init() {
	time = 10;
	msg_amount = 200;	
	inc = 0;
	state.position.push_back(0);
	state.position.push_back(0);
	state.position.push_back(0.3);
	state.velocity.push_back(0);
	state.velocity.push_back(0);
	state.velocity.push_back(0);
	state.effort.push_back(0);
	state.effort.push_back(0);
	state.effort.push_back(0);
	state.name.push_back("joint1");
	state.name.push_back("joint2");
	state.name.push_back("joint3");
}
Trajectory::Trajectory(ros::Publisher* pub, double a1, double a2, double a3, double b1, double b2, double b3) {
	init();	
	this->a1 = a1;
	this->a2 = a2;
	this->a3 = a3;
	this->b1 = b1;
	this->b2 = b2;
	this->b3 = b3;
	lin_delta_1 = (b1-a1)/msg_amount;
	lin_delta_2 = (b2-a2)/msg_amount;
	lin_delta_3 = (b3-a3)/msg_amount;
	state.position[0] = a1;
	state.position[1] = a2;
	state.position[2] = a3;
	
	pub_ptr = pub;
}

int main(int argc, char **argv)
{
	cout << "main start\n";
	ros::init(argc, argv, "gen_trajectory");
	ros::NodeHandle n;
	ros::Publisher trajectory_pub = n.advertise<sensor_msgs::JointState>("trajectory_joint_states", 1);
	if(argc < 7) return 1;
	cout << "init t\n";
	Trajectory t(&trajectory_pub, atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]));
	cout << "end init t\n";
	ros::Timer timer = n.createTimer(ros::Duration(4./200), &Trajectory::next_step_lin, &t);
	
	ros::spin();
	

	return 0;
}


