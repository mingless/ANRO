#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

using namespace std;

class Trajectory
{
    private:
        bool _is_init;
	bool mode; //1 for joint trajectory, 2 for position
        double delta[3], accel[3];
	ros::NodeHandle n; 
        ros::Publisher pub;
	ros::Timer timer;
	ros::Subscriber get_target_state;
        sensor_msgs::JointState initial, current, target;

    public:
        Trajectory();
        ~Trajectory(){};

        bool is_init();
        void init(sensor_msgs::JointStateConstPtr msg);
	void target_states_cb(const sensor_msgs::JointStateConstPtr &msg);
        bool use_lin;
        int msg_amount;
        int inc; //number of passed increments in current timer cycle

        bool compare_target(sensor_msgs::JointStateConstPtr input);
        void publish_current();

        void next_step(const ros::TimerEvent &event);
        void next_step_lin(const ros::TimerEvent &event);
        void next_step_nonlin(const ros::TimerEvent &event);

        void init_inter(sensor_msgs::JointStateConstPtr msg);
        void init_lin(sensor_msgs::JointStateConstPtr msg);
        void init_nonlin(sensor_msgs::JointStateConstPtr msg);
};

