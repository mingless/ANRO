#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

using namespace std;

class Trajectory
{
    private:
        bool _is_init;
        double delta[3], accel[3];
        ros::NodeHandle* nh_ptr;
        ros::Publisher* pub_ptr;
        sensor_msgs::JointState initial, current, target;

    public:
        Trajectory(ros::NodeHandle *n, ros::Publisher *pub);
        ~Trajectory(){};

        bool is_init();
        void init(sensor_msgs::JointStateConstPtr msg);

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

