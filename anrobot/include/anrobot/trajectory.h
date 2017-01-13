#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <anrobot/InvKinematics.h>
#include <std_srvs/SetBool.h>

class Trajectory {
    protected:
        bool _is_init;
        double delta[3], accel[3];

        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Timer timer;
        ros::Subscriber get_target;

        sensor_msgs::JointState initial, current, target;

    public:
        Trajectory();
        ~Trajectory(){};

        bool is_init();
        void init(sensor_msgs::JointStateConstPtr msg);

        bool use_lin;
        int msg_amount;
        int inc; //number of passed increments in current timer cycle

        bool compare_target(sensor_msgs::JointStateConstPtr input);

        void target_states_cb(const sensor_msgs::JointStateConstPtr &msg);
        virtual void publish_current();

        void init_inter(sensor_msgs::JointStateConstPtr msg);
        void init_lin(sensor_msgs::JointStateConstPtr msg);
        void init_nonlin(sensor_msgs::JointStateConstPtr msg);

        virtual void next_step(const ros::TimerEvent &event);
        virtual void next_step_lin(const ros::TimerEvent &event);
        virtual void next_step_nonlin(const ros::TimerEvent &event);
};

class InvTrajectory : public Trajectory{
    protected:
        ros::Publisher pub_end, pub_states;
        ros::ServiceClient end_to_joints;
        ros::ServiceClient trajectory_finished;

        geometry_msgs::Point end_target, end_current, end_initial;

    public:
        InvTrajectory();
        ~InvTrajectory(){};

        void init();

        bool compare_target(geometry_msgs::PointConstPtr input);
        bool validate_reachability(geometry_msgs::PointConstPtr input);
	    void target_states_cb(const geometry_msgs::PointConstPtr &msg);
        virtual void publish_current();
        void announce_state(bool state);

        void init_inter(geometry_msgs::PointConstPtr msg);
        void init_lin(geometry_msgs::PointConstPtr msg);
        void init_nonlin(geometry_msgs::PointConstPtr msg);

        virtual void next_step(const ros::TimerEvent &event);
        virtual void next_step_lin(const ros::TimerEvent &event);
        virtual void next_step_nonlin(const ros::TimerEvent &event);
};
