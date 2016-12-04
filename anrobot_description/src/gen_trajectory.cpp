#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
using namespace std;


class Trajectory
{
    private:
        int time;
        //a - current/starting points, b - target points
        double a1, a2, a3, b1, b2, b3,
               lin_delta_1, lin_delta_2, lin_delta_3;
        sensor_msgs::JointState state;
        ros::Publisher* pub_ptr;
        void init();

    public:
        int msg_amount;
        int inc; //number of passed increments in current timer cycle
        Trajectory(ros::Publisher* pub);

        void init_lin(double b1, double b2, double b3);
        void next_step_lin(const ros::TimerEvent& event);

};

void Trajectory::init()
{
    time = 10;
    msg_amount = 200;
    inc = 0;
    state.position.push_back(0);
    state.position.push_back(0);
    state.position.push_back(1.65);
    state.velocity.push_back(0);
    state.velocity.push_back(0);
    state.velocity.push_back(0);
    state.effort.push_back(0);
    state.effort.push_back(0);
    state.effort.push_back(0);
    state.name.push_back("joint1");
    state.name.push_back("joint2");
    state.name.push_back("joint3");
    a1 = state.position[0]; a2 = state.position[1]; a3 = state.position[2];
}

Trajectory::Trajectory(ros::Publisher* pub)
{
    init();
    pub_ptr = pub;
}

void Trajectory::next_step_lin(const ros::TimerEvent& event)
{
    inc++;
    state.header.stamp = ros::Time::now();
    pub_ptr->publish(state);
    state.position[0] += lin_delta_1;
    state.position[1] += lin_delta_2;
    state.position[2] += lin_delta_3;
}

void Trajectory::init_lin(double b1, double b2, double b3)
{
    inc = 0;
    this->a1 = state.position[0];
    this->a2 = state.position[1];
    this->a3 = state.position[2];
    this->b1 = b1;
    this->b2 = b2;
    this->b3 = b3;
    lin_delta_1 = (b1-a1)/msg_amount;
    lin_delta_2 = (b2-a2)/msg_amount;
    lin_delta_3 = (b3-a3)/msg_amount;
}

void target_states_cb(const sensor_msgs::JointStateConstPtr &msg, ros::Timer *timer, Trajectory *t)
{
    t->init_lin(msg->position[0],msg->position[1],msg->position[2]);
    timer->start();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gen_trajectory");
    ros::NodeHandle n;

    ros::Publisher trajectory_pub = n.advertise<sensor_msgs::JointState>("trajectory_joint_states", 1);

    Trajectory t(&trajectory_pub);
    ros::Timer timer = n.createTimer(ros::Duration(4./200), &Trajectory::next_step_lin, &t, false, false);

    ros::Subscriber get_target_state = n.subscribe<sensor_msgs::JointState>("joint_states", 100, boost::bind(target_states_cb, _1, &timer, &t));

    while(ros::ok())
    {
        if(t.inc > t.msg_amount) timer.stop();
        ros::spinOnce();
    }

    return 0;
}
