#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
using namespace std;


class Trajectory
{
    private:
        sensor_msgs::JointState initial, target;
        double delta[3], accel[3];

    public:
        int msg_amount;
        bool is_init = 0;
        int inc; //number of passed increments in current timer cycle
        Trajectory(ros::Publisher* pub);
        ros::Publisher* pub_ptr;
        sensor_msgs::JointState current;

        void init(sensor_msgs::JointStateConstPtr msg);
        void set_target(sensor_msgs::JointStateConstPtr target);
        bool compare_target(sensor_msgs::JointStateConstPtr input);

        void init_lin(sensor_msgs::JointStateConstPtr msg);
        void init_nonlin(sensor_msgs::JointStateConstPtr msg);

        void next_step_lin(const ros::TimerEvent& event);
        void next_step_nonlin(const ros::TimerEvent& event);

};

void Trajectory::init(sensor_msgs::JointStateConstPtr msg)
{
    msg_amount = 300;
    inc = 0;
    is_init = 1;
    current = *msg;
    initial = current;
    target = current;
}

Trajectory::Trajectory(ros::Publisher* pub)
{
    pub_ptr = pub;
    is_init = 0;
}

void Trajectory::next_step_lin(const ros::TimerEvent& event)
{
    if (++inc < msg_amount)
    {
        current.position[0] += delta[0];
        current.position[1] += delta[1];
        current.position[2] += delta[2];
    }

    current.header.stamp = ros::Time::now();
    pub_ptr->publish(current);
}

void Trajectory::next_step_nonlin(const ros::TimerEvent& event)
{
    ++inc;

    if (inc < msg_amount/3)
    {
        for(int i = 0; i < 3; ++i)
        {
            delta[i] += accel[i];
            current.position[i] += delta[i];
        }
    }

    if (inc >= msg_amount/3 && inc < msg_amount*2/3)
    {
        for(int i = 0; i < 3; ++i)
        {
            current.position[i] += delta[i];
        }
    }

    if (inc >= msg_amount*2/3 && inc < msg_amount)
    {
        for(int i = 0; i < 3; ++i)
        {
            delta[i] -= accel[i];
            current.position[i] += delta[i];
        }
    }

    current.header.stamp = ros::Time::now();
    pub_ptr->publish(current);
}

void Trajectory::set_target(sensor_msgs::JointStateConstPtr new_target)
{
    this->target = *new_target;
}

void Trajectory::init_lin(sensor_msgs::JointStateConstPtr msg)
{
    inc = 0;
    initial = current;
    set_target(msg);
    delta[0] = (target.position[0]-initial.position[0])/msg_amount;
    delta[1] = (target.position[1]-initial.position[1])/msg_amount;
    delta[2] = (target.position[2]-initial.position[2])/msg_amount;
}

void Trajectory::init_nonlin(sensor_msgs::JointStateConstPtr msg)
{
    inc = 0;
    initial = current;
    set_target(msg);
    for (int i = 0; i <3; ++i)
    {
        delta[i] = 0;
        accel[i] = 9*(target.position[i]-initial.position[i])/(2*msg_amount*msg_amount);
    }
}

void target_states_cb(const sensor_msgs::JointStateConstPtr &msg, ros::Timer *timer, Trajectory *t)
{
    if (!t->is_init)
    {
        t->init(msg);
    }
    if (t->compare_target(msg))
    {
        t->init_nonlin(msg);
        timer->start();
    }
    else
        t->current.header.stamp = ros::Time::now();
    t->pub_ptr->publish(t->current);
}

bool Trajectory::compare_target(sensor_msgs::JointStateConstPtr input)
{
    double diff = 0, sq;
    for(int i = 0; i < 3; ++i)
    {
        sq = (input->position[i] - target.position[i]);
        diff += sq*sq;
    }
    if (diff > 0.01)
        return 1;  // a != b
    else
        return 0;  // a = b
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gen_trajectory");
    ros::NodeHandle n;

    ros::Publisher trajectory_pub = n.advertise<sensor_msgs::JointState>("trajectory_joint_states", 1);

    Trajectory t(&trajectory_pub);
    ros::Timer timer = n.createTimer(ros::Duration(4./200), &Trajectory::next_step_nonlin, &t, false, false);

    ros::Subscriber get_target_state = n.subscribe<sensor_msgs::JointState>("joint_states", 100, boost::bind(target_states_cb, _1, &timer, &t));

    while(ros::ok())
    {
        if(t.inc > t.msg_amount) timer.stop();
        ros::spinOnce();
    }

    return 0;
}
