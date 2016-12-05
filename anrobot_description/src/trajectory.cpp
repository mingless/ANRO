#include "anrobot_description/trajectory.h"

using namespace std;

Trajectory::Trajectory(ros::NodeHandle* n, ros::Publisher* pub) {
    pub_ptr = pub;
    _is_init = 0;
    nh_ptr = n;
}

bool Trajectory::compare_target(sensor_msgs::JointStateConstPtr input) {
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
};

void Trajectory::init(sensor_msgs::JointStateConstPtr msg) {
    msg_amount = 300;
    inc = 0;
    _is_init = 1;
    current = *msg;
    initial = current;
    target = current;
}

bool Trajectory::is_init() {
    return _is_init;
}

void Trajectory::init_inter(sensor_msgs::JointStateConstPtr msg) {
    if(use_lin) this->init_lin(msg);
    else this->init_nonlin(msg);
}

void Trajectory::init_lin(sensor_msgs::JointStateConstPtr msg) {
    inc = 0;
    initial = current;
    target = *msg;
    delta[0] = (target.position[0]-initial.position[0])/msg_amount;
    delta[1] = (target.position[1]-initial.position[1])/msg_amount;
    delta[2] = (target.position[2]-initial.position[2])/msg_amount;
}

void Trajectory::init_nonlin(sensor_msgs::JointStateConstPtr msg) {
    inc = 0;
    initial = current;
    target = *msg;
    for (int i = 0; i <3; ++i)
    {
        delta[i] = 0;
        accel[i] = 9*(target.position[i]-initial.position[i])/(2*msg_amount*msg_amount);
    }
}

void Trajectory::next_step(const ros::TimerEvent& event) {
    if(use_lin) this->next_step_lin(event);
    else this->next_step_nonlin(event);
}

void Trajectory::next_step_lin(const ros::TimerEvent& event) {
    if (++inc < msg_amount)
    {
        current.position[0] += delta[0];
        current.position[1] += delta[1];
        current.position[2] += delta[2];
    }
    this->publish_current();
}

void Trajectory::next_step_nonlin(const ros::TimerEvent& event) {
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
    this->publish_current();
}

void Trajectory::publish_current() {
    current.header.stamp = ros::Time::now();
    this->pub_ptr->publish(current);
}