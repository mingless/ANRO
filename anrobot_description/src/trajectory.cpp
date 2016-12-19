#include <anrobot_description/trajectory.h>

Trajectory::Trajectory() {
    _is_init = 0;
    n.param("use_lin", use_lin, true);

    pub = n.advertise<sensor_msgs::JointState>("trajectory_joint_states", 1);

    timer = n.createTimer(ros::Duration(4./200),
            &Trajectory::next_step, this, false, false);

    get_target_state = n.subscribe<sensor_msgs::JointState>(
            "joint_states", 100, &Trajectory::target_states_cb, this);
}

bool Trajectory::compare_target(sensor_msgs::JointStateConstPtr input) {
    double diff = 0, sq;
    for(int i = 0; i < 3; ++i) {
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
    n.param("use_lin", use_lin, true);

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
    for (int i = 0; i <3; ++i) {
        delta[i] = 0;
        accel[i] = 9*(target.position[i]-initial.position[i])/(2*msg_amount*msg_amount);
    }
}

void Trajectory::next_step(const ros::TimerEvent& event) {
    if(use_lin) this->next_step_lin(event);
    else this->next_step_nonlin(event);

    if(inc > msg_amount) {
        timer.stop();
        return;
    }
}

void Trajectory::next_step_lin(const ros::TimerEvent& event) {
    if (++inc < msg_amount) {
        current.position[0] += delta[0];
        current.position[1] += delta[1];
        current.position[2] += delta[2];
    }
    this->publish_current();
}

void Trajectory::next_step_nonlin(const ros::TimerEvent& event) {
    ++inc;

    if (inc < msg_amount/3) {
        for(int i = 0; i < 3; ++i) {
            delta[i] += accel[i];
            current.position[i] += delta[i];
        }
    }

    if (inc >= msg_amount/3 && inc < msg_amount*2/3) {
        for(int i = 0; i < 3; ++i) {
            current.position[i] += delta[i];
        }
    }

    if (inc >= msg_amount*2/3 && inc < msg_amount) {
        for(int i = 0; i < 3; ++i) {
            delta[i] -= accel[i];
            current.position[i] += delta[i];
        }
    }
    this->publish_current();
}

void Trajectory::publish_current() {
    current.header.stamp = ros::Time::now();
    this->pub.publish(current);
}

void Trajectory::target_states_cb(const sensor_msgs::JointStateConstPtr &msg)
{
    if (!is_init()) {
        init(msg);
    }
    if (compare_target(msg)) {
        init_inter(msg);
        timer.start();
    }
    else {
        publish_current();
    }
}

InvTrajectory::InvTrajectory() {
    _is_init = 0;
    n.param("use_lin", use_lin, true);

    pub = n.advertise<sensor_msgs::JointState>("end_trajectory", 1);

    timer = n.createTimer(ros::Duration(4./200),
            &InvTrajectory::next_step, this, false, false);

    get_target = n.subscribe<geometry_msgs::Point>(
            "target_end", 100, &InvTrajectory::target_states_cb, this);

    end_to_joints = n.serviceClient<anrobot_description::InvKinematics>(
            "inv_kinematics");
}

bool InvTrajectory::compare_target(geometry_msgs::PointConstPtr input) {
    double diff = 0;
    {
        diff += (input->x - end_target.x)*(input->x - end_target.x);
        diff += (input->y - end_target.y)*(input->y - end_target.y);
        diff += (input->z - end_target.z)*(input->z - end_target.z);
    }
    if (diff > 0.01)
        return 1;  // a != b
    else
        return 0;  // a = b
};

void InvTrajectory::init(geometry_msgs::PointConstPtr msg) {
    msg_amount = 300;
    inc = 0;
    _is_init = 1;
    end_current = *msg;
    end_initial = end_current;
    end_target = end_current;
}

bool InvTrajectory::is_init() {
    return _is_init;
}

void InvTrajectory::init_inter(geometry_msgs::PointConstPtr msg) {
    n.param("use_lin", use_lin, true);

    if(use_lin) this->init_lin(msg);
    else this->init_nonlin(msg);
}

void InvTrajectory::publish_current() {
    anrobot_description::InvKinematics srv;
    srv.request.point = end_current;
    end_to_joints.call(srv);
    this->pub.publish(end_current);
    if(srv.response.success) {
        this->pub.publish(srv.response.states);
    }
}

void InvTrajectory::target_states_cb(const geometry_msgs::PointConstPtr &msg)
{
    if (!is_init()) {
        init(msg);
    }
    if (compare_target(msg)) {
        init_inter(msg);
        timer.start();
    }
    else {
        publish_current();
    }
}

void InvTrajectory::init_lin(geometry_msgs::PointConstPtr msg) {
    inc = 0;
    end_initial = end_current;
    end_target = *msg;
    delta[0] = (end_target.x-end_initial.x)/msg_amount;
    delta[1] = (end_target.y-end_initial.y)/msg_amount;
    delta[2] = (end_target.z-end_initial.z)/msg_amount;
}

void InvTrajectory::init_nonlin(geometry_msgs::PointConstPtr msg) {
    inc = 0;
    end_initial = end_current;
    end_target = *msg;
    {
        delta[0] = 0;
        accel[0] = 9*(end_target.x-end_initial.x)/(2*msg_amount*msg_amount);
        delta[1] = 0;
        accel[1] = 9*(end_target.y-end_initial.y)/(2*msg_amount*msg_amount);
        delta[2] = 0;
        accel[2] = 9*(end_target.z-end_initial.z)/(2*msg_amount*msg_amount);
    }
}

void InvTrajectory::next_step(const ros::TimerEvent& event) {
    if(use_lin) this->next_step_lin(event);
    else this->next_step_nonlin(event);

    if(inc > msg_amount) {
        timer.stop();
        return;
    }
}

void InvTrajectory::next_step_lin(const ros::TimerEvent& event) {
    if (++inc < msg_amount) {
        end_current.x += delta[0];
        end_current.y += delta[1];
        end_current.z += delta[2];
    }
    this->publish_current();
}

void InvTrajectory::next_step_nonlin(const ros::TimerEvent& event) {
    ++inc;

    if (inc < msg_amount/3) {
            delta[0] += accel[0];
            end_current.x += delta[0];
            delta[1] += accel[1];
            end_current.y += delta[1];
            delta[2] += accel[2];
            end_current.z += delta[2];
    }

    if (inc >= msg_amount/3 && inc < msg_amount*2/3) {
            end_current.x += delta[0];
            end_current.y += delta[1];
            end_current.z += delta[2];
    }

    if (inc >= msg_amount*2/3 && inc < msg_amount) {
            delta[0] -= accel[0];
            delta[1] -= accel[1];
            delta[2] -= accel[2];
            end_current.x += delta[0];
            end_current.y += delta[1];
            end_current.z += delta[2];
    }
    this->publish_current();
}
