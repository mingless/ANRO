#include <anrobot_description/trajectory.h>
#include <anrobot_description/InvKinematics.h>


void target_states_cb(const sensor_msgs::JointStateConstPtr &msg, ros::Timer *timer, Trajectory *t)
{
    if (!t->is_init()) {
        t->init(msg);
    }
    if (t->compare_target(msg)) {
        t->init_inter(msg);
        timer->start();
    }
    else {
        t->publish_current();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inv_gen_trajectory");
    ros::NodeHandle n;

    ros::Publisher trajectory_pub = n.advertise<sensor_msgs::JointState>("trajectory_joint_states", 1);
    ros::ServiceClient get_joints_from_inv = n.serviceClient<anrobot_description::InvKinematics>("inv_kinematics");

    Trajectory t(&n, &trajectory_pub);
    n.param("use_lin", t.use_lin, true);

    ros::Timer timer = n.createTimer(ros::Duration(4./200), &Trajectory::next_step, &t, false, false);

    ros::Subscriber get_target_state = n.subscribe<sensor_msgs::JointState>("joint_states", 100, boost::bind(target_states_cb, _1, &timer, &t));

    while(ros::ok()) {
        n.param("use_lin", t.use_lin, true);
        if(t.inc > t.msg_amount) timer.stop();
        ros::spinOnce();
    }

    return 0;
}
