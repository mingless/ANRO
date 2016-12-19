#include <anrobot_description/trajectory.h>
#include <anrobot_description/InvKinematics.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inv_gen_trajectory");
    InvTrajectory inv;
    ros::spin();
    return 0;
}
