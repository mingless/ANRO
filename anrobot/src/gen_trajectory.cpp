#include <anrobot/trajectory.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gen_trajectory");
    Trajectory t;
    ros::spin();
    return 0;
}
