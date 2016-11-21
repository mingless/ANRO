#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include "anrobot_description/FwdKinematics.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_param_client");
    if(argc != 5)
    {
        ROS_ERROR("Four params required: theta1 theta2 d3 use_kdl");
        return 1;
    }

    ros::NodeHandle n;
    ros::Rate r(1);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher joint_states = n.advertise<sensor_msgs::JointState>("set_joint_states", 1);

    ros::ServiceClient kdl_client = n.serviceClient<anrobot_description::FwdKinematics>("kdl_fwd");
    ros::ServiceClient simple_client = n.serviceClient<anrobot_description::FwdKinematics>("simple_fwd");

    anrobot_description::FwdKinematics srv;
    srv.request.theta1 = atoll(argv[1]);
    srv.request.theta2 = atoll(argv[2]);
    srv.request.d3 = atoll(argv[3]);
    srv.request.use_kdl = atoll(argv[4]);

    while (ros::ok())
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time::now();

        marker.ns = "end_pose";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        if(srv.request.use_kdl)
        {
            if(kdl_client.call(srv))
            {
                marker.pose = srv.response.end_pose;
            }
        }
        else
        {
            if(simple_client.call(srv))
            {
                marker.pose = srv.response.end_pose;
            }
        }

        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }

            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        marker_pub.publish(marker);

        r.sleep();
    }

    return 0;
}
