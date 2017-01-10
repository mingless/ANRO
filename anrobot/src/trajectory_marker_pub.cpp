#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>


/*
void get_point_cb(const geometry_msgs::PointConstPtr &msg, ros::Publisher *pub) {
    visualization_msgs::Marker marker;

    marker.pose.position.x = msg->x;
    marker.pose.position.y = msg->y;
    marker.pose.position.z = msg->z;

    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    marker.ns = "point_trajectory_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.8f;
    marker.color.g = 0.8f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    pub->publish(marker);
}*/

void get_point_cb(const geometry_msgs::PointConstPtr &msg, ros::Publisher *pub) {
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = msg->x;
    pose.pose.position.y = msg->y;
    pose.pose.position.z = msg->z;

    pose.header.frame_id = "/base_link";
    pose.header.stamp = ros::Time::now();

    pub->publish(pose);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_marker_pub");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<geometry_msgs::PoseStamped>("visualization_marker", 1);
    ros::Subscriber get_point = n.subscribe<geometry_msgs::Point>("end_trajectory", 100, boost::bind(get_point_cb, _1, &marker_pub));

    ros::spin();

	return 0;
}
