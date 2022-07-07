#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char ** argv) {

    ros::init(argc, argv, "etq_demo_goal");

    ros::NodeHandle nh;

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PointStamped>("etq/goal", 1, true);

    ros::Rate rate(10);

    int iter = 0;

    while (nh.ok()) {

        geometry_msgs::PointStamped goal_point;

        goal_point.header.stamp = ros::Time::now();
        goal_point.header.frame_id = "world";

        goal_point.point.x = 5.0;
        goal_point.point.y = 3.0;
        goal_point.point.z = 0.0;

        goal_pub.publish(goal_point);

        ros::spinOnce();

        rate.sleep();
    }

}