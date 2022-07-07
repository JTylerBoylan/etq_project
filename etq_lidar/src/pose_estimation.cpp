#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

int main (int argc, char** argv) {

    ros::init(argc, argv, "pose_estimator");

    ros::NodeHandle node;

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";

    bool new_pose = false;

    // DEMO POSITION
    geometry_msgs::Point pos;
    pos.x = 0.0;
    pos.y = 0.0;
    pos.z = 0.25;
    pose.pose.position = pos;
    //

    auto imuCallback = [&](const sensor_msgs::ImuConstPtr &msg) {
        pose.pose.orientation = msg->orientation;
        new_pose = true;
    };

    ros::Subscriber imu_sub = node.subscribe<sensor_msgs::Imu>("imu/data", 1, imuCallback);

    ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>("etq/pose", 1, true);

    while (node.ok()) {
        if (new_pose) {
            pose.header.stamp = ros::Time::now();
            pose_pub.publish(pose);
            new_pose = false;
        }
        ros::spinOnce();
    }

}