#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "world2lidar_transformer");

    ros::NodeHandle node("etq");

    tf2_ros::TransformBroadcaster br;

    auto poseCallback = [&](const geometry_msgs::PoseStampedConstPtr &msg) {

        geometry_msgs::TransformStamped ts;

        ts.header.frame_id = "world";
        ts.header.stamp = msg->header.stamp;
        ts.child_frame_id = "laser";
        ts.transform.translation.x = msg->pose.position.x;
        ts.transform.translation.y = msg->pose.position.y;
        ts.transform.translation.z = msg->pose.position.z;
        ts.transform.rotation = msg->pose.orientation;

        br.sendTransform(ts);

    };

    ros::Subscriber pose_sub = node.subscribe<geometry_msgs::PoseStamped>("pose", 1, poseCallback);

    ros::spin();

    return 0;
}