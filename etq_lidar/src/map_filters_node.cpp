#include <filters/filter_chain.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>

using namespace grid_map;

int main (int argc, char ** argv) {
  
  ros::init(argc, argv, "map_filters");
  ros::NodeHandle node;
  
  filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");
  if (!filterChain.configure("grid_map_filters", node)) {
    ROS_ERROR("Could not configure the filter chain!");
    return 0;
  }
  
  GridMap map_in;
  GridMap map_out;
  
  auto mapCallback = [&](const grid_map_msgs::GridMapConstPtr &msg) {
    GridMapRosConverter::fromMessage(*msg, map_in);
  };
  
  ros::Subscriber map_sub = node.subscribe<grid_map_msgs::GridMap>("/etq/elevation_map", 10, mapCallback);
  ros::Publisher map_pub = node.advertise<grid_map_msgs::GridMap>("/etq/map", 10, true);
  
  ros::Rate rate(5);
  while (node.ok()) {
    
    if (!filterChain.update(map_in, map_out)) {
      ROS_ERROR("Could not update the grid map filter chain!");
      break;
    }
    
    map_out.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap msg;
    GridMapRosConverter::toMessage(map_out, msg);
    map_pub.publish(msg);

    ROS_INFO("Filtered Map published. (Timestamp: %.5f)", msg.info.header.stamp.toSec());
    
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0; 
}
