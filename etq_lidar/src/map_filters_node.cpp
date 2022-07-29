#include <ros/ros.h>

#include <filters/filter_chain.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

using namespace grid_map;

int main (int argc, char ** argv) {
  
  ros::init(argc, argv, "map_filters");
  ros::NodeHandle node("etq");
  
  filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");
  if (!filterChain.configure("grid_map_filters", node)) {
    ROS_ERROR("Could not configure the filter chain!");
    return 0;
  }

  ros::Publisher map_pub = node.advertise<grid_map_msgs::GridMap>("map", 10, true);
  
  auto mapCallback = [&](const grid_map_msgs::GridMapConstPtr &msg_in) {

    ROS_INFO("------------");
    ROS_INFO("Filtering Grid Map...");

    clock_t cstart, cend;

    GridMap map_in;
    GridMap map_out;

    GridMapRosConverter::fromMessage(*msg_in, map_in);

    cstart = clock();

    if (!filterChain.update(map_in, map_out)) {
      ROS_ERROR("Could not update the grid map filter chain!");
      return;
    }

    cend = clock();

    float t = float(cend - cstart) / float(CLOCKS_PER_SEC) * 1000.0f;

    ROS_INFO("Computing Time: %.4f ms", t );
    
    map_out.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap msg_out;
    GridMapRosConverter::toMessage(map_out, msg_out);
    map_pub.publish(msg_out);

    ROS_INFO("Filtered Map published. (Timestamp: %.5f)", msg_out.info.header.stamp.toSec());
    ROS_INFO("------------");
    
  };
  
  ros::Subscriber map_sub = node.subscribe<grid_map_msgs::GridMap>("elevation_map", 10, mapCallback);

  ros::spin();
  
  return 0; 
}
