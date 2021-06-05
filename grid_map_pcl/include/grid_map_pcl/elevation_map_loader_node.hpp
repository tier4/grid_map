#ifndef ELEVATION_MAP_LOADER_NODE_HPP_
#define ELEVATION_MAP_LOADER_NODE_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class ElevationMapLoaderNode : public rclcpp::Node
{
public:
  explicit ElevationMapLoaderNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_elevation_map_;
  void setVerbosityLevelToDebugIfFlagSet();
  std::string getMapFrame();
  std::string getPcdFilePath();
  std::string getMapLayerName();
  void processPointcloud(grid_map::GridMapPclLoader * gridMapPclLoader);
};

#endif  // ELEVATION_MAP_LOADER_NODE_HPP_
