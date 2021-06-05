/*
 * grid_map_pcl_loader_node.cpp
 *
 *  Created on: Aug 26, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"

#include "grid_map_pcl/elevation_map_loader_node.hpp"

namespace gm = ::grid_map::grid_map_pcl;

ElevationMapLoaderNode::ElevationMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("elevation_map_loader", options)
{
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_elevation_map_ =
    this->create_publisher<grid_map_msgs::msg::GridMap>("output/elevation_map", durable_qos);

  this->setVerbosityLevelToDebugIfFlagSet();

  grid_map::GridMapPclLoader gridMapPclLoader(this->get_logger());
  const std::string pathToCloud = this->getPcdFilePath();

  gridMapPclLoader.loadParameters(gm::getParameterPath());
  gridMapPclLoader.loadCloudFromPcdFile(pathToCloud);

  this->processPointcloud(&gridMapPclLoader);

  grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
  gridMap.setFrameId(this->getMapFrame());

  auto msg = grid_map::GridMapRosConverter::toMessage(gridMap);
  pub_elevation_map_->publish(std::move(msg));
}

void ElevationMapLoaderNode::setVerbosityLevelToDebugIfFlagSet()
{
  bool isSetVerbosityLevelToDebug;
  this->declare_parameter("set_verbosity_to_debug", false);
  this->get_parameter("set_verbosity_to_debug", isSetVerbosityLevelToDebug);

  if (!isSetVerbosityLevelToDebug) {
    return;
  }

  auto ret =
    rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to change logging severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

std::string ElevationMapLoaderNode::getMapFrame()
{
  this->declare_parameter("map_frame", std::string("map"));

  std::string mapFrame;
  this->get_parameter("map_frame", mapFrame);
  std::cout << "map_frame: " << mapFrame << std::endl;
  return mapFrame;
}

std::string ElevationMapLoaderNode::getPcdFilePath()
{
  if (!this->has_parameter("folder_path")) {
    this->declare_parameter("folder_path", std::string(""));
  }
  this->declare_parameter("pcd_filename", "input_cloud");

  std::string inputCloudName, folderPath;
  this->get_parameter("folder_path", folderPath);
  this->get_parameter("pcd_filename", inputCloudName);
  std::string pathToCloud = folderPath + "/" + inputCloudName;
  std::cout << "pathToCloud: " << pathToCloud << std::endl;
  return pathToCloud;
}

std::string ElevationMapLoaderNode::getMapLayerName()
{
  this->declare_parameter("map_layer_name", std::string("elevation"));

  std::string mapLayerName;
  this->get_parameter("map_layer_name", mapLayerName);
  return mapLayerName;
}

void ElevationMapLoaderNode::processPointcloud(grid_map::GridMapPclLoader * gridMapPclLoader)
{
  const auto start = std::chrono::high_resolution_clock::now();
  gridMapPclLoader->preProcessInputCloud();
  gridMapPclLoader->initializeGridMapGeometryFromInputCloud();
  gm::printTimeElapsedToRosInfoStream(start, "Initialization took: ", this->get_logger());
  gridMapPclLoader->addLayerFromInputCloud(this->getMapLayerName());
  gm::printTimeElapsedToRosInfoStream(start, "Total time: ", this->get_logger());
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ElevationMapLoaderNode)
