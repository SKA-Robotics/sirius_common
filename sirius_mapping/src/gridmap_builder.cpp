#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <string>
#include <limits>

std::string input_topic = "input_map";
std::string output_topic = "output_map";
float map_size, map_resolution, no_overwrite_time, uncertainty_threshold;

ros::Publisher publisher;
ros::Subscriber subscriber;

grid_map::GridMap globalMap;
grid_map::GridMap localMap;

void InitializeGlobalMap();
void GridMapCallback(const grid_map_msgs::GridMap& msg);
void PublishGlobalMap();
void ProcessMapsAtPosition(grid_map::GridMap& original_map, grid_map::GridMap& new_map, grid_map::Position position, float new_timestamp);
void UpdateCell(grid_map::GridMap& map, grid_map::Position position, float new_traversability, float new_uncertainty, float new_timestamp);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gridmap_builder");
  ros::NodeHandle node_handle("~");
  node_handle.param<float>("map_size", map_size, 30.0);
  node_handle.param<float>("map_resolution", map_resolution, 0.05);
  node_handle.param<float>("no_overwrite_time", no_overwrite_time, 7.0);
  node_handle.param<float>("uncertainty_threshold", uncertainty_threshold, 1.0);
  InitializeGlobalMap();
  subscriber = node_handle.subscribe(input_topic, 1, GridMapCallback);
  publisher = node_handle.advertise<grid_map_msgs::GridMap>(output_topic, 5);
  ros::spin();
  return 0;
}

void InitializeGlobalMap()
{
  globalMap = grid_map::GridMap({ "traversability", "uncertainty_range", "time" });
  globalMap.setFrameId("map");
  globalMap.setGeometry(grid_map::Length(map_size, map_size), map_resolution, grid_map::Position(0, 0));
}

void GridMapCallback(const grid_map_msgs::GridMap& msg)
{
  grid_map::GridMapRosConverter::fromMessage(msg, localMap, { "traversability", "uncertainty_range" });

  for (grid_map::GridMapIterator iterator(localMap); !iterator.isPastEnd(); ++iterator)
  {
    grid_map::Position position;
    localMap.getPosition(iterator.getUnwrappedIndex(), position);
    if (!globalMap.isInside(position))
    {
      continue;
    }
    ProcessMapsAtPosition(globalMap, localMap, position, msg.info.header.stamp.toSec());
  }
  PublishGlobalMap();
}

void PublishGlobalMap()
{
  ros::Time time = ros::Time::now();
  globalMap.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(globalMap, message);
  publisher.publish(message);
}

void ProcessMapsAtPosition(grid_map::GridMap& original_map, grid_map::GridMap& new_map, grid_map::Position position, float new_timestamp) {
    float original_traversability = original_map.atPosition("traversability", position);
    float new_traversability = new_map.atPosition("traversability", position);
    float original_uncertainty = original_map.atPosition("uncertainty_range", position);
    float new_uncertainty = new_map.atPosition("uncertainty_range", position);
    float original_timestamp = original_map.atPosition("time", position);

    if (isnan(original_uncertainty) ||
        (new_timestamp > original_timestamp + no_overwrite_time) ||
        (new_uncertainty < original_uncertainty)) {
      if (new_uncertainty < uncertainty_threshold) {
        UpdateCell(original_map, position, new_traversability, new_uncertainty, new_timestamp);
      } else {
        UpdateCell(original_map, position, std::numeric_limits<float>::quiet_NaN(), new_uncertainty, new_timestamp);
      }
    }
}

void UpdateCell(grid_map::GridMap& map, grid_map::Position position, float new_traversability, float new_uncertainty, float new_timestamp) {
    map.atPosition("traversability", position) = new_traversability;
    map.atPosition("uncertainty_range", position) = new_uncertainty;
    map.atPosition("time", position) = new_timestamp;
}