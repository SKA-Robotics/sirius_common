#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <string>
#include <limits>

std::string input_topic = "input_map";
std::string output_topic = "output_map";

ros::Publisher publisher;
ros::Subscriber subscriber;

grid_map::GridMap globalMap;
grid_map::GridMap localMap;

void InitializeGlobalMap();
void GridMapCallback(const grid_map_msgs::GridMap& msg);
void PublishGlobalMap();
bool ShouldCopyNewValue(float original_uncertainty, float new_uncertainty, float original_timestamp, float new_timestamp);
bool ShouldClearValue(float original_uncertainty, float new_uncertainty, float original_timestamp, float new_timestamp);
void ProcessMapsAtPosition(grid_map::GridMap& original_map, grid_map::GridMap& new_map, grid_map::Position position, float new_timestamp);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gridmap_builder");
  ros::NodeHandle node_handle("~");
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
  globalMap.setGeometry(grid_map::Length(30, 30), 0.05, grid_map::Position(0, 0));
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

bool ShouldCopyNewValue(float original_uncertainty, float new_uncertainty, float original_timestamp, float new_timestamp)
{
  if (isnan(new_uncertainty))
  {
    return false;
  }
  if (isnan(original_uncertainty) || isnan(original_timestamp))
  {
    return true;
  }
  float time_diff = new_timestamp - original_timestamp;
  float uncertainty_diff = new_uncertainty - original_uncertainty;
  float uncertainty_penalty = 1.0f; // todo: make it a parameter
  float time_penalty = 1.0f;
  float penalty = uncertainty_penalty * uncertainty_diff - time_penalty * time_diff;
  if (penalty < 0.0f)
  {
    return true;
  }
  return false;
}

bool ShouldClearValue(float original_uncertainty, float new_uncertainty, float original_timestamp, float new_timestamp) {
  float uncertainty_threshold = 0.3f;
  if (original_uncertainty > uncertainty_threshold) {
    return true;
  }
  return false;
}

void ProcessMapsAtPosition(grid_map::GridMap& original_map, grid_map::GridMap& new_map, grid_map::Position position, float new_timestamp) {
    float& original_traversability = original_map.atPosition("traversability", position);
    float new_traversability = new_map.atPosition("traversability", position);
    float& original_uncertainty = original_map.atPosition("uncertainty_range", position);
    float new_uncertainty = new_map.atPosition("uncertainty_range", position);
    float& original_timestamp = original_map.atPosition("time", position);
    if (ShouldCopyNewValue(original_uncertainty, new_uncertainty, original_timestamp, new_timestamp))
    {
      original_traversability = new_traversability;
      original_uncertainty = new_uncertainty;
      original_timestamp = new_timestamp;
    }
    if (ShouldClearValue(original_uncertainty, new_uncertainty, original_timestamp, new_timestamp)) {
      original_traversability = std::numeric_limits<double>::quiet_NaN();
      original_uncertainty = std::numeric_limits<double>::quiet_NaN();
      original_timestamp = new_timestamp;
    }
}
