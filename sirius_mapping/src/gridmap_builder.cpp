#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <string>

std::string input_topic = "/traversability_estimation/traversability_map";
std::string output_topic = "/constructed_map";

ros::Publisher publisher;
ros::Subscriber subscriber;

grid_map::GridMap globalMap;
grid_map::GridMap localMap;

void InitializeGlobalMap();
void GridMapCallback(const grid_map_msgs::GridMap& msg);
void PublishGlobalMap();
bool CompareGridNode(float original_uncertainty, float new_uncertainty, float original_timestamp, float new_timestamp);

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
  globalMap = grid_map::GridMap({ "elevation", "traversability", "uncertainty_range", "time" });
  globalMap.setFrameId("map");
  globalMap.setGeometry(grid_map::Length(30, 30), 0.05, grid_map::Position(0, 0));
}

void GridMapCallback(const grid_map_msgs::GridMap& msg)
{
  grid_map::GridMapRosConverter::fromMessage(msg, localMap, { "elevation", "traversability", "uncertainty_range" });

  for (grid_map::GridMapIterator iterator(localMap); !iterator.isPastEnd(); ++iterator)
  {
    grid_map::Position position;
    localMap.getPosition(iterator.getUnwrappedIndex(), position);
    if (!globalMap.isInside(position))
    {
      continue;
    }
    float& original_elevation = globalMap.atPosition("elevation", position);
    float new_elevation = localMap.atPosition("elevation", position);
    float& original_traversability = globalMap.atPosition("traversability", position);
    float new_traversability = localMap.atPosition("traversability", position);
    float& original_uncertainty = globalMap.atPosition("uncertainty_range", position);
    float new_uncertainty = localMap.atPosition("uncertainty_range", position);
    float& original_timestamp = globalMap.atPosition("time", position);
    float new_timestamp = msg.info.header.stamp.toSec();
    if (CompareGridNode(original_uncertainty, new_uncertainty, original_timestamp, new_timestamp))
    {
      original_elevation = new_elevation;
      original_traversability = new_traversability;
      original_uncertainty = new_uncertainty;
      original_timestamp = new_timestamp;
    }
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

bool CompareGridNode(float original_uncertainty, float new_uncertainty, float original_timestamp, float new_timestamp)
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
  float uncertainty_penalty = 1.0f;
  float time_penalty = 1.0f;
  float penalty = uncertainty_penalty * uncertainty_diff - time_penalty * time_diff;
  if (penalty < 0.0f)
  {
    return true;
  }
  return false;
}