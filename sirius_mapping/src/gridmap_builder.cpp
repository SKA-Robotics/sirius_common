#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <string>

std::string input_topic = "input_map";
std::string global_output_topic = "output_global_map";
std::string local_output_topic = "output_local_map";

ros::Publisher localPublisher;
ros::Publisher globalPublisher;
ros::Subscriber subscriber;

grid_map::GridMap globalMap;
grid_map::GridMap localMap;

void InitializeGlobalMap();
void GridMapCallback(const grid_map_msgs::GridMap& msg);
void PublishGlobalMap();
void PublishLocalMap();
bool CompareGridNode(float original_uncertainty, float new_uncertainty, float original_timestamp, float new_timestamp);

int map_size;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gridmap_builder");
  ros::NodeHandle node_handle("~");
  ros::param::param<int>("map_size", map_size, 100);
  InitializeGlobalMap();
  subscriber = node_handle.subscribe(input_topic, 1, GridMapCallback);
  globalPublisher = node_handle.advertise<grid_map_msgs::GridMap>(global_output_topic, 5);
  localPublisher = node_handle.advertise<grid_map_msgs::GridMap>(local_output_topic, 5);
  ros::spin();
  return 0;
}

void InitializeGlobalMap()
{
  globalMap = grid_map::GridMap({ "elevation", "traversability", "uncertainty_range", "time" });
  globalMap.setFrameId("map");
  globalMap.setGeometry(grid_map::Length(map_size, map_size), 0.05, grid_map::Position(0, 0));
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

  grid_map::Position center = localMap.getPosition();
  double radius = 0.7;
  for (grid_map::CircleIterator iterator(localMap, center, radius); !iterator.isPastEnd(); ++iterator)
  {
    localMap.at("traversability", *iterator) = 1.0;
  }
  PublishLocalMap();
}

void PublishGlobalMap()
{
  ros::Time time = ros::Time::now();
  globalMap.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(globalMap, message);
  globalPublisher.publish(message);
}

void PublishLocalMap()
{
  ros::Time time = ros::Time::now();
  globalMap.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(localMap, message);
  localPublisher.publish(message);
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
  float time_penalty = 5.0f;
  float penalty = uncertainty_penalty * uncertainty_diff - time_penalty * time_diff;
  if (penalty < 0.0f)
  {
    return true;
  }
  return false;
}