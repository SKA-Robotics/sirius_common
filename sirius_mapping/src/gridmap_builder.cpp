#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <string>

class GridmapBuilderNode : public rclcpp::Node
{
public:
  GridmapBuilderNode() : Node("gridmap_builder")
  {
    this->declare_parameter("map_size", 100);
    this->declare_parameter("input_topic", "input_map");
    this->declare_parameter("global_output_topic", "output_global_map");
    this->declare_parameter("local_output_topic", "output_local_map");

    map_size_ = this->get_parameter("map_size").as_int();
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string global_out  = this->get_parameter("global_output_topic").as_string();
    std::string local_out   = this->get_parameter("local_output_topic").as_string();

    initializeGlobalMap();

    subscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic, 1,
      [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
        gridMapCallback(msg);
      });

    global_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      global_out, 5);
    local_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      local_out, 5);
  }

private:
  void initializeGlobalMap()
  {
    global_map_ = grid_map::GridMap(
      {"elevation", "traversability", "uncertainty_range", "time"});
    global_map_.setFrameId("map");
    global_map_.setGeometry(
      grid_map::Length(map_size_, map_size_), 0.05,
      grid_map::Position(0, 0));
  }

  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap local_map;
    // ROS2 grid_map API: fromMessage takes a const-ref, not SharedPtr
    grid_map::GridMapRosConverter::fromMessage(
      *msg, local_map,
      {"elevation", "traversability", "uncertainty_range"});

    // In ROS2 grid_map_msgs, GridMap.header.stamp holds the map timestamp.
    double new_timestamp = rclcpp::Time(msg->header.stamp).seconds();

    for (grid_map::GridMapIterator it(local_map); !it.isPastEnd(); ++it) {
      grid_map::Position pos;
      local_map.getPosition(it.getUnwrappedIndex(), pos);
      if (!global_map_.isInside(pos)) continue;

      global_map_.atPosition("elevation",         pos) =
        local_map.atPosition("elevation",         pos);
      global_map_.atPosition("traversability",    pos) =
        local_map.atPosition("traversability",    pos);
      global_map_.atPosition("uncertainty_range", pos) =
        local_map.atPosition("uncertainty_range", pos);
      global_map_.atPosition("time", pos) = static_cast<float>(new_timestamp);
    }

    publishGlobalMap();

    grid_map::Position center = local_map.getPosition();
    for (grid_map::CircleIterator it(local_map, center, 0.7);
         !it.isPastEnd(); ++it)
    {
      local_map.at("traversability", *it) = 1.0f;
    }
    publishLocalMap(local_map);
  }

  void publishGlobalMap()
  {
    global_map_.setTimestamp(this->now().nanoseconds());
    // ROS2 API: toMessage() returns unique_ptr
    auto msg_ptr = grid_map::GridMapRosConverter::toMessage(global_map_);
    global_publisher_->publish(*msg_ptr);
  }

  void publishLocalMap(const grid_map::GridMap & local_map)
  {
    auto msg_ptr = grid_map::GridMapRosConverter::toMessage(local_map);
    local_publisher_->publish(*msg_ptr);
  }

  int map_size_;
  grid_map::GridMap global_map_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscriber_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr global_publisher_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr local_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridmapBuilderNode>());
  rclcpp::shutdown();
  return 0;
}
