/**
 * costmap_click_cost_node — ROS 2 node.
 * Subscribes to /clicked_point (RViz) and local costmap; prints cost value at clicked position.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace hnn_ros2_utils
{

class CostmapClickCostNode : public rclcpp::Node
{
public:
  CostmapClickCostNode()
  : Node("costmap_click_cost_node"),
    costmap_received_(false)
  {
    declare_parameter<std::string>("local_costmap_topic", "/local_costmap/costmap");
    declare_parameter<std::string>("clicked_point_topic", "/clicked_point");

    local_costmap_topic_ = get_parameter("local_costmap_topic").as_string();
    clicked_point_topic_ = get_parameter("clicked_point_topic").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_costmap_topic_, 10,
      std::bind(&CostmapClickCostNode::costmapCallback, this, std::placeholders::_1));

    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_point_topic_, 10,
      std::bind(&CostmapClickCostNode::clickedPointCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribed to %s and %s. Click a point in RViz to get local costmap cost.",
      clicked_point_topic_.c_str(), local_costmap_topic_.c_str());
  }

private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    costmap_ = *msg;
    costmap_received_ = true;
  }

  void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!costmap_received_) {
      RCLCPP_WARN(get_logger(), "No costmap received yet. Start navigation or ensure local costmap is published on %s",
        local_costmap_topic_.c_str());
      return;
    }

    const std::string costmap_frame = costmap_.header.frame_id;
    const std::string point_frame = msg->header.frame_id;

    geometry_msgs::msg::PointStamped point_in_costmap_frame;
    if (point_frame != costmap_frame) {
      try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
          costmap_frame, point_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
        tf2::doTransform(*msg, point_in_costmap_frame, transform);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "TF lookup failed: %s. Using point as-is (may be wrong frame).", ex.what());
        point_in_costmap_frame = *msg;
      }
    } else {
      point_in_costmap_frame = *msg;
    }

    const double x = point_in_costmap_frame.point.x;
    const double y = point_in_costmap_frame.point.y;

    const double ox = costmap_.info.origin.position.x;
    const double oy = costmap_.info.origin.position.y;
    const double res = costmap_.info.resolution;
    const uint32_t w = costmap_.info.width;
    const uint32_t h = costmap_.info.height;

    int col = static_cast<int>((x - ox) / res);
    int row = static_cast<int>((y - oy) / res);

    if (col < 0 || row < 0 || col >= static_cast<int>(w) || row >= static_cast<int>(h)) {
      RCLCPP_INFO(get_logger(), "Clicked (%.3f, %.3f) [frame: %s] -> grid (%d, %d) OUT OF BOUNDS (costmap size %ux%u, origin (%.3f, %.3f), resolution %.3f)",
        x, y, costmap_frame.c_str(), col, row, w, h, ox, oy, res);
      return;
    }

    const size_t idx = static_cast<size_t>(row) * w + static_cast<size_t>(col);
    int cost = static_cast<int>(costmap_.data[idx]);

    RCLCPP_INFO(get_logger(), "Clicked (%.3f, %.3f) [frame: %s] -> grid (%d, %d) -> cost = %d (0=free, 254=lethal, 255=unknown)",
      x, y, costmap_frame.c_str(), col, row, cost);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::OccupancyGrid costmap_;
  bool costmap_received_;
  std::string local_costmap_topic_;
  std::string clicked_point_topic_;
};

}  // namespace hnn_ros2_utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hnn_ros2_utils::CostmapClickCostNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
