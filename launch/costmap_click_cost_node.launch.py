"""
Launch file for costmap_click_cost_node.
Subscribes to /clicked_point (RViz) and local costmap; prints cost value at clicked position.

Costmap source:
  - costmap_type=occupancy_grid -> nav_msgs/msg/OccupancyGrid (e.g. /local_costmap/costmap)
  - costmap_type=costmap       -> nav2_msgs/msg/Costmap (e.g. /local_costmap/costmap_raw)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    local_costmap_topic_arg = DeclareLaunchArgument(
        'local_costmap_topic',
        default_value='/local_costmap/costmap_raw',
        description='Topic for local costmap',
    )
    clicked_point_topic_arg = DeclareLaunchArgument(
        'clicked_point_topic',
        default_value='/clicked_point',
        description='Topic for clicked point from RViz (PointStamped)',
    )
    costmap_type_arg = DeclareLaunchArgument(
        'costmap_type',
        default_value='costmap',
        description='Costmap message type: "occupancy_grid" (nav_msgs) or "costmap" (nav2_msgs)',
    )

    costmap_click_cost_node = Node(
        package='hnn_ros2_utils',
        executable='costmap_click_cost_node',
        name='costmap_click_cost_node',
        output='screen',
        parameters=[
            {
                'local_costmap_topic': LaunchConfiguration('local_costmap_topic'),
                'clicked_point_topic': LaunchConfiguration('clicked_point_topic'),
                'costmap_type': LaunchConfiguration('costmap_type'),
            }
        ],
    )

    return LaunchDescription([
        local_costmap_topic_arg,
        clicked_point_topic_arg,
        costmap_type_arg,
        costmap_click_cost_node,
    ])
