#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

class OccupancyGridPublisher : public rclcpp::Node
{
public:
  OccupancyGridPublisher()
  : Node("occupancy_grid_publisher")
  {
    // Initialize occupancy grid message as map
    occupancy_grid.header.stamp = now();
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.map_load_time = now();
    occupancy_grid.info.resolution = map_resolution; // meters/cell
    occupancy_grid.info.width = grid_width;  // cells
    occupancy_grid.info.height = grid_height;  // cells
    occupancy_grid.info.origin.position.x = -map_width / 2.0; // meters
    occupancy_grid.info.origin.position.y = -map_height / 2.0; // meters
    occupancy_grid.info.origin.position.z = map_level; // meters
    occupancy_grid.info.origin.orientation.w = 1.0;

    // Initialize as empty map (0 for free, 100 for occupied, -1 for unknown)
    occupancy_grid.data.resize(grid_width * grid_height, -1);

    // Publisher
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);

    // Subscriber
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed/zed_node/mapping/fused_cloud", 10, std::bind(&OccupancyGridPublisher::pointCloudCallback, this, std::placeholders::_1));

    // Timer
    timer_ = create_wall_timer(std::chrono::milliseconds(1000),
      std::bind(&OccupancyGridPublisher::publishOccupancyGrid, this));
  }

private:

  // Occupancy Grid
  nav_msgs::msg::OccupancyGrid occupancy_grid;

  // PCL type Point Cloud
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  
  // MAP PARAMETERS

  const int map_precision = 10; // cells per meter. 
  const double map_resolution = 1.0 / static_cast<double>(map_precision); // meters per cell. Must be 1/N.
  
  const double map_width = 20.0; // meters. Must result in integer grid cells.
  const double map_height = 20.0; // meters. Must result in integer grid cells.

  const int grid_width = map_width * map_precision; // cells
  const int grid_height = map_height * map_precision; // cells

  const double unitree_height = 0.4; // meters
  const double zedmount_height = 0.1; // meters
  const double map_level = -(unitree_height + zedmount_height); // meters

  void publishOccupancyGrid()
  {    

    // Simulate an obstacle in the center
    // int obstacle_index = (occupancy_grid->info.width / 2) + (occupancy_grid->info.height / 2) * occupancy_grid->info.width;
    // occupancy_grid->data[obstacle_index] = 100;

    // Publish the message
    publisher_->publish(occupancy_grid);

    RCLCPP_INFO(get_logger(), "OccupancyGrid published");
  }

  // Fill in the 2D map based on the 3D map
  void project_3Dmap_to_grid()
  {
    occupancy_grid.data.resize(grid_width * grid_height, 0);
  }

  // Adjust floor level of map
  void snap_grid_to_floor()
  {
    // Initialize variables to hold the minimum z coordinate and its corresponding point
    float min_z = std::numeric_limits<float>::max();
    pcl::PointXYZ min_point;

    // Iterate through each point in the point cloud
    for (const auto& point : pcl_cloud.points) {
        // Check if the z coordinate of the current point is smaller than the current minimum
        if (point.z < min_z) {
            // Update the minimum z coordinate and its corresponding point
            min_z = point.z;
            min_point = point;
        }
    }

    // Print the minimum z coordinate and its corresponding point
    RCLCPP_INFO(get_logger(), "Minimum z coordinate: %f", min_z);

    // Set grid height
    occupancy_grid.info.origin.position.z = min_z; // meters
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
  {
    // Process the PointCloud2 data and update the occupancy grid map

    // Example: Convert PointCloud2 to PointCloud data
    pcl::fromROSMsg(*point_cloud_msg, pcl_cloud);

    // Check if the point cloud is empty
    if (pcl_cloud.empty()) {
        RCLCPP_ERROR(get_logger(), "Received empty point cloud");
        return;
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Received PointCloud message");
    }

    snap_grid_to_floor();

    project_3Dmap_to_grid();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridPublisher>());
  rclcpp::shutdown();
  return 0;
}
