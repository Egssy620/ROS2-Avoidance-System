#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

namespace wall_filter_pkg  // ✅ PLUGIN と一致
{

class WallFilterNode : public rclcpp::Node
{
public:
  explicit WallFilterNode(const rclcpp::NodeOptions & options)
  : Node("wall_filter_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "WallFilterNode started!");

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/filtered_points",
      rclcpp::SensorDataQoS(),  // ✅ QoS を LiDAR に合わせる
      std::bind(&WallFilterNode::callback, this, std::placeholders::_1)
    );

    pub_wall_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/wall_points", 10);
    pub_obs_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_points", 10);

    declare_parameter("jump_threshold", 0.15);
    get_parameter("jump_threshold", jump_threshold_);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received filtered_points: %u points", msg->width);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (cloud->points.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Not enough points to process!");
      return;
    }

    enum Label { WALL, OBSTACLE };
    Label current_label = WALL;

    wall_cloud->header = obs_cloud->header = cloud->header;

    wall_cloud->points.push_back(cloud->points[0]);

    for (size_t i = 1; i < cloud->points.size(); ++i) {
      const auto &p1 = cloud->points[i-1];
      const auto &p2 = cloud->points[i];

      double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
      double r1 = std::hypot(p1.x, p1.y);
      double r2 = std::hypot(p2.x, p2.y);

      if (dist > jump_threshold_) {
        if (r2 < r1) {
          current_label = OBSTACLE;
        } else {
          current_label = WALL;
        }
      }

      if (current_label == WALL) {
        wall_cloud->points.push_back(p2);
      } else {
        obs_cloud->points.push_back(p2);
      }
    }

    sensor_msgs::msg::PointCloud2 msg_wall, msg_obs;
    pcl::toROSMsg(*wall_cloud, msg_wall);
    pcl::toROSMsg(*obs_cloud, msg_obs);

    msg_wall.header = msg->header;
    msg_obs.header  = msg->header;

    RCLCPP_INFO(this->get_logger(), "Publishing wall_points: %zu | obstacle_points: %zu",
      wall_cloud->points.size(), obs_cloud->points.size());

    pub_wall_->publish(msg_wall);
    pub_obs_->publish(msg_obs);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_wall_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obs_;
  double jump_threshold_;
};

}  // namespace wall_filter_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(wall_filter_pkg::WallFilterNode)

