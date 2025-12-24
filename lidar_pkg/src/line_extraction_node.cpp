#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>

namespace line_extraction
{

class LineExtractionNode : public rclcpp::Node
{
public:
  explicit LineExtractionNode(const rclcpp::NodeOptions & options)
  : Node("line_extraction_node", options)
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/wall_points",
      rclcpp::SensorDataQoS(),
      std::bind(&LineExtractionNode::cloud_callback, this, std::placeholders::_1)
    );

    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/detected_lines_array", 10);

    RCLCPP_INFO(this->get_logger(), "✅ LineExtractionNode started (MarkerArray version)");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>(*cloud));

    visualization_msgs::msg::MarkerArray marker_array;

    // === LINE_LIST marker ===
    visualization_msgs::msg::Marker line_marker;
    line_marker.header = msg->header;
    line_marker.ns = "detected_lines";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.05;
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    int line_count = 0;
    const int max_lines = 4;

    while (remaining->size() > 50 && line_count < max_lines) {
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_LINE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.05);
      seg.setMaxIterations(100);

      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

      seg.setInputCloud(remaining);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.empty()) {
        break;
      }

      float px = coefficients->values[0];
      float py = coefficients->values[1];
      float pz = coefficients->values[2];
      float dx = coefficients->values[3];
      float dy = coefficients->values[4];
      float dz = coefficients->values[5];

      geometry_msgs::msg::Point p1, p2;
      p1.x = px - dx * 5.0;
      p1.y = py - dy * 5.0;
      p1.z = 0.0;
      p2.x = px + dx * 5.0;
      p2.y = py + dy * 5.0;
      p2.z = 0.0;

      line_marker.points.push_back(p1);
      line_marker.points.push_back(p2);

      // === Text marker ===
      visualization_msgs::msg::Marker text_marker;
      text_marker.header = msg->header;
      text_marker.ns = "line_labels";
      text_marker.id = line_count;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.scale.z = 0.3;
      text_marker.color.r = 0.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 0.0;
      text_marker.color.a = 1.0;

      Eigen::Vector3d mid(
        (p1.x + p2.x) / 2.0,
        (p1.y + p2.y) / 2.0,
        0.0);
      text_marker.pose.position.x = mid.x();
      text_marker.pose.position.y = mid.y();
      text_marker.pose.position.z = 0.2;

      text_marker.text = "Line " + std::to_string(line_count);

      marker_array.markers.push_back(text_marker);

      RCLCPP_INFO(this->get_logger(),
        "Line %d: (%.2f, %.2f) -> (%.2f, %.2f), inliers: %zu",
        line_count, p1.x, p1.y, p2.x, p2.y, inliers->indices.size());

      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(remaining);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*remaining);

      line_count++;
    }

    marker_array.markers.push_back(line_marker);

    marker_array_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Published %d line segments with labels.", line_count);
  }

};

}  // namespace line_extraction

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(line_extraction::LineExtractionNode)

