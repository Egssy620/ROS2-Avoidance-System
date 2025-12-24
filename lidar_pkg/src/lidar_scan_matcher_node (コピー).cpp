#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace lidar_scan_matcher
{

class LidarScanMatcherNode : public rclcpp::Node
{
public:
  explicit LidarScanMatcherNode(const rclcpp::NodeOptions & options)
  : Node("lidar_scan_matcher_node", options)
  {
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/wall_points",
      rclcpp::SensorDataQoS(),
      std::bind(&LidarScanMatcherNode::cloud_callback, this, std::placeholders::_1)
    );

    line_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
      "/detected_lines",
      10,
      std::bind(&LidarScanMatcherNode::lines_callback, this, std::placeholders::_1)
    );

    relative_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
      "/lidar_scan_matcher/relative_transform", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "LidarScanMatcherNode started (ICP + Umeyama)");
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> prev_lines_;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> current_lines_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr line_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr relative_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *current_cloud);

    if (!prev_cloud_) {
      prev_cloud_ = current_cloud;
      RCLCPP_INFO(this->get_logger(), "Initial cloud stored.");
      return;
    }

    // === Step 1: ICP ===
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(current_cloud);
    icp.setInputTarget(prev_cloud_);
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud);
    
    Eigen::Matrix4f T_icp = icp.getFinalTransformation();
    
    RCLCPP_INFO(this->get_logger(), "ICP fitness: %.6f", icp.getFitnessScore());

    Eigen::Matrix4f T_final = T_icp;

    // === Step 2: Umeyama (オプション) ===
    if (prev_lines_.size() == 4 && current_lines_.size() == 4) {
      Eigen::MatrixXd P(2, 4);
      Eigen::MatrixXd Q(2, 4);

      for (size_t i = 0; i < 4; ++i) {
        Eigen::Vector2d p_mid = (prev_lines_[i].first + prev_lines_[i].second) * 0.5;
        Eigen::Vector2d q_mid = (current_lines_[i].first + current_lines_[i].second) * 0.5;

        P.col(i) = p_mid;
        Q.col(i) = q_mid;
      }

      Eigen::Matrix3d T_umeyama = Eigen::umeyama(Q, P, false);
      Eigen::Matrix2d R_umeyama = T_umeyama.block<2,2>(0,0);
      double yaw = std::atan2(R_umeyama(1,0), R_umeyama(0,0));
      Eigen::Vector2d t_umeyama = T_umeyama.block<2,1>(0,2);

      RCLCPP_INFO(this->get_logger(), "Umeyama correction: Yaw=%.3f deg, T=[%.3f, %.3f]",
        yaw * 180.0 / M_PI, t_umeyama.x(), t_umeyama.y());

      T_final(0,3) += t_umeyama.x();
      T_final(1,3) += t_umeyama.y();
      T_final(0,0) = std::cos(yaw);
      T_final(0,1) = -std::sin(yaw);
      T_final(1,0) = std::sin(yaw);
      T_final(1,1) = std::cos(yaw);
    }

    // === echo 用 ===
	double final_yaw = std::atan2(T_final(1,0), T_final(0,0));
	double yaw_deg = final_yaw * 180.0 / M_PI;

	// === 小数点第1位に丸める ===
	double tx = std::round(T_final(0,3) * 10.0) / 10.0;
	double ty = std::round(T_final(1,3) * 10.0) / 10.0;
	double yaw_d = std::round(yaw_deg * 10.0) / 10.0;

	geometry_msgs::msg::Vector3 vec_msg;
	vec_msg.x = tx;
	vec_msg.y = ty;
	vec_msg.z = yaw_d;

	relative_pub_->publish(vec_msg);

	RCLCPP_INFO(this->get_logger(),
	  "Relative Transform: X=%.1f m, Y=%.1f m, Yaw=%.1f deg",
	  tx, ty, yaw_d);
	    broadcast_tf(T_final, msg->header.stamp);

    prev_cloud_ = current_cloud;
  }

  void lines_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    current_lines_.clear();
    for (size_t i = 0; i + 1 < msg->points.size(); i += 2) {
      Eigen::Vector2d p1(msg->points[i].x, msg->points[i].y);
      Eigen::Vector2d p2(msg->points[i+1].x, msg->points[i+1].y);
      current_lines_.emplace_back(p1, p2);
    }
    RCLCPP_INFO(this->get_logger(), "Received %zu line segments", current_lines_.size());
    prev_lines_ = current_lines_;
  }

  void broadcast_tf(const Eigen::Matrix4f &T, const rclcpp::Time &stamp)
  {
    tf2::Quaternion q;
    double yaw = std::atan2(T(1,0), T(0,0));
    q.setRPY(0, 0, yaw);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_link";

    tf_msg.transform.translation.x = T(0,3);
    tf_msg.transform.translation.y = T(1,3);
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

};

} // namespace lidar_scan_matcher

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_scan_matcher::LidarScanMatcherNode)

