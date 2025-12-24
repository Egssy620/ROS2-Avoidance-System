// lidar_pkg/src/scan_shadow_filter_node.cpp
// LiDARスキャンの影除去フィルタノード

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>

#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace scan_shadow_filter
{

class ScanShadowFilterNode : public rclcpp::Node
{
public:
  explicit ScanShadowFilterNode(const rclcpp::NodeOptions & options)
  : Node("scan_shadow_filter_node", options)
  {
    // Parameters
    raw_mode_ = this->declare_parameter<bool>("raw_mode", false);
    invert_vertical_  = this->declare_parameter<bool>("invert_vertical", false);

    threshold_        = this->declare_parameter<double>("threshold",        0.10);  // 隣接差の影/外れ値しきい値
    proximity_radius_ = this->declare_parameter<double>("proximity_radius", 0.1);  // 近傍半径[m]
    output_frame_id_  = this->declare_parameter<std::string>("output_frame_id", "base_link");

    angle_min_deg_ = this->declare_parameter<double>("angle_min_deg", -90.0);
    angle_max_deg_ = this->declare_parameter<double>("angle_max_deg", 90.0);
    angle_min_rad_ = angle_min_deg_ * M_PI / 180.0;
    angle_max_rad_ = angle_max_deg_ * M_PI / 180.0;

    // Laser -> base_link の固定オフセット
    laser_offset_x_   = this->declare_parameter<double>("laser_offset_x",   0.0);
    laser_offset_y_   = this->declare_parameter<double>("laser_offset_y",   0.0);
    laser_offset_yaw_ = this->declare_parameter<double>("laser_offset_yaw", 0.0);   // 取付向き補正[rad]

    // 近距離ゲート
    min_range_        = this->declare_parameter<double>("min_range",        0.01);  // LaserScanの下限とmaxを採用
    min_radius_robot_ = this->declare_parameter<double>("min_radius_robot", 0.00);  // ロボ座標での最小半径[m]
    epsilon_radius_   = this->declare_parameter<double>("epsilon_radius",   0.05);  // 0扱い閾値[m]（0=無効）

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&ScanShadowFilterNode::scan_callback, this, std::placeholders::_1));

    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);
    
    // === Request full scan ===
    sub_fullscan_req_ = this->create_subscription<std_msgs::msg::Bool>(
        "/request_full_scan", 10,
        std::bind(&ScanShadowFilterNode::on_fullscan_request, this, std::placeholders::_1));

    // === Publisher for full scan ===
    pub_fullscan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/full_scan_points", 10);
    
    RCLCPP_INFO(this->get_logger(),
      "scan_shadow_filter_node started. threshold=%.3f proximity_radius=%.3f yaw=%.3f "
      "min_range=%.3f min_radius_robot=%.3f epsilon_radius=%.3f frame=%s",
      threshold_, proximity_radius_, laser_offset_yaw_,
      min_range_, min_radius_robot_, epsilon_radius_, output_frame_id_.c_str());
  }

private:
  // LaserScan callback
  void publish_raw_cloud(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
      std::vector<std::array<float,2>> pts;
      pts.reserve(msg->ranges.size());

      for (size_t i = 0; i < msg->ranges.size(); ++i) {
          float r = msg->ranges[i];
          if (!std::isfinite(r)) continue;

          double ang = msg->angle_min + i * msg->angle_increment;

          float x = r * std::cos(ang);
          float y = r * std::sin(ang);

          pts.push_back({x, y});
      }

      auto cloud = build_pointcloud2_xyz(pts, msg->header, msg->header.frame_id);
      pub_cloud_->publish(cloud);
  }

  // === Full Scan モード ===
  bool full_scan_mode_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_fullscan_req_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_fullscan_;

  void on_fullscan_request(const std_msgs::msg::Bool::SharedPtr msg)
  {
      if (msg->data) {
          full_scan_mode_ = true;
          RCLCPP_WARN(this->get_logger(), "[LiDAR] Full Scan Mode ON");
      }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
      const auto &ranges = msg->ranges;
      if (ranges.empty()) {
          publish_empty_cloud(msg->header);
          return;
      }

      // 生データをそのまま出す
      if (raw_mode_) {

          std::vector<std::array<float,2>> pts_raw;
          pts_raw.reserve(ranges.size());

          for (size_t i = 0; i < ranges.size(); ++i) {
              float r = ranges[i];
              if (!std::isfinite(r)) continue;

              double ang = msg->angle_min + i * msg->angle_increment;

              float x = r * std::cos(ang);
              float y = r * std::sin(ang);

              pts_raw.push_back({x, y});
          }

          auto cloud_raw = build_pointcloud2_xyz(
              pts_raw, msg->header, msg->header.frame_id   // frame_id = laser
          );

          pub_cloud_->publish(cloud_raw);
          return;
      }

      // フィルタ処理
      std::vector<bool> mask = compute_shadow_mask(ranges, threshold_);

      std::vector<std::array<float,2>> pts_sel;
      pts_sel.reserve(ranges.size());

      const double cL = std::cos(laser_offset_yaw_);
      const double sL = std::sin(laser_offset_yaw_);
      const double rmin_gate = std::max(static_cast<double>(msg->range_min), min_range_);
      const double r2_gate   = proximity_radius_ * proximity_radius_;
      const double r2_min_rb = (min_radius_robot_ > 0.0) ? (min_radius_robot_ * min_radius_robot_) : 0.0;

      for (size_t i = 0; i < ranges.size(); ++i) {
          const float r = ranges[i];
          if (!std::isfinite(r)) continue;
          if (!mask[i]) continue;

          const double ang = msg->angle_min + static_cast<double>(i) * msg->angle_increment;

          if (ang < angle_min_rad_ || ang > angle_max_rad_) continue;
          if (static_cast<double>(r) <= rmin_gate) continue;

          const double xl = r * std::cos(ang);
          const double yl = r * std::sin(ang);

          const double xb0 = cL * xl - sL * yl + laser_offset_x_;
          const double yb0 = sL * xl + cL * yl + laser_offset_y_;

          // 天地反転
          double xb = xb0;
          double yb = yb0;
          if (invert_vertical_) {
              yb = -yb;
          }

          const double r2 = xb * xb + yb * yb;

          if (r2_min_rb > 0.0 && r2 < r2_min_rb) continue;
          if (epsilon_radius_ > 0.0 && std::sqrt(r2) < epsilon_radius_) continue;
          if (r2 > r2_gate) continue;

          pts_sel.push_back({static_cast<float>(xb), static_cast<float>(yb)});
      }

      auto cloud_filtered =
          build_pointcloud2_xyz(pts_sel, msg->header, output_frame_id_);

      pub_cloud_->publish(cloud_filtered);

    // Full Scan モード　生点群を送る
    if (full_scan_mode_) {
        full_scan_mode_ = false;  // 1回だけ
        RCLCPP_WARN(this->get_logger(), "[LiDAR] Publishing FULL scan result");

        std::vector<std::array<float, 2>> pts_full;
        pts_full.reserve(ranges.size());

        for (size_t i = 0; i < ranges.size(); ++i) {
            float r = ranges[i];
            if (!std::isfinite(r)) continue;

            double ang = msg->angle_min + i * msg->angle_increment;
            if (ang < angle_min_rad_ || ang > angle_max_rad_) continue;

            double xl = r * std::cos(ang);
            double yl = r * std::sin(ang);

            double xb = cL*xl - sL*yl + laser_offset_x_;
            double yb = sL*xl + cL*yl + laser_offset_y_;

            pts_full.push_back({static_cast<float>(xb), static_cast<float>(yb)});
        }

        sensor_msgs::msg::PointCloud2 cloud_full =
            build_pointcloud2_xyz(pts_full, msg->header, output_frame_id_);
        pub_fullscan_->publish(cloud_full);
    }

    RCLCPP_DEBUG(this->get_logger(),
      "filtered_points (within %.2fm): %zu pts", proximity_radius_, pts_sel.size());
  }

  static sensor_msgs::msg::PointCloud2
  build_pointcloud2_xyz(const std::vector<std::array<float,2>> &points,
                        std_msgs::msg::Header header,
                        const std::string& frame_id)
  {
    header.frame_id = frame_id;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width  = points.size();
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");

    for (const auto &p : points) {
      *ix = p[0];
      *iy = p[1];
      *iz = 0.0f;
      ++ix; ++iy; ++iz;
    }
    return cloud;
  }

  static std::vector<bool> compute_shadow_mask(const std::vector<float> &ranges, double th)
  {
    std::vector<bool> mask(ranges.size(), true);
    if (th <= 0.0) return mask; // 0以下で無効化

    for (size_t i = 0; i + 1 < ranges.size(); ++i) {
      if (std::isfinite(ranges[i]) && std::isfinite(ranges[i+1])) {
        if (std::fabs(ranges[i+1] - ranges[i]) > th) {
          mask[i]   = false;
          mask[i+1] = false;
        }
      }
    }
    return mask;
  }

  void publish_empty_cloud(const std_msgs::msg::Header &header)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width  = 0;
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(0);

    pub_cloud_->publish(cloud);
  }

  // members
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;

  // parameters
  bool raw_mode_;
  double threshold_;
  double proximity_radius_;
  std::string output_frame_id_;

  double laser_offset_x_, laser_offset_y_, laser_offset_yaw_;

  double min_range_;
  double min_radius_robot_;
  double epsilon_radius_;

  double angle_min_deg_;
  double angle_max_deg_;
  double angle_min_rad_;
  double angle_max_rad_;
  bool invert_vertical_;

};

} // namespace scan_shadow_filter

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_shadow_filter::ScanShadowFilterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
