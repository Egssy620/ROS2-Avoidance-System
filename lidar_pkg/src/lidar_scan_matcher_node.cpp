#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Dense>

static inline double wrapToPi(double a){
  while(a >  M_PI) a -= 2*M_PI;
  while(a < -M_PI) a += 2*M_PI;
  return a;
}

class LidarScanMatcherNode : public rclcpp::Node {
public:
  LidarScanMatcherNode() : Node("lidar_scan_matcher_node")
  {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&LidarScanMatcherNode::onScan, this, std::placeholders::_1)
    );

    source_sub_ = create_subscription<std_msgs::msg::String>(
      "/cmd_source", 10,
      std::bind(&LidarScanMatcherNode::onSource, this, std::placeholders::_1)
    );

    heading_done_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/heading_done", 10,
      std::bind(&LidarScanMatcherNode::onHeadingDone, this, std::placeholders::_1)
    );

    yaw_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/yaw_delta", 10
    );

    RCLCPP_INFO(this->get_logger(),
      "LidarScanMatcherNode started (Hybrid ICP+NDT+SVD, no draw)");
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;

  double accumulated_yaw_ = 0.0;
  std::string current_source_ = "none";

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heading_done_sub_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;

  // ---- LaserScan → PCL cloud ----
  pcl::PointCloud<pcl::PointXYZ>::Ptr toCloud(const sensor_msgs::msg::LaserScan &scan)
  {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(scan.ranges.size());

    double ang = scan.angle_min;
    for (float r : scan.ranges) {
      if (std::isfinite(r)) {
        pcl::PointXYZ p;
        p.x = r * std::cos(ang);
        p.y = r * std::sin(ang);
        p.z = 0;
        cloud->push_back(p);
      }
      ang += scan.angle_increment;
    }
    return cloud;
  }

  // ---- Hybrid yaw estimation ----
  double estimateYaw(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &tgt)
  {
    // ① NDT
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setResolution(0.5);
    ndt.setMaximumIterations(20);
    ndt.setInputSource(src);
    ndt.setInputTarget(tgt);

    pcl::PointCloud<pcl::PointXYZ> ndt_out;
    ndt.align(ndt_out);

    Eigen::Matrix4f T_ndt = ndt.getFinalTransformation();
    double yaw_ndt = std::atan2(T_ndt(1,0), T_ndt(0,0));

    // ② ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(20);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);

    pcl::PointCloud<pcl::PointXYZ> icp_out;
    icp.align(icp_out);

    // ③ SVD 回転（より安定）
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    const size_t N = std::min(icp_out.size(), tgt->size());
    for(size_t i=0;i<N;i++){
      Eigen::Vector2d a(icp_out[i].x, icp_out[i].y);
      Eigen::Vector2d b((*tgt)[i].x, (*tgt)[i].y);
      H += a * b.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d R = svd.matrixU() * svd.matrixV().transpose();
    double yaw_svd = std::atan2(R(1,0), R(0,0));

    // ---- Hybrid (SVD優先) ----
    return 0.8 * yaw_svd + 0.2 * yaw_ndt;
  }

  // ---- Reset accumulated yaw ----
  void resetYaw()
  {
    accumulated_yaw_ = 0.0;
    RCLCPP_INFO(get_logger(), "[Yaw Reset]");
  }

  void onSource(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data != current_source_) {
      resetYaw();
    }
    current_source_ = msg->data;
  }

  void onHeadingDone(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(msg->data){
      resetYaw();
    }
  }

  // ---- Main LiDAR callback ----
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto curr = toCloud(*msg);

    if(!prev_cloud_){
      prev_cloud_ = curr;
      return;
    }

    double dyaw = estimateYaw(curr, prev_cloud_);
    accumulated_yaw_ += dyaw;

    prev_cloud_ = curr;

    std_msgs::msg::Float32 out;
    out.data = accumulated_yaw_;
    yaw_pub_->publish(out);
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarScanMatcherNode>());
  rclcpp::shutdown();
  return 0;
}
