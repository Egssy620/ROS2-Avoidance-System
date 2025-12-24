#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/float32.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <unordered_map>
#include <atomic>
#include <thread>
#include <cmath>

// ---- ZYX(Euler) from rotation matrix ----
static cv::Vec3d rotm2eulZYX(const cv::Mat& R){
  double r11 = R.at<double>(0,0), r21 = R.at<double>(1,0), r31 = R.at<double>(2,0);
  double r32 = R.at<double>(2,1), r33 = R.at<double>(2,2);
  double yaw   = std::atan2(r21, r11);                           // Z
  double pitch = std::atan2(-r31, std::sqrt(r32*r32 + r33*r33)); // Y
  double roll  = std::atan2(r32, r33);                           // X
  return cv::Vec3d(yaw, pitch, roll);
}

class ArUcoLocalizerNode : public rclcpp::Node {
public:
  ArUcoLocalizerNode() : Node("aruco_localizer_node"),
                         steady_clock_(RCL_STEADY_TIME){
    // ---- Parameters (軽量・必要最小限) ----
    cam_index_     = this->declare_parameter<int>("camera_index", 0);
    fps_           = this->declare_parameter<int>("fps", 30);
    width_         = this->declare_parameter<int>("width", 1280);
    height_        = this->declare_parameter<int>("height", 720);
    calib_width_   = this->declare_parameter<int>("calib_width", 1920);
    calib_height_  = this->declare_parameter<int>("calib_height", 1080);
    marker_length_ = this->declare_parameter<double>("marker_length", 0.25);

    // ---- Camera calibration ----
    {
      cv::FileStorage fs("/home/ryuzo/ros2_ws/src/camera_calibration_pkg/calibration.yaml", cv::FileStorage::READ);
      if(!fs.isOpened()){
        RCLCPP_ERROR(get_logger(), "Failed to open calibration.yaml");
        throw std::runtime_error("no calibration");
      }
      fs["camera_matrix"] >> K_;
      fs["distortion_coefficients"] >> D_;
      fs.release();
      // スケール（元のキャリブ解像度→現在の処理解像度）
      double sx = static_cast<double>(width_)  / static_cast<double>(calib_width_);
      double sy = static_cast<double>(height_) / static_cast<double>(calib_height_);
      K_.at<double>(0,0) *= sx; // fx
      K_.at<double>(1,1) *= sy; // fy
      K_.at<double>(0,2) *= sx; // cx
      K_.at<double>(1,2) *= sy; // cy
    }

    // ---- Publishers ----
    pose_pub_   = this->create_publisher<geometry_msgs::msg::PoseStamped>("marker_pose", rclcpp::QoS(10));
    pose2d_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("robot_pose2d", rclcpp::QoS(10));
    // dtは軽量・監視用途のためBestEffort + 購読者がいる時だけ送信
    dt_pub_     = this->create_publisher<std_msgs::msg::Float32>("/pose_dt", rclcpp::QoS(10).best_effort());

    // ---- ArUco dictionary & params ----
    dict_   = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    params_ = cv::aruco::DetectorParameters::create();
    // 軽量寄りの初期値（必要なら微調整）
    params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    params_->minMarkerPerimeterRate = 0.02;

    // ---- Marker map (固定配置：必要最小限の例) ----
    // 環境に合わせて調整してください
    const double DX = 1.0, DY = 2.5;
    marker_map_.clear();
    marker_map_[0] = cv::Vec3d(-DX,    0.0,   0.0);
    marker_map_[1] = cv::Vec3d( 0.0,   0.0,   0.0);
    marker_map_[2] = cv::Vec3d(+DX,    0.0,   0.0);
    marker_map_[3] = cv::Vec3d(-DX,   -DY,    0.0);
    marker_map_[4] = cv::Vec3d( 0.0,  -DY,    0.0);
    marker_map_[5] = cv::Vec3d(+DX,   -DY,    0.0);

    // ---- Camera open (720p, MJPG, BUFFERSIZE=1) ----
    if(!cap_.open(cam_index_, cv::CAP_V4L2)){
      RCLCPP_ERROR(this->get_logger(), "Camera open failed at index %d.", cam_index_);
      throw std::runtime_error("camera open failed");
    }
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap_.set(cv::CAP_PROP_FPS, fps_);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // ---- Worker thread start ----
    running_.store(true);
    worker_ = std::thread(&ArUcoLocalizerNode::workerLoop, this);
  }

  ~ArUcoLocalizerNode() override {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
    cap_.release();
  }

private:
  void workerLoop(){
    while (rclcpp::ok() && running_.load()) {
      // 最新のみ取得（滞留防止）
      if (!cap_.grab()) { std::this_thread::sleep_for(std::chrono::milliseconds(2)); continue; }
      cv::Mat frame;
      if (!cap_.retrieve(frame) || frame.empty()) continue;

      // ---- ArUco detect ----
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::Mat gray; cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      cv::aruco::detectMarkers(gray, dict_, corners, ids, params_);
      if (ids.empty()) continue;

      // ---- Build PnP correspondences ----
      std::vector<cv::Point2f> img_pts; img_pts.reserve(ids.size()*4);
      std::vector<cv::Point3f> obj_pts; obj_pts.reserve(ids.size()*4);
      const double half = marker_length_ * 0.5;

      for (size_t k = 0; k < ids.size(); ++k) {
        auto it = marker_map_.find(ids[k]);
        if (it == marker_map_.end()) continue;
        const cv::Vec3d c = it->second;
        cv::Point3f w0(c[0]-half, c[1]+half, c[2]);
        cv::Point3f w1(c[0]+half, c[1]+half, c[2]);
        cv::Point3f w2(c[0]+half, c[1]-half, c[2]);
        cv::Point3f w3(c[0]-half, c[1]-half, c[2]);
        const auto& ic = corners[k];
        obj_pts.push_back(w0); img_pts.push_back(ic[0]);
        obj_pts.push_back(w1); img_pts.push_back(ic[1]);
        obj_pts.push_back(w2); img_pts.push_back(ic[2]);
        obj_pts.push_back(w3); img_pts.push_back(ic[3]);
      }
      if (obj_pts.size() < 4) continue;

      // ---- PnP (軽量RANSAC→Refine) ----
      cv::Mat rvec, tvec;
      std::vector<int> inliers;
      bool ok = cv::solvePnPRansac(
          obj_pts, img_pts, K_, D_, rvec, tvec, false,
          100, 3.0, 0.99, inliers, cv::SOLVEPNP_EPNP);
      if (!ok) continue;
      cv::solvePnPRefineLM(obj_pts, img_pts, K_, D_, rvec, tvec);

      // ---- Publish pose ----
      cv::Mat Rcw; cv::Rodrigues(rvec, Rcw);     // world->camera
      cv::Mat Rwc = Rcw.t();
      cv::Mat Cw  = -Rwc * tvec;                 // camera position in world
      cv::Vec3d eul = rotm2eulZYX(Rwc);
      double yaw = eul[0];

      auto now_ros = this->get_clock()->now();
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = now_ros;
      msg.header.frame_id = "map";
      msg.pose.position.x = Cw.at<double>(0);
      msg.pose.position.y = Cw.at<double>(1);
      msg.pose.position.z = Cw.at<double>(2);
      double half_yaw = 0.5 * yaw;
      msg.pose.orientation.w = std::cos(half_yaw);
      msg.pose.orientation.z = std::sin(half_yaw);
      pose_pub_->publish(msg);

      geometry_msgs::msg::Pose2D p2d;
      p2d.x = Cw.at<double>(0);
      p2d.y = Cw.at<double>(1);
      p2d.theta = yaw;
      pose2d_pub_->publish(p2d);

      // ---- Δt（前回配信からの秒数）をBestEffortで軽量配信 ----
      auto now_steady = steady_clock_.now();
      if (last_pub_steady_.nanoseconds() != 0 && dt_pub_->get_subscription_count() > 0) {
        std_msgs::msg::Float32 dt;
        dt.data = static_cast<float>((now_steady - last_pub_steady_).seconds());
        dt_pub_->publish(dt);
      }
      last_pub_steady_ = now_steady;
    }
  }

  // --- Data ---
  cv::Mat K_, D_;
  cv::VideoCapture cap_;
  int cam_index_{4}, fps_{30}, width_{1280}, height_{720};
  int calib_width_{1920}, calib_height_{1080};
  double marker_length_{0.25};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr       pose2d_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           dt_pub_;

  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
  std::unordered_map<int, cv::Vec3d> marker_map_;

  std::atomic<bool> running_{false};
  std::thread worker_;

  rclcpp::Clock steady_clock_;
  rclcpp::Time  last_pub_steady_{0,0,RCL_STEADY_TIME};
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArUcoLocalizerNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
