#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <unordered_map>
#include <atomic>
#include <thread>
#include <cmath>
#include <algorithm>

// ---------------- Utils ----------------
static cv::Vec3d rotm2eulZYX(const cv::Mat& R){
  double r11 = R.at<double>(0,0), r21 = R.at<double>(1,0), r31 = R.at<double>(2,0);
  double r32 = R.at<double>(2,1), r33 = R.at<double>(2,2);
  double yaw   = std::atan2(r21, r11);
  double pitch = std::atan2(-r31, std::sqrt(r32*r32 + r33*r33));
  double roll  = std::atan2(r32, r33);
  return cv::Vec3d(yaw, pitch, roll);
}

static inline double ang_wrap(double a){
  while(a >  M_PI) a -= 2*M_PI;
  while(a < -M_PI) a += 2*M_PI;
  return a;
}

// フィルタ用に wrapToPi も用意（中身は同じ）
static inline double wrapToPi(double a){
  while(a >  M_PI) a -= 2*M_PI;
  while(a < -M_PI) a += 2*M_PI;
  return a;
}

static double angle_mean(const std::vector<double>& ang, const std::vector<double>& w){
  double sx=0.0, sy=0.0;
  for (size_t i=0;i<ang.size();++i){
    sx += w[i]*std::cos(ang[i]);
    sy += w[i]*std::sin(ang[i]);
  }
  return std::atan2(sy, sx);
}

// 状態表現
struct Pose2D {
  double x;
  double y;
  double yaw;
};

// ---------------- Node ----------------
class ArUcoLocalizerNode : public rclcpp::Node {
public:
  ArUcoLocalizerNode() : Node("aruco_pnp_node"), steady_clock_(RCL_STEADY_TIME){
    // ---- Parameters ----
    cam_index_     = this->declare_parameter<int>("camera_index", 4);
    fps_           = this->declare_parameter<int>("fps", 30);
    width_         = this->declare_parameter<int>("width", 1920);
    height_        = this->declare_parameter<int>("height", 1080);
    calib_width_   = this->declare_parameter<int>("calib_width", 1920);
    calib_height_  = this->declare_parameter<int>("calib_height", 1080);
    marker_length_ = this->declare_parameter<double>("marker_length", 0.242);
    enable_draw_   = this->declare_parameter<bool>("enable_draw", false);

    // ---- Camera calibration ----
    cv::FileStorage fs("/home/ryuzo/ros2_ws/src/aruco_pnp_node/calibration.yaml", cv::FileStorage::READ);
    if(!fs.isOpened()){
      RCLCPP_ERROR(get_logger(), "Failed to open calibration.yaml");
      throw std::runtime_error("no calibration");
    }
    fs["camera_matrix"] >> K_;
    fs["distortion_coefficients"] >> D_;
    fs.release();
    double sx = (double)width_  / calib_width_;
    double sy = (double)height_ / calib_height_;
    K_.at<double>(0,0) *= sx; // fx
    K_.at<double>(1,1) *= sy; // fy
    K_.at<double>(0,2) *= sx; // cx
    K_.at<double>(1,2) *= sy; // cy

    // ---- Subscribers ----
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&ArUcoLocalizerNode::onCmdVel, this, std::placeholders::_1));
    
    sub_lidar_yaw_ = this->create_subscription<std_msgs::msg::Float32>(
      "/lidar_yaw_est", 10,
      std::bind(&ArUcoLocalizerNode::onLidarYaw, this, std::placeholders::_1));


    // ---- Publishers ----
    pose_pub_         = this->create_publisher<geometry_msgs::msg::PoseStamped>("/marker_pose", 10);
    pose2d_pub_       = this->create_publisher<geometry_msgs::msg::Pose2D>("/robot_pose2d", 10);
    marker_pose_pub_  = this->create_publisher<geometry_msgs::msg::PoseArray>("/aruco_markers", 10);
    marker_id_pub_    = this->create_publisher<std_msgs::msg::Int32MultiArray>("/aruco_marker_ids", 10);
    dt_pub_           = this->create_publisher<std_msgs::msg::Float32>("/pose_dt", rclcpp::QoS(10).best_effort());
    pose_source_pub_  = this->create_publisher<std_msgs::msg::String>("/pose_source", 10);
    
    // ---- ArUco ----
    dict_   = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    params_ = cv::aruco::DetectorParameters::create();
    params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    params_->minMarkerPerimeterRate = 0.02;

    // ---- Marker map (world座標系) ----
    const double DX = 2.5, DY = 2.5;
    marker_map_[0]  = cv::Vec3d(  -DX-DX/2, -1.0,   0.0);
    marker_map_[1]  = cv::Vec3d(  -DX/2,    -1.0,   0.0);
    marker_map_[2]  = cv::Vec3d(   DX/2,    -1.0,   0.0);
    marker_map_[3]  = cv::Vec3d(   DX+DX/2, -1.0,   0.0);
    marker_map_[4]  = cv::Vec3d(  -2*DX,    -1.0-DY,   0.0);
    marker_map_[5]  = cv::Vec3d(  -DX,      -1.0-DY,   0.0);
    marker_map_[6]  = cv::Vec3d(   0,       -1.0-DY,   0.0);
    marker_map_[7]  = cv::Vec3d(   DX,      -1.0-DY,   0.0);
    marker_map_[8]  = cv::Vec3d(   2*DX+0.3, -1.0-DY, 0.0);
    marker_map_[9]  = cv::Vec3d(  -DX-DX/2, -1.0-2*DY,   0.0);
    marker_map_[10] = cv::Vec3d( -DX/2,    -1.0-2*DY,  0.0);
    marker_map_[11] = cv::Vec3d(  DX/2,    -1.0-2*DY,  0.0);
    marker_map_[12] = cv::Vec3d(  DX+DX/2, -1.0-2*DY,  0.0);
    marker_map_[13] = cv::Vec3d( -2*DX,    -1.0-3*DY,  0.0);
    marker_map_[14] = cv::Vec3d( -DX,      -1.0-3*DY,  0.0);
    marker_map_[15] = cv::Vec3d(  0,       -1.0-3*DY,  0.0);
    marker_map_[16] = cv::Vec3d(  DX,      -1.0-3*DY,  0.0);
    marker_map_[17] = cv::Vec3d(  2*DX,    -1.0-3*DY,  0.0);
    marker_map_[18] = cv::Vec3d(  -DX-DX/2, -1.0-4*DY,   0.0);
    marker_map_[19] = cv::Vec3d( -DX/2,    -1.0-4*DY,  0.0);
    marker_map_[20] = cv::Vec3d(  DX/2,    -1.0-4*DY,  0.0);
    marker_map_[21] = cv::Vec3d(  5.85, -0.15,  0.0);
    marker_map_[22] = cv::Vec3d(  -6.0, -1.0-DY,  0.0);

    // ---- Camera open ----
    if(!cap_.open(cam_index_, cv::CAP_V4L2)){
      RCLCPP_ERROR(this->get_logger(), "Camera open failed at index %d.", cam_index_);
      throw std::runtime_error("camera open failed");
    }
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap_.set(cv::CAP_PROP_FPS, fps_);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    RCLCPP_INFO(this->get_logger(), "ArUcoLocalizerNode 起動");
    running_.store(true);
    worker_ = std::thread(&ArUcoLocalizerNode::workerLoop, this);
  }

  ~ArUcoLocalizerNode() override {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
    cap_.release();
  }

private:
  // ==== cmd_vel 受信 ====
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
    have_cmd_ = true;
  }
  
  void onLidarYaw(const std_msgs::msg::Float32::SharedPtr msg)
  {
      lidar_yaw_est_ = msg->data;
  }

  // ==== フィルタ付き姿勢更新 ====
  void updatePoseWithConsistencyCheck(
          double x_meas, double y_meas, double yaw_meas,
          const rclcpp::Time& t_now)
  {
      std_msgs::msg::String src_msg;

      // ---- 初回は無条件採用 ----
      if (!have_estimate_) {
          x_est_.x = x_meas;
          x_est_.y = y_meas;
          x_est_.yaw = yaw_meas;
          have_estimate_ = true;
          last_pose_time_ = t_now;

          src_msg.data = "measured_initial";
          pose_source_pub_->publish(src_msg);
          publishPose(x_est_);
          return;
      }

      // ---- 距離差だけチェック ----
      double dx = x_meas - x_est_.x;
      double dy = y_meas - x_est_.y;
      double dist = std::hypot(dx, dy);

      const double THRESH = 1.0;  // ★ 1m以上離れた観測は外れ値とみなす

      if (dist > THRESH) {
          // ---- 外れ値 → 前回の姿勢をそのまま使う ----
          src_msg.data = "reject_and_keep_previous";
          pose_source_pub_->publish(src_msg);

          // x_est_ は更新しない
          publishPose(x_est_);
          return;
      }

      // ---- 正常な範囲 → 観測をそのまま採用 ----
      x_est_.x = x_meas;
      x_est_.y = y_meas;
      x_est_.yaw = yaw_meas;
      last_pose_time_ = t_now;

      src_msg.data = "measured";
      pose_source_pub_->publish(src_msg);

      publishPose(x_est_);
      // ======== yaw デバッグ publish ========
      std_msgs::msg::Float32MultiArray dbg;
      dbg.data.resize(3);
      dbg.data[0] = yaw_meas;        // カメラの観測yaw
      dbg.data[1] = x_est_.yaw;      // フィルタ後の推定yaw
      dbg.data[2] = lidar_yaw_est_;  // LiDAR 推定yaw
      yaw_debug_pub_->publish(dbg);
  }

  // ==== 姿勢 Publish 共通関数 ====
  void publishPose(const Pose2D& p)
  {
    // 2D Pose
    geometry_msgs::msg::Pose2D msg2d;
    msg2d.x     = p.x;
    msg2d.y     = p.y;
    msg2d.theta = p.yaw;
    pose2d_pub_->publish(msg2d);

    // 3D Pose
    geometry_msgs::msg::PoseStamped msg3d;
    msg3d.header.stamp = this->get_clock()->now();
    msg3d.header.frame_id = "map";
    msg3d.pose.position.x = p.x;
    msg3d.pose.position.y = p.y;
    msg3d.pose.position.z = 0.0;
    msg3d.pose.orientation.w = std::cos(0.5 * p.yaw);
    msg3d.pose.orientation.z = std::sin(0.5 * p.yaw);
    pose_pub_->publish(msg3d);
  }

  // ==== メインループ ====
  void workerLoop(){
    const double RMS_THRESH_PX = 3.5;
    const double W_EPS = 1e-6;
    const double half = marker_length_ * 0.5;

    const std::vector<cv::Point3f> marker_corners_local = {
      {-half, +half, 0.0f}, {+half, +half, 0.0f},
      {+half, -half, 0.0f}, {-half, -half, 0.0f}
    };

    while (rclcpp::ok() && running_.load()) {
      cv::Mat frame;
      if(!cap_.read(frame)) continue;

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::Mat gray; cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      cv::aruco::detectMarkers(gray, dict_, corners, ids, params_);
      if(ids.empty()) continue;

      struct Hypo { double x,y,yaw; double rms; int id; };
      std::vector<Hypo> hyps; hyps.reserve(ids.size());

      for (size_t k=0;k<ids.size();++k){
        auto it = marker_map_.find(ids[k]);
        if(it == marker_map_.end()) continue;
        const auto& img_c = corners[k];
        if(img_c.size()!=4) continue;

        cv::Mat rvec_cm, tvec_cm;
        bool ok = cv::solvePnP(marker_corners_local, img_c, K_, D_, rvec_cm, tvec_cm,
                               false, cv::SOLVEPNP_IPPE_SQUARE);
        if(!ok) continue;

        std::vector<cv::Point2f> proj;
        cv::projectPoints(marker_corners_local, rvec_cm, tvec_cm, K_, D_, proj);
        double se=0.0;
        for(int j=0;j<4;++j){
          cv::Point2f d = proj[j]-img_c[j];
          se += d.x*d.x + d.y*d.y;
        }
        double rms = std::sqrt(se/4.0);
        if (rms > RMS_THRESH_PX) continue;  // 粗いフィルタ（必要なら）

        cv::Mat R_cm; cv::Rodrigues(rvec_cm, R_cm);
        cv::Mat R_mc = R_cm.t();
        const cv::Vec3d& c = it->second; // markerの世界座標
        cv::Mat t_cm = tvec_cm;
        cv::Mat Cw = -R_mc*t_cm + (cv::Mat_<double>(3,1)<<c[0],c[1],c[2]);
        cv::Vec3d eul = rotm2eulZYX(R_mc);

        hyps.push_back({Cw.at<double>(0), Cw.at<double>(1), eul[0], rms, ids[k]});
      }

      if(hyps.empty()) continue;

      // ---- Publish detected markers (世界座標: marker_map の値) ----
      {
        geometry_msgs::msg::PoseArray arr;
        std_msgs::msg::Int32MultiArray ids_msg;
        arr.header.stamp = this->get_clock()->now();
        arr.header.frame_id = "map";

        for (size_t k = 0; k < ids.size(); ++k) {
          auto it = marker_map_.find(ids[k]);
          if (it == marker_map_.end()) continue;
          const cv::Vec3d& c = it->second;

          geometry_msgs::msg::Pose p;
          p.position.x = c[0];
          p.position.y = c[1];
          p.position.z = c[2];
          p.orientation.w = 1.0;
          p.orientation.z = 0.0;

          arr.poses.push_back(p);
          ids_msg.data.push_back(ids[k]);
        }

        if (!arr.poses.empty()) {
          marker_pose_pub_->publish(arr);
          marker_id_pub_->publish(ids_msg);
        }

        if (enable_draw_) {
          cv::aruco::drawDetectedMarkers(frame, corners, ids);
          for (const auto& h : hyps) {
            cv::Point2d text_pos(50, 50 + 25 * h.id);
            cv::putText(frame, "ID:" + std::to_string(h.id) +
                                " RMS:" + cv::format("%.2f", h.rms),
                        text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0,255,0), 1, cv::LINE_AA);
          }
          cv::imshow("ArUco Localization", frame);
          int key = cv::waitKey(1);
          if (key == 'q' || key == 27) {
            RCLCPP_INFO(this->get_logger(), "Exit visualization.");
            running_.store(false);
            break;
          }
        }
      }

      // ---- 自己位置融合 (Pose2D) ----
      std::vector<double> w, yawv;
      double fx=0, fy=0, sumw=0;
      for(auto& h:hyps){
        double ww = 1.0/(h.rms+W_EPS);
        fx += ww*h.x;
        fy += ww*h.y;
        sumw += ww;
        w.push_back(ww);
        yawv.push_back(h.yaw);
      }
      if (sumw <= 0.0) continue;

      double x   = fx/sumw;
      double y   = fy/sumw;
      double yaw = angle_mean(yawv, w);

      auto t_now = this->get_clock()->now();
      updatePoseWithConsistencyCheck(x, y, yaw, t_now);
    }
  }

  // --- Data ---
  cv::Mat K_, D_;
  cv::VideoCapture cap_;
  int cam_index_, fps_, width_, height_, calib_width_, calib_height_;
  double marker_length_;
  bool enable_draw_;
  int good_obs_count_ = 0;   // 安定した観測が連続したフレーム数

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr marker_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr marker_id_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dt_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_source_pub_;
  double lidar_yaw_est_ = std::numeric_limits<double>::quiet_NaN();
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_lidar_yaw_;

  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
  std::unordered_map<int, cv::Vec3d> marker_map_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  rclcpp::Clock steady_clock_;

  // cmd_vel
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  geometry_msgs::msg::Twist last_cmd_;
  bool have_cmd_ = false;

  // フィルタ状態
  Pose2D x_est_;
  bool have_estimate_ = false;
  rclcpp::Time last_pose_time_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArUcoLocalizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
