#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <unordered_map>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <cmath>

// ZYX(Euler) from rotation matrix
static cv::Vec3d rotm2eulZYX(const cv::Mat& R)
{
  // R is 3x3, double
  double r11 = R.at<double>(0,0), r12 = R.at<double>(0,1), r13 = R.at<double>(0,2);
  double r21 = R.at<double>(1,0), r22 = R.at<double>(1,1), r23 = R.at<double>(1,2);
  double r31 = R.at<double>(2,0), r32 = R.at<double>(2,1), r33 = R.at<double>(2,2);

  double yaw   = std::atan2(r21, r11);                         // Z
  double pitch = std::atan2(-r31, std::sqrt(r32*r32 + r33*r33)); // Y
  double roll  = std::atan2(r32, r33);                         // X
  return cv::Vec3d(yaw, pitch, roll);
}

class ArUcoLocalizerNode : public rclcpp::Node
{
public:
  ArUcoLocalizerNode() : Node("aruco_localizer_node")
  {
    // カメラキャリブレーション読み込み
    cv::FileStorage fs("/home/ryuzo/ros2_ws/src/camera_calibration_pkg/calibration.yaml", cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;
    fs.release();
    RCLCPP_INFO(this->get_logger(), "Loaded calibration.yaml");

    pose_pub_  = this->create_publisher<geometry_msgs::msg::PoseStamped>("marker_pose", 10);
    pose2d_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("robot_pose2d", 10);

    // VideoCapture
    cap_.open(0);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Camera open failed!");
      rclcpp::shutdown();
      return;
    }

    cv::namedWindow("ArUcoLocalizer", cv::WINDOW_NORMAL);
    cv::resizeWindow("ArUcoLocalizer", 960, 540);

    // マーカー地図（MATLABコードと同一）
    // XY平面
    marker_length_ = 0.25;

    // 中心距離 [m]
    const double DX = 1.0; // 横
    const double DY = 2.5; // 縦

    // x列: [-DX, 0, +DX],  y行: [0, -DY, -2*DY]
    // 行0: 0,1,2 / 行1: 3,4,5 / 行2: 6,7,8
    marker_map_.clear();
    marker_map_[0] = cv::Vec3d(-DX,    0.0,   0.001);
    marker_map_[1] = cv::Vec3d( 0.0,   0.0,   0.001);
    marker_map_[2] = cv::Vec3d(+DX,    0.0,   0.001);
    marker_map_[3] = cv::Vec3d(-DX,   -DY,    0.001);
    marker_map_[4] = cv::Vec3d( 0.0,  -DY,    0.001);
    marker_map_[5] = cv::Vec3d(+DX,   -DY,    0.001);
    marker_map_[6] = cv::Vec3d(-DX, -2*DY,    0.001);
    marker_map_[7] = cv::Vec3d( 0.0, -2*DY,   0.001);
    marker_map_[8] = cv::Vec3d(+DX, -2*DY,    0.001);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), // 20 Hz
      std::bind(&ArUcoLocalizerNode::process, this));
  }

private:
  void process()
  {
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) return;

    // ArUco検出
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(frame, dict, corners, ids);

    if (!ids.empty()) {
      // 追加描画
      cv::aruco::drawDetectedMarkers(frame, corners, ids, cv::Scalar(0,255,0));

      // 対応点(画像2D, ワールド3D)を全マーカー分で作成
      std::vector<cv::Point2f> img_pts; img_pts.reserve(ids.size()*4);
      std::vector<cv::Point3f> obj_pts; obj_pts.reserve(ids.size()*4);

      const double half = marker_length_ * 0.5;

      for (size_t k = 0; k < ids.size(); ++k) {
        int id = ids[k];
        auto it = marker_map_.find(id);
        if (it == marker_map_.end()) continue;

        const cv::Vec3d c = it->second; // center (cx, cy, cz)
        // MATLAB順序: 左上→右上→右下→左下（Blender基準）
        // OpenCVのcornersも通常TL,TR,BR,BLなので対応させる
        std::vector<cv::Point3f> world_corners = {
          cv::Point3f(c[0]-half, c[1]+half, c[2]),
          cv::Point3f(c[0]+half, c[1]+half, c[2]),
          cv::Point3f(c[0]+half, c[1]-half, c[2]),
          cv::Point3f(c[0]-half, c[1]-half, c[2])
        };
        const auto& img_corners = corners[k];

        for (int j = 0; j < 4; ++j) {
          obj_pts.push_back(world_corners[j]);
          img_pts.push_back(img_corners[j]);
        }

        // ID描画（左上に）
        cv::putText(frame, "ID " + std::to_string(id),
          img_corners[0] + cv::Point2f(5, -5),
          cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
      }

      // 一括PnPでカメラ姿勢推定
      if (obj_pts.size() >= 4) {
        cv::Mat rvec, tvec;
        std::vector<int> inliers;
        bool ok = cv::solvePnPRansac(
          obj_pts, img_pts, camera_matrix_, dist_coeffs_,
          rvec, tvec, false, 2000, 1.0, 0.999, inliers,
          cv::SOLVEPNP_ITERATIVE);

        if (ok) {
          // R_cam_world, t_cam_world? solvePnPは X_cam = R * X_world + t
          cv::Mat Rcw;
          cv::Rodrigues(rvec, Rcw); // world->camera
          // カメラ姿勢(世界座標系): R_wc = Rcw^T, C_w = -Rcw^T * t
          cv::Mat Rwc = Rcw.t();
          cv::Mat Cw  = -Rwc * tvec;

          // Blender→MATLAB補正 T=[1,0,0;0,-1,0;0,0,-1] と同等を適用
          // R_corr = T * Rwc, t_corr = T * Cw
          cv::Mat T = (cv::Mat_<double>(3,3) << 1,0,0, 0,-1,0, 0,0,-1);
          cv::Mat Rcorr = T * Rwc;
          cv::Mat tcorr = T * Cw;

          // オイラー角ZYX(Yaw,Pitch,Roll)
          cv::Vec3d eul = rotm2eulZYX(Rcorr);
          auto deg = [](double r){ return r * 180.0 / M_PI; };

          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[CAMERA POSE - WORLD FRAME] Pos[m]=%.4f, %.4f, %.4f, YPR[deg]=%.2f, %.2f, %.2f",
            tcorr.at<double>(0), tcorr.at<double>(1), tcorr.at<double>(2),
            deg(eul[0]), deg(eul[1]), deg(eul[2]));

          // eul = rotm2eulZYX(Rcorr); // [yaw, pitch, roll]
          const double ros_yaw = -eul[0]; // 符号をROS向けに反転（CCWを＋）

          // PoseStamped (yawのみでQuaternion)
          geometry_msgs::msg::PoseStamped msg;
          msg.header.stamp = this->get_clock()->now();
          msg.header.frame_id = "world";
          msg.pose.position.x = tcorr.at<double>(0);
          msg.pose.position.y = tcorr.at<double>(1);
          msg.pose.position.z = tcorr.at<double>(2);
          const double half = 0.5 * ros_yaw;
          msg.pose.orientation.w = std::cos(half);
          msg.pose.orientation.x = 0.0;
          msg.pose.orientation.y = 0.0;
          msg.pose.orientation.z = std::sin(half);
          pose_pub_->publish(msg);

          // Pose2D
          geometry_msgs::msg::Pose2D p2d;
          p2d.x = tcorr.at<double>(0);
          p2d.y = tcorr.at<double>(1);
          p2d.theta = ros_yaw; // ここが要点
          pose2d_pub_->publish(p2d);
        }
      }
    }

    // 表示
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(960, 540));
    cv::imshow("ArUcoLocalizer", resized);
    cv::waitKey(1);
  }

  // メンバ
  cv::Mat camera_matrix_, dist_coeffs_;
  cv::VideoCapture cap_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<int, cv::Vec3d> marker_map_;
  double marker_length_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArUcoLocalizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

