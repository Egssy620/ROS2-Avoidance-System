// manage_node/src/manage_node.cpp
// 自律移動の全体管理ノード

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <ctime>
#include <fstream>
#include <cstdlib>

static inline double wrapToPi(double a){
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

enum class Phase { 
    HEADING,          // ゴール方向へ向く
    DRIVE,            // 前進
    TASK,             // タスク実行
    HEADING_TO_TASK,  // タスク用の姿勢に向く
    OBSTACLE_CHECK,   // フルスキャン
    DONE              // 終了
};

class ManageNode : public rclcpp::Node {
public:
  ManageNode() : Node("manage_node")
  {
    // ログディレクトリ作成
    log_dir_ = makeLogDirectory();
    std::string pose_path = log_dir_ + "/pose_log.csv";
    log_pose_file_.open(pose_path, std::ios::out);
    log_pose_file_ << "time,x,y,yaw\n";
    RCLCPP_INFO(this->get_logger(), "Pose log saved to: %s", pose_path.c_str());

    // ゴール定義
    goals_ = {
      { -1.5, -9.0 },  // ゴール0
      {  3.5, -3.0 },  // ゴール1
      {  1.0, -3.0 },  // ゴール2
      { -1.0, -3.0 },  // ゴール3
      { -3.5, -3.0 },  // ゴール4
    };
    goal_index_ = 0;
    goal_ = goals_[goal_index_];

    //  仮自己位置
    car_pos_ = {0.0, 0.0};
    car_yaw_ = 0.0;
    received_pose_ = false;

    // パラメータ
    lookahead_dist_ = this->declare_parameter("lookahead_distance", 1.5);  // LookAhead距離

    // Publisher
    heading_target_pub_ = this->create_publisher<std_msgs::msg::Float32>("/heading_target", 10);
    target_pub_         = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_point", 10);
    task_start_pub_     = this->create_publisher<std_msgs::msg::Bool>("/task_start", 10);
    fullscan_pub_       = this->create_publisher<std_msgs::msg::Bool>("/request_full_scan", 10);
    cmd_source_pub_     = this->create_publisher<std_msgs::msg::String>("/cmd_source", 10);

    {
      rclcpp::QoS latched(1);
      latched.transient_local();
      goal_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/current_goal", latched);
    }

    // Subscriber
    pose2d_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/robot_pose2d", 10,
      std::bind(&ManageNode::onPose2D, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/filtered_points", 10,
      std::bind(&ManageNode::onLidar, this, std::placeholders::_1));

    fullscan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/full_scan_points", 10,
      std::bind(&ManageNode::onFullScanReceived, this, std::placeholders::_1));

    marker_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/aruco_markers", 10,
      std::bind(&ManageNode::onMarkerArray, this, std::placeholders::_1));

    marker_id_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/aruco_marker_ids", 10,
      std::bind(&ManageNode::onMarkerIDs, this, std::placeholders::_1));

    heading_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/heading_done", 10,
      std::bind(&ManageNode::onHeadingDone, this, std::placeholders::_1));

    drive_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/drive_done", 10,
      std::bind(&ManageNode::onDriveDone, this, std::placeholders::_1));

    task_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/task_done", 10,
      std::bind(&ManageNode::onTaskDone, this, std::placeholders::_1));

    //  Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ManageNode::onTimer, this));

    second_scan_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ManageNode::onSecondScanTimer, this));
    second_scan_timer_->cancel();

    fullscan_timeout_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&ManageNode::onFullScanTimeout, this));
    fullscan_timeout_timer_->cancel();

    wait_counter_sec_    = 0;
    waiting_second_scan_ = false;
    waiting_fullscan_    = false;

    // 初期フェーズ
    phase_ = Phase::HEADING;

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    RCLCPP_INFO(this->get_logger(), "ManageNode 起動");
  }

  ~ManageNode(){
    if (log_pose_file_.is_open())
      log_pose_file_.close();
  }

private:
  // ============================================================
  //                      VOICEVOX スピーカ
  // ============================================================
  std::string makeLogDirectory()
  {
    const char* home = std::getenv("HOME");
    if (!home) home = "/tmp"; // fallback

    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << home << "/ros2_logs/" << std::put_time(&tm, "%Y-%m-%d");
    std::string dir_path = oss.str();

    std::filesystem::create_directories(dir_path);
    return dir_path;
  }
  
  void playVoice(const std::string &text)
  {
    auto url_encode = [&](const std::string &s) {
      std::ostringstream encoded;
      encoded << std::hex << std::uppercase;
      for (unsigned char c : s) {
        if (isalnum(c) || c=='-'||c=='_'||c=='.'||c=='~')
          encoded << c;
        else
          encoded << '%' << std::setw(2) << int(c);
      }
      return encoded.str();
    };

    std::string encoded_text = url_encode(text);
    std::string cmd =
      "curl -s -X POST 'http://localhost:50021/audio_query?text=" + encoded_text +
      "&speaker=3' "
      "| curl -s -H 'Content-Type: application/json' -X POST -d @- "
      "'http://localhost:50021/synthesis?speaker=3' "
      "-o /tmp/voice.wav && aplay -q /tmp/voice.wav";

    RCLCPP_INFO(this->get_logger(), "[VOICE] %s", text.c_str());
    std::system(cmd.c_str());
  }

  // ============================================================
  //                  ゴール近傍に障害物があるか
  // ============================================================
  bool isGoalBlocked()
  {
    if (!received_pose_) return false;
    if (lidar_points_.empty()) return false;

    const double R_GOAL   = 0.3; // ゴール周囲半径
    const int    MIN_PTS  = 5;   // 最低点数

    cv::Point2d g = goal_;
    int cnt = 0;

    for (auto &p : lidar_points_) {
      double d_goal = cv::norm(p - g);
      if (d_goal < R_GOAL) cnt++;
    }

    if (cnt >= MIN_PTS) {
      RCLCPP_WARN(this->get_logger(),
                  "Goal #%d blocked: %d points within %.2f m",
                  goal_index_, cnt, R_GOAL);
      return true;
    }
    return false;
  }

  // ============================================================
  //                         Goal スキップ
  // ============================================================
  void skipCurrentGoal()
  {
    // 一旦停止
    std_msgs::msg::String stop_src;
    stop_src.data = "none";
    cmd_source_pub_->publish(stop_src);

    if (std::find(skipped_goals_.begin(), skipped_goals_.end(), goal_index_) 
        == skipped_goals_.end()) {
      skipped_goals_.push_back(goal_index_);
      char buf[200];
      std::snprintf(buf, sizeof(buf),
                    "ゴール%d番の付近に障害物があります。スキップします。",
                    goal_index_);
      playVoice(buf);
    }

    // 次のゴールへ
    goal_index_ = (goal_index_ + 1) % goals_.size();
    goal_ = goals_[goal_index_];

    double dx = goal_.x - car_pos_.x;
    double dy = goal_.y - car_pos_.y;
    double yaw = std::atan2(dy, dx);

    std_msgs::msg::Float32 msg;
    msg.data = yaw;
    heading_target_pub_->publish(msg);

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING;

    RCLCPP_WARN(this->get_logger(),
                "Skip → Next goal %d に向けて HEADING (yaw=%.3f)",
                goal_index_, yaw);
  }

  // ============================================================
  //                        Heading Done
  // ============================================================
  void onHeadingDone(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) return;

    if (phase_ == Phase::OBSTACLE_CHECK) {
      RCLCPP_INFO(this->get_logger(),
                  "OBSTACLE_CHECK: heading done → full scan start");
      startObstacleScan();
      return;
    }

    if (phase_ == Phase::HEADING_TO_TASK) {
      RCLCPP_INFO(this->get_logger(),
                  "HEADING_TO_TASK done → TASK 開始");

      phase_ = Phase::TASK;

      std_msgs::msg::Bool start_msg;
      start_msg.data = true;
      task_start_pub_->publish(start_msg);

      std_msgs::msg::String src;
      src.data = "task";
      cmd_source_pub_->publish(src);
      return;
    }

    if (phase_ == Phase::HEADING) {
      // 経路生成
      updatePathToCurrentGoal();
      heading_sent_ = false;

      std_msgs::msg::String src;
      src.data = "drive";
      cmd_source_pub_->publish(src);
      phase_ = Phase::DRIVE;
      return;
    }
  }

  // ============================================================
  //                        Drive Done
  // ============================================================
  void onDriveDone(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) return;
    if (phase_ != Phase::DRIVE) return;

    // Goal0に戻ってきた かつ スキップゴールあり → 障害物チェック
    if (goal_index_ == 0 && !skipped_goals_.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Returned to Goal0 with skipped goals → OBSTACLE_CHECK");

      double yaw_origin = std::atan2(0.0 - car_pos_.y, 0.0 - car_pos_.x);

      std_msgs::msg::Float32 msg_yaw;
      msg_yaw.data = yaw_origin;
      heading_target_pub_->publish(msg_yaw);

      std_msgs::msg::String src;
      src.data = "heading";
      cmd_source_pub_->publish(src);

      phase_ = Phase::OBSTACLE_CHECK;
      return;
    }

    double back_yaw = -M_PI / 2.0;  // タスク時の姿勢
    std_msgs::msg::Float32 msg_yaw;
    msg_yaw.data = back_yaw;
    heading_target_pub_->publish(msg_yaw);

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING_TO_TASK;
  }

  // ============================================================
  //                        Task Done
  // ============================================================
  void onTaskDone(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) return;
    if (phase_ != Phase::TASK) return;

    RCLCPP_INFO(this->get_logger(),
                "Task done → heading toward NEXT goal");

    // 次のゴールに更新
    goal_index_ = (goal_index_ + 1) % goals_.size();
    goal_ = goals_[goal_index_];

    double dx = goal_.x - car_pos_.x;
    double dy = goal_.y - car_pos_.y;
    double next_dir = std::atan2(dy, dx);

    std_msgs::msg::Float32 msg_target;
    msg_target.data = next_dir;
    heading_target_pub_->publish(msg_target);

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING;
  }

  // ============================================================
  //                         Pose 受信
  // ============================================================
  void onPose2D(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    const double dx_cam = 0.0;
    const double dy_cam = -0.355;

    double yaw_cam = msg->theta;
    double x_robot = msg->x + dx_cam * std::cos(yaw_cam) - dy_cam * std::sin(yaw_cam);
    double y_robot = msg->y + dx_cam * std::sin(yaw_cam) + dy_cam * std::cos(yaw_cam);

    // 描画・運動モデルのための yaw 補正
    double yaw_robot = yaw_cam + M_PI/2.0;

    car_pos_ = cv::Point2d(x_robot, y_robot);
    car_yaw_ = wrapToPi(yaw_robot);

    // 初回受信処理
    if (!received_pose_) {
        received_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "初回Pose受信: 経路を初期化します");
        updatePathToCurrentGoal();
    }

    trajectory_.push_back(car_pos_);
    if (trajectory_.size() > 200)
      trajectory_.erase(trajectory_.begin());

    // 起動直後の初回 HEADING のための heading_target
    if (phase_ == Phase::HEADING && !heading_sent_) {
      double dx = goal_.x - car_pos_.x;
      double dy = goal_.y - car_pos_.y;
      double theta_target = std::atan2(dy, dx);

      std_msgs::msg::Float32 msg_target;
      msg_target.data = theta_target;
      heading_target_pub_->publish(msg_target);

      heading_sent_ = true;
    }
  }

  // ============================================================
  //                       Marker / ID
  // ============================================================
  void onMarkerArray(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    marker_points_.clear();
    for (auto &p : msg->poses)
      marker_points_.emplace_back(p.position.x, p.position.y);
  }

  void onMarkerIDs(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    marker_ids_ = msg->data;
  }

  // ============================================================
  //                         LiDAR（通常）
  // ============================================================
  void onLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    lidar_points_.clear();

    if (!received_pose_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "自己位置未受信のため LiDAR 点群を描画できません");
      return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    const double cx = car_pos_.x;
    const double cy = car_pos_.y;
    const double cth = car_yaw_;
    const double c = std::cos(cth);
    const double s = std::sin(cth);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      double x_local = *iter_x;
      double y_local = *iter_y;
      double z_local = *iter_z;

      if (!std::isfinite(x_local) || !std::isfinite(y_local)) continue;
      if (z_local < -0.2 || z_local > 1.0) continue;

      double x_map = c * x_local - s * y_local + cx;
      double y_map = s * x_local + c * y_local + cy;

      lidar_points_.emplace_back(x_map, y_map);
    }
  }

  // ============================================================
  //                       フルスキャン要求
  // ============================================================
  void requestFullScan()
  {
    std_msgs::msg::Bool msg;
    msg.data = true;
    fullscan_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
                "Sent full scan request to LiDAR node");

    waiting_fullscan_ = true;
    fullscan_timeout_timer_->reset();
  }

  void startObstacleScan()
  {
    playVoice("ゴール付近に障害物を確認しました。スキャンします。");
    requestFullScan();

    std_msgs::msg::String src;
    src.data = "none";
    cmd_source_pub_->publish(src);
  }

  // skipped_goals_ 付近の障害物だけを抽出
  std::vector<cv::Point2d> extractSkippedGoalObstacles(
      const std::vector<cv::Point2d>& pts)
  {
    std::vector<cv::Point2d> result;
    const double R = 1.5;

    for (int gid : skipped_goals_) {
      cv::Point2d g = goals_[gid];
      for (auto &p : pts) {
        if (cv::norm(p - g) < R) {
          result.push_back(p);
        }
      }
    }
    return result;
  }

  void onFullScanTimeout()
  {
    if (!waiting_fullscan_)
      return;

    RCLCPP_ERROR(this->get_logger(),
        "[ERROR] full_scan timeout: LiDAR node not responding!");

    waiting_fullscan_    = false;
    waiting_second_scan_ = false;
    skipped_goals_.clear();

    playVoice("LiDARが応答しません。通常巡回に戻ります。");

    double back_yaw = M_PI / 2.0;

    std_msgs::msg::Float32 msg;
    msg.data = back_yaw;
    heading_target_pub_->publish(msg);

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING_TO_TASK;
  }

  // フルスキャン受信
  void onFullScanReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    waiting_fullscan_ = false;
    fullscan_timeout_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Received full scan");

    std::vector<cv::Point2d> full_pts;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    double cx = car_pos_.x;
    double cy = car_pos_.y;
    double cth = car_yaw_;
    double c = std::cos(cth);
    double s = std::sin(cth);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y)) continue;
      if (*iter_z < -0.3 || *iter_z > 1.2) continue;

      double x_map = c * (*iter_x) - s * (*iter_y) + cx;
      double y_map = s * (*iter_x) + c * (*iter_y) + cy;
      full_pts.emplace_back(x_map, y_map);
    }

    if (!waiting_second_scan_) {
      // 1回目
      auto obs = extractSkippedGoalObstacles(full_pts);

      if (!obs.empty()) {
        const auto &p = obs.front();
        char buf[200];
        std::snprintf(buf, sizeof(buf),
             "ゴール付近の障害物を取り除いてください。座標は、%.2fメートル、%.2fメートル付近です。",
             p.x, p.y);
        playVoice(buf);
      } else {
        playVoice("スキップしたゴール付近に明確な障害物は検出されませんでした。");
      }

      waiting_second_scan_ = true;
      wait_counter_sec_ = 0;
      second_scan_timer_->reset();
      RCLCPP_INFO(this->get_logger(),
                  "[OBSTACLE_CHECK] First full scan done. Waiting 30 sec for second scan.");
      return;
    }

    // 2回目
    waiting_second_scan_ = false;
    second_scan_timer_->cancel();

    auto remaining = extractSkippedGoalObstacles(full_pts);

    if (remaining.empty()) {
      playVoice("障害物が除去されました。タスクを再開します。");
    } else {
      playVoice("まだ障害物がありますが、巡回を再開します。");
    }

    skipped_goals_.clear();

    double back_yaw = M_PI/2.0;
    std_msgs::msg::Float32 msg_yaw;
    msg_yaw.data = back_yaw;
    heading_target_pub_->publish(msg_yaw);

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING_TO_TASK;
  }

  // カウントダウン
  void onSecondScanTimer()
  {
    if (!waiting_second_scan_) {
      second_scan_timer_->cancel();
      return;
    }

    wait_counter_sec_++;
    RCLCPP_INFO(this->get_logger(),
        "[OBSTACLE_CHECK] wait %d / 30 sec", wait_counter_sec_);

    if (wait_counter_sec_ >= 30) {
      second_scan_timer_->cancel();
      playVoice("再スキャンします。");
      requestFullScan();
    }
  }

  // ============================================================
  //                        経路生成
  // ============================================================
  void updatePathToCurrentGoal()
  {
    path_.clear();
    cv::Point2d start = car_pos_;
    cv::Point2d goal  = goal_;
    const int N = 20;
    for (int i=0; i<=N; ++i) {
      double t = static_cast<double>(i) / N;
      double x = (1-t)*start.x + t*goal.x;
      double y = (1-t)*start.y + t*goal.y;
      path_.emplace_back(x, y);
    }
    heading_sent_ = false;
  }

  // Pure Pursuit 用の注視点
  cv::Point2d computeLookaheadPoint()
  {
    const double r = lookahead_dist_;
    const cv::Point2d C = car_pos_;

    cv::Point2d best_pt = goal_;
    bool found = false;

    for (size_t i=0; i+1<path_.size(); ++i) {
      cv::Point2d P1 = path_[i];
      cv::Point2d P2 = path_[i+1];
      cv::Point2d d  = P2 - P1;

      double a = d.dot(d);
      double b = 2 * ((P1.x-C.x)*d.x + (P1.y-C.y)*d.y);
      double c = (P1.x-C.x)*(P1.x-C.x) + (P1.y-C.y)*(P1.y-C.y) - r*r;

      double D = b*b - 4*a*c;
      if (D < 0) continue;

      double sqrtD = std::sqrt(D);
      double t1 = (-b + sqrtD) / (2*a);
      double t2 = (-b - sqrtD) / (2*a);

      std::vector<double> ts;
      if (t1 >= 0.0 && t1 <= 1.0) ts.push_back(t1);
      if (t2 >= 0.0 && t2 <= 1.0) ts.push_back(t2);

      if (!ts.empty()) {
        double t = *std::max_element(ts.begin(), ts.end());
        best_pt = P1 + t*d;
        found = true;
      }
    }

    double dist_to_goal = cv::norm(goal_ - car_pos_);
    if (dist_to_goal < lookahead_dist_) {
      best_pt = goal_;
      found = true;
    }

    return found ? best_pt : goal_;
  }

  double computePathLengthAtPoint(const cv::Point2d& pt)
  {
    double total = 0.0;
    for (size_t i=1; i<path_.size(); ++i) {
      double seg_len = cv::norm(path_[i] - path_[i-1]);
      if (cv::norm(pt - path_[i]) < seg_len) {
        total += cv::norm(pt - path_[i-1]);
        break;
      }
      total += seg_len;
    }
    return total;
  }

  // ============================================================
  //                         可視化 Timer
  // ============================================================
  cv::Point toPix(const cv::Point2d& pt, double scale, const cv::Point2d& offset) {
    double x_vis = pt.x;
    double y_vis = -pt.y;
    return cv::Point(static_cast<int>(offset.x + x_vis*scale),
                     static_cast<int>(offset.y + y_vis*scale));
  }

  void onTimer()
  {
    // === ゴール付近の危険チェック (DRIVE中のみ) ===
    if (phase_ == Phase::DRIVE) {
      if (isGoalBlocked()) {
        skipCurrentGoal();
        return;
      }
    }

    static bool window_initialized = false;
    if (!window_initialized) {
      cv::namedWindow("Manage Visualization", cv::WINDOW_NORMAL);
      cv::moveWindow("Manage Visualization", 50, 50);
      window_initialized = true;
    }

    const int W = 1920, H = 480;
    const double scale = 30.0;
    const cv::Point2d offset(W/2.0, H*0.25);

    cv::Mat canvas(H, W, CV_8UC3, cv::Scalar(255,255,255));

    // 経路線
    for (size_t i=0; i+1<path_.size(); ++i) {
      cv::line(canvas, toPix(path_[i],   scale, offset),
                        toPix(path_[i+1], scale, offset),
               cv::Scalar(0,0,0), 2);
    }

    // ゴール群
    for (size_t i=0; i<goals_.size(); ++i) {
      cv::Scalar color = (i == static_cast<size_t>(goal_index_))
                        ? cv::Scalar(0,0,0)
                        : cv::Scalar(180,180,180);
      cv::circle(canvas, toPix(goals_[i], scale, offset),
                 6, color, 2);
    }

    // マーカー描画
    for (size_t i=0; i<marker_points_.size(); ++i) {
      cv::Point2d m = marker_points_[i];
      cv::circle(canvas, toPix(m, scale, offset),
                 5, cv::Scalar(0,0,255), -1);
      if (i < marker_ids_.size()) {
        std::string label = "ID:" + std::to_string(marker_ids_[i]);
        cv::putText(canvas, label, toPix(m, scale, offset)+cv::Point(8,-8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 1);
      }
    }

    if (!received_pose_) {
      cv::imshow("Manage Visualization", canvas);
      cv::waitKey(1);
      return;
    }

    // 軌跡
    for (size_t i=1; i<trajectory_.size(); ++i) {
      cv::line(canvas, toPix(trajectory_[i-1], scale, offset),
                        toPix(trajectory_[i],   scale, offset),
               cv::Scalar(255,0,0), 1);
    }

    // LiDAR
    for (auto &p : lidar_points_) {
      cv::circle(canvas, toPix(p, scale, offset),
                 1, cv::Scalar(120,120,120), -1);
    }

    // 現在位置と向き
    cv::circle(canvas, toPix(car_pos_, scale, offset),
               6, cv::Scalar(255,0,0), -1);

    cv::Point2d head = car_pos_ + 0.6*cv::Point2d(std::cos(car_yaw_), std::sin(car_yaw_));
    cv::arrowedLine(canvas, toPix(car_pos_, scale, offset),
                            toPix(head,    scale, offset),
                    cv::Scalar(255,0,0), 2);

    // 注視円
    cv::circle(canvas, toPix(car_pos_, scale, offset),
               static_cast<int>(lookahead_dist_*scale),
               cv::Scalar(255,0,0), 1, cv::LINE_AA);

    // ① Pure Pursuit 注視点
    cv::Point2d lookahead = computeLookaheadPoint();
    cv::circle(canvas, toPix(lookahead, scale, offset),
               6, cv::Scalar(0,200,255), -1);

    // ② 障害物回避
    cv::Point2d selected_pt = lookahead;

    if (phase_ == Phase::DRIVE && !lidar_points_.empty()) {
      // 最近点
      double min_dist = 1e9;
      cv::Point2d nearest_pt;
      for (auto &p : lidar_points_) {
        double d = std::hypot(p.x - car_pos_.x, p.y - car_pos_.y);
        if (d < min_dist) {
          min_dist = d;
          nearest_pt = p;
        }
      }

      double r1 = lookahead_dist_;
      double r2 = 1.5;

      cv::Point2d C1 = car_pos_;
      cv::Point2d C2 = nearest_pt;
      double dx = C2.x - C1.x;
      double dy = C2.y - C1.y;
      double d = std::hypot(dx, dy);

      std::vector<cv::Point2d> intersections;

      if (d < (r1 + r2) && d > std::fabs(r1 - r2)) {
        double a = (r1*r1 - r2*r2 + d*d) / (2*d);
        double h = std::sqrt(std::max(r1*r1 - a*a, 0.0));

        cv::Point2d P0 = C1 + a * (C2 - C1) / d;
        double rx = -dy * (h / d);
        double ry =  dx * (h / d);

        intersections.emplace_back(P0.x + rx, P0.y + ry);
        intersections.emplace_back(P0.x - rx, P0.y - ry);
      }

      // 前方 ±90° の交点のみ採用
      std::vector<cv::Point2d> front_inters;
      for (auto &p : intersections) {
        double dx2 = p.x - car_pos_.x;
        double dy2 = p.y - car_pos_.y;
        double angle = wrapToPi(std::atan2(dy2, dx2) - car_yaw_);
        if (std::fabs(angle) < M_PI/2.0) {
          front_inters.push_back(p);
        }
      }

      if (!front_inters.empty()) {
        double best_dist = 1e9;
        for (auto &p : front_inters) {
          double dist = std::hypot(p.x - goal_.x, p.y - goal_.y);
          if (dist < best_dist) {
            best_dist = dist;
            selected_pt = p;
          }
        }

        // 描画
        for (auto &p : intersections) {
          cv::circle(canvas, toPix(p, scale, offset),
                     5, cv::Scalar(0,0,255), -1);
        }
        cv::circle(canvas, toPix(selected_pt, scale, offset),
                   7, cv::Scalar(0,0,0), 2);
      }
    }

    // ④ DriveNode 用 target_point を publish
    geometry_msgs::msg::PointStamped tgt_msg;
    tgt_msg.header.stamp = this->now();
    tgt_msg.header.frame_id = "map";
    tgt_msg.point.x = selected_pt.x;
    tgt_msg.point.y = selected_pt.y;
    tgt_msg.point.z = 0.0;
    target_pub_->publish(tgt_msg);

    // ゴールを publish
    geometry_msgs::msg::Pose2D goal_msg;
    goal_msg.x = goal_.x;
    goal_msg.y = goal_.y;
    goal_msg.theta = 0.0;
    goal_pub_->publish(goal_msg);

    cv::imshow("Manage Visualization", canvas);
    cv::waitKey(1);
  }

  // ============================================================
  //                         メンバ変数
  // ============================================================
  // Sub / Pub / Timer
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr marker_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr marker_id_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr fullscan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heading_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr task_done_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            heading_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr        goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               task_start_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               fullscan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             cmd_source_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr second_scan_timer_;
  rclcpp::TimerBase::SharedPtr fullscan_timeout_timer_;

  // full_scan 関連
  bool waiting_fullscan_    = false;
  bool waiting_second_scan_ = false;
  int  wait_counter_sec_    = 0;

  // 状態管理
  Phase phase_;
  bool received_pose_ = false;
  bool heading_sent_  = false;

  cv::Point2d car_pos_;
  double car_yaw_;
  std::vector<cv::Point2d> trajectory_;

  std::vector<cv::Point2d> pos_buffer_;
  std::vector<double>       yaw_buffer_;

  std::vector<cv::Point2d> goals_;
  int        goal_index_;
  cv::Point2d goal_;

  double lookahead_dist_;
  std::vector<cv::Point2d> path_;

  std::vector<cv::Point2d> marker_points_;
  std::vector<int>         marker_ids_;

  std::vector<cv::Point2d> lidar_points_;

  // ゴールスキップ関連
  std::vector<int> skipped_goals_;

  std::ofstream log_pose_file_;
  std::string   log_dir_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManageNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
