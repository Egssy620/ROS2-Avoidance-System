// task_node/src/task_node.cpp
// タスク実行ノード(仮)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cstdlib>
#include <string>
#include <sstream>
#include <iomanip>

class TaskNode : public rclcpp::Node {
public:
  TaskNode() : Node("task_node"), active_(false), elapsed_(0.0) {
    task_start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/task_start", 10,
      std::bind(&TaskNode::onTaskStart, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/task/cmd_vel", 10);
    task_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/task_done", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TaskNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "TaskNode 起動完了");
  }

private:
  // 音声再生関数
  void speak(const std::string &text) {
    auto url_encode = [](const std::string &s) {
      std::ostringstream encoded;
      encoded << std::hex << std::uppercase;
      for (unsigned char c : s) {
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
          encoded << c;
        } else {
          encoded << '%' << std::setw(2) << int(c);
        }
      }
      return encoded.str();
    };

    std::string encoded_text = url_encode(text);
    std::string cmd =
      "curl -s -X POST 'http://localhost:50021/audio_query?text=" + encoded_text +
      "&speaker=3' "
      "| curl -s -H 'Content-Type: application/json' -X POST -d @- "
      "http://localhost:50021/synthesis?speaker=3 "
      "-o /tmp/voice.wav && aplay -f S16_LE -r 48000 -c 2 -q /tmp/voice.wav";

    RCLCPP_INFO(this->get_logger(), "Speak: %s", text.c_str());
    std::system(cmd.c_str());
  }

  void onTaskStart(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !active_) {
      active_ = true;
      elapsed_ = 0.0;
      RCLCPP_INFO(this->get_logger(), "タスク開始: 5秒間ゼロ速度を送信します");

      // タスク開始時
      speak("タスクを開始するのだ");
    }
  }

  void onTimer() {
    if (!active_) return;

    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_pub_->publish(stop);

    elapsed_ += 0.1;
    if (elapsed_ >= 5.0) {
      active_ = false;
      RCLCPP_INFO(this->get_logger(), "タスク完了: 5秒経過 -> task_done送信");

      std_msgs::msg::Bool done;
      done.data = true;
      task_done_pub_->publish(done);

      // タスク終了時
      speak("タスクが完了したのだ");
    }
  }

  bool active_;
  double elapsed_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr task_start_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr task_done_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskNode>());
  rclcpp::shutdown();
  return 0;
}