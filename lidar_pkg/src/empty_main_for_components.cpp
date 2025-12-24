#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto options = rclcpp::NodeOptions();
  // コンポーネントは自動ロード（リンク済み）されます
  auto loader = std::make_shared<rclcpp_components::ComponentManager>(options);
  exec.add_node(loader);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

