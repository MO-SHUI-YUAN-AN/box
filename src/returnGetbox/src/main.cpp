#include <chrono>
#include <memory>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/vis.hpp>

namespace
{
constexpr int kCameraIndex = 0;
constexpr int kPublishQueueSize = 10;
constexpr int kTimerIntervalMs = 100;
constexpr double kStartupDelaySeconds = 4.5;
constexpr double kBrightnessThreshold = 60.0;
}

class GripDetectNode : public rclcpp::Node
{
public:
  GripDetectNode()
  : Node("grip_detect_node"), publisher_(create_publisher<robot_interfaces::msg::Vis>("detect_result", kPublishQueueSize))
  {
    camera_.open(kCameraIndex);
    if (!camera_.isOpened()) {
      throw std::runtime_error("无法打开 USB 摄像头，索引: " + std::to_string(kCameraIndex));
    }

    RCLCPP_INFO(get_logger(), "USB 摄像头已打开，等待 %.1f 秒后开始检测", kStartupDelaySeconds);
    std::this_thread::sleep_for(std::chrono::duration<double>(kStartupDelaySeconds));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(kTimerIntervalMs),
      std::bind(&GripDetectNode::detectAndPublish, this));
  }

private:
  void detectAndPublish()
  {
    cv::Mat frame;
    if (!camera_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "读取摄像头画面失败");
      return;
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    const double brightness = cv::mean(gray)[0];
    const float result = brightness < kBrightnessThreshold ? 1.0F : -1.0F;

    robot_interfaces::msg::Vis msg;
    msg.x = result;
    publisher_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "当前亮度: %.2f, 阈值: %.2f, 抓取结果: %s",
      brightness,
      kBrightnessThreshold,
      result > 0.0F ? "成功(1)" : "失败(-1)");
  }

  cv::VideoCapture camera_;
  rclcpp::Publisher<robot_interfaces::msg::Vis>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<GripDetectNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("grip_detect_main"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
