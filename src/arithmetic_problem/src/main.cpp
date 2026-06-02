// main.cpp - arithmetic_problem 的 ROS2 节点入口
//
// 这个文件只负责 ROS2 通信和程序启动顺序：
//   1. 创建 ROS2 节点
//   2. 等待 calculate 话题收到启动信号 1
//   3. 收到后向 calculate 话题回发 1，作为握手反馈
//   4. 调用 Calculator 完成算式识别、计算、众数统计
//   5. 把最终结果发布到 vip_box_id 话题，然后退出

#include "arithmetic_problem/calculate.h"

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/int.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

namespace {

// 相机配置和模型路径集中放在这里，后续迁移机器或改成 ROS2 参数时只需要改这一处。
constexpr const char* kCameraConfig =
    "/home/yuan/Vscode_word/awork_mycode/arithmetic_problem/include/camera_driver/camera_init/HIKcamera0.yaml";
constexpr const char* kOnnxModel =
    "/home/yuan/Vscode_word/awork_mycode/arithmetic_problem/src/best.onnx";

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("arithmetic_problem_node");

    // 最终识别结果发布到 vip_box_id，消息内容为对 4 取模后的答案（1-4）。
    auto result_publisher =
        node->create_publisher<robot_interfaces::msg::Int>("vip_box_id", 10);

    // calculate 话题同时用于接收启动信号和回发握手反馈。
    auto calculate_publisher =
        node->create_publisher<robot_interfaces::msg::Int>("calculate", 10);

    // 启动开关：只有收到 calculate=1 后才会开始创建相机和推理器。
    // 用 atomic 是为了保证回调和主循环之间读写状态安全。
    std::atomic_bool start_calculate{false};

    auto calculate_subscription =
        node->create_subscription<robot_interfaces::msg::Int>(
            "calculate",
            10,
            [&](const robot_interfaces::msg::Int::SharedPtr msg) {
                // 只响应第一次 data=1 的消息；后续重复消息直接忽略。
                if (msg->data != 1 || start_calculate.load()) {
                    return;
                }

                // 收到启动信号后，先向同一话题回发 1，通知上游本节点已经接收到命令。
                robot_interfaces::msg::Int feedback;
                feedback.data = 1;
                calculate_publisher->publish(feedback);

                start_calculate.store(true);
                RCLCPP_INFO(node->get_logger(), "收到 calculate=1，已反馈 1，开始计算");
            });

    RCLCPP_INFO(node->get_logger(), "等待 calculate 话题收到 1 后开始计算");

    // 在这里阻塞等待启动开关。spin_some 会触发订阅回调，sleep 避免空转占满 CPU。
    // while (rclcpp::ok() && !start_calculate.load()) {
    //     rclcpp::spin_some(node);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    if (!rclcpp::ok()) {
        return 0;
    }

    // 再 spin 两次，给 DDS 留出一点时间把握手反馈发出去。
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);

    // ---- 调 calculate 库拿稳定结果 ----
    arithmetic_problem::Calculator::Config cfg;
    cfg.camera_config_path = kCameraConfig;
    cfg.onnx_model_path = kOnnxModel;
    cfg.show_window = false;  // 节点运行时默认不弹 OpenCV 窗口。

    // Calculator 内部会完成：相机取帧、ONNX 推理、算式筛选、计算、众数统计。
    arithmetic_problem::Calculator calculator(cfg);
    const auto result = calculator.run();

    if (!result.valid) {
        RCLCPP_WARN(node->get_logger(),
                    "Calculator 没有产出有效结果（样本数=%zu），仍将发布当前众数 %d",
                    result.sample_count, result.answer);
    }

    // 发布一次最终答案到 vip_box_id。
    robot_interfaces::msg::Int msg;
    msg.data = result.answer;
    result_publisher->publish(msg);

    RCLCPP_INFO(node->get_logger(),
                "已向 vip_box_id 发布结果: %d (占比 %.1f%%, 样本数 %zu)",
                msg.data, result.dominance * 100.0, result.sample_count);

    // 给 DDS 一点时间把最终结果真正发出去。
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    rclcpp::spin_some(node);

    rclcpp::shutdown();
    return 0;
}
