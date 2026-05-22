// main.cpp - ROS2 入口
//
// 流程：
//   1. 初始化 ROS2 节点
//   2. 调 Calculator::run() 拿到经过众数统计后的稳定结果
//   3. 在话题 vip_box_id 上以 robot_interfaces::msg::Int 发布该结果
//   4. 发完一次后 spin 一小段时间确保消息送出，然后退出

#include "arithmetic_problem/calculate.h"

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/int.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

namespace {

// 路径与原 arithmetic.cpp 中保持一致；如需迁移机器，集中修改这里即可。
constexpr const char* kCameraConfig =
    "/home/yuan/Vscode_word/awork_mycode/arithmetic_problem/include/camera_driver/camera_init/HIKcamera0.yaml";
constexpr const char* kOnnxModel =
    "/home/yuan/Vscode_word/awork_mycode/arithmetic_problem/src/best.onnx";

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("arithmetic_problem_node");
    auto result_publisher =
        node->create_publisher<robot_interfaces::msg::Int>("vip_box_id", 10);
    auto calculate_publisher =
        node->create_publisher<robot_interfaces::msg::Int>("calculate", 10);

    std::atomic_bool start_calculate{false};
    auto calculate_subscription =
        node->create_subscription<robot_interfaces::msg::Int>(
            "calculate",
            10,
            [&](const robot_interfaces::msg::Int::SharedPtr msg) {
                if (msg->data != 1 || start_calculate.load()) {
                    return;
                }

                robot_interfaces::msg::Int feedback;
                feedback.data = 1;
                calculate_publisher->publish(feedback);
                start_calculate.store(true);
                RCLCPP_INFO(node->get_logger(), "收到 calculate=1，已反馈 1，开始计算");
            });

    RCLCPP_INFO(node->get_logger(), "等待 calculate 话题收到 1 后开始计算");
    while (rclcpp::ok() && !start_calculate.load()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!rclcpp::ok()) {
        return 0;
    }

    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);

    // ---- 调 calculate 库拿稳定结果 ----
    arithmetic_problem::Calculator::Config cfg;
    cfg.camera_config_path = kCameraConfig;
    cfg.onnx_model_path = kOnnxModel;
    cfg.show_window = false;  // 节点跑起来通常不需要弹窗

    arithmetic_problem::Calculator calculator(cfg);
    const auto result = calculator.run();

    if (!result.valid) {
        RCLCPP_WARN(node->get_logger(),
                    "Calculator 没有产出有效结果（样本数=%zu），仍将发布当前众数 %d",
                    result.sample_count, result.answer);
    }

    // ---- 发布一次结果 ----
    robot_interfaces::msg::Int msg;
    msg.data = result.answer;
    result_publisher->publish(msg);

    RCLCPP_INFO(node->get_logger(),
                "已向 vip_box_id 发布结果: %d (占比 %.1f%%, 样本数 %zu)",
                msg.data, result.dominance * 100.0, result.sample_count);

    // 给 DDS 一点时间把消息真正发出去
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    rclcpp::spin_some(node);

    rclcpp::shutdown();
    return 0;
}
