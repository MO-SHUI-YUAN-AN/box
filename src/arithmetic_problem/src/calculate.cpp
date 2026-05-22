// calculate.cpp - Calculator 类实现
//
// 流程严格对应 need.md 中"原有功能实现流程"的 1-6 步：
//   1. 从工业相机获取图像
//   2. opencv dnn 加载 onnx 模型推理 outputs
//   3. 遍历 outputs 拿到每个 output.box
//   4. 用运算符号 special_box 做行筛选 + 高度筛选
//   5. 按 x 坐标拼算式计算，结果对 4 取模
//   6. cout 输出结果
// 在此基础上加入了多次结果的众数统计：最多 100 项，最少 20 项；
// 当某项占比 > 30% 或运行超过 5 秒就返回当前众数。

#include "arithmetic_problem/calculate.h"

#include "arithmetic_problem/arithmetic_utils.hpp"
#include "camera_driver.h"
#include "inference.h"

#include <opencv2/core/cuda.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace arithmetic_problem {

Calculator::Calculator(const Config& config) : config_(config) {
    camera_ = std::make_unique<Camera>(config_.camera_config_path);
    if (!camera_->isOpened()) {
        std::cerr << "[Calculator] 相机打开失败: " << config_.camera_config_path << std::endl;
    }

    const std::string empty_classes_path;
    detector_ = std::make_unique<Inference>(
        config_.onnx_model_path,
        config_.model_input_shape,
        empty_classes_path,
        config_.run_with_cuda);

    // 预热一次，吃掉首次推理的额外开销
    cv::Mat warmup = cv::Mat::zeros(config_.model_input_shape, CV_8UC3);
    detector_->runInference(warmup);
}

Calculator::~Calculator() {
    if (config_.show_window) {
        cv::destroyAllWindows();
    }
}

Calculator::Result Calculator::run() {
    Result result{};

    if (camera_ == nullptr || !camera_->isOpened()) {
        std::cerr << "[Calculator] 相机不可用，直接返回" << std::endl;
        return result;
    }

    // 环境信息打印一次
    std::cout << "===== Calculator 启动 =====" << std::endl;
    std::cout << "OpenCV 版本: " << CV_VERSION
              << ", CUDA 设备数: " << cv::cuda::getCudaEnabledDeviceCount() << std::endl;

    if (config_.show_window) {
        cv::namedWindow("Arithmetic Recognition", cv::WINDOW_NORMAL);
    }

    // 用 unordered_map 做频率统计，answer(1-4) -> count
    std::unordered_map<int, std::size_t> histogram;
    std::vector<int> samples;
    samples.reserve(config_.max_samples);

    const auto start_time = std::chrono::steady_clock::now();

    while (samples.size() < config_.max_samples) {
        // 超时检查：超过 timeout 立刻退出循环
        auto now = std::chrono::steady_clock::now();
        if (now - start_time >= config_.timeout) {
            std::cout << "[Calculator] 已超过 " << config_.timeout.count()
                      << " ms，停止采样" << std::endl;
            break;
        }

        // ---- Step 1: 从工业相机获取图像 ----
        cv::Mat frame;
        if (!camera_->getFrame(frame) || frame.empty()) {
            continue;
        }

        // ---- Step 2: ONNX 推理 ----
        std::vector<Detection> outputs = detector_->runInference(frame);

        // ---- Step 3+4: 遍历 + 用 special_box 筛选 ----
        const Detection* special_box = nullptr;
        for (const auto& d : outputs) {
            if (isOperator(d.class_id)) {
                special_box = &d;
                break;
            }
        }
        if (special_box == nullptr) {
            continue;
        }

        const int ref_y = special_box->box.y;
        const int ref_h = special_box->box.height;

        std::vector<Detection> filtered;
        filtered.reserve(outputs.size());
        for (const auto& d : outputs) {
            // y 坐标相差大于一个 special_box 高度 → 不在同一行
            if (std::abs(d.box.y - ref_y) > ref_h) continue;
            // 同行但框过大（> 2 倍 special_box 高度）→ 视为干扰
            if (d.box.height > 2 * ref_h) continue;
            filtered.push_back(d);
        }
        if (filtered.empty()) {
            continue;
        }

        // ---- Step 5: 按 x 坐标拼算式 + 计算 + 对 4 取模 ----
        std::sort(filtered.begin(), filtered.end(),
                  [](const Detection& a, const Detection& b) {
                      return a.box.x < b.box.x;
                  });

        std::string expression;
        expression.reserve(filtered.size());
        for (const auto& d : filtered) {
            expression += classIdToChar(d.class_id);
        }
        expression = fixParentheses(expression);

        const long long raw = calcExpression(expression);
        const int answer = modTo1_4(raw);

        // ---- Step 6: cout 输出 ----
        std::cout << "[Calculator] 算式: " << expression
                  << " | 原始: " << raw
                  << " | mod4: " << answer << std::endl;

        // 加入统计
        samples.push_back(answer);
        histogram[answer] += 1;
        result.last_expression = expression;

        // 可视化（可选）
        if (config_.show_window) {
            cv::Mat display = frame.clone();
            for (const auto& d : filtered) {
                cv::rectangle(display, d.box, cv::Scalar(0, 255, 0), 2);
                cv::putText(display, d.className,
                            cv::Point(d.box.x, d.box.y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7,
                            cv::Scalar(0, 255, 0), 2);
            }
            std::string info = "Expr: " + expression + "  mod4 = " + std::to_string(answer);
            cv::putText(display, info, cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0,
                        cv::Scalar(0, 0, 255), 2);
            cv::imshow("Arithmetic Recognition", display);
            char key = static_cast<char>(cv::waitKey(1));
            if (key == 'q' || key == 27) break;
        }

        // 达到最少样本数后才检查"占比 > 30%"的提前返回条件
        if (samples.size() >= config_.min_samples) {
            int top_value = 0;
            std::size_t top_count = 0;
            for (const auto& kv : histogram) {
                if (kv.second > top_count) {
                    top_count = kv.second;
                    top_value = kv.first;
                }
            }
            const double dominance =
                static_cast<double>(top_count) / static_cast<double>(samples.size());
            if (dominance > config_.dominance_threshold) {
                result.valid = true;
                result.answer = top_value;
                result.dominance = dominance;
                result.sample_count = samples.size();
                std::cout << "[Calculator] 提前收敛: 众数=" << top_value
                          << ", 占比=" << (dominance * 100.0) << "%"
                          << ", 样本数=" << samples.size() << std::endl;
                return result;
            }
        }
    }

    // 退出循环（超时或达到 max_samples）后，把当前众数作为最终结果
    if (!samples.empty()) {
        int top_value = 0;
        std::size_t top_count = 0;
        for (const auto& kv : histogram) {
            if (kv.second > top_count) {
                top_count = kv.second;
                top_value = kv.first;
            }
        }
        const double dominance =
            static_cast<double>(top_count) / static_cast<double>(samples.size());
        // 即使样本不足 min_samples，也尽量返回一个结果（valid 用占比是否合理来决定）
        result.valid = (samples.size() >= config_.min_samples);
        result.answer = top_value;
        result.dominance = dominance;
        result.sample_count = samples.size();
        std::cout << "[Calculator] 最终结果: 众数=" << top_value
                  << ", 占比=" << (dominance * 100.0) << "%"
                  << ", 样本数=" << samples.size() << std::endl;
    } else {
        std::cout << "[Calculator] 未采到任何有效样本" << std::endl;
    }

    return result;
}

}  // namespace arithmetic_problem
