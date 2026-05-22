# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于海康威视工业相机和 OpenCV 的算式识别程序。主流程是从相机取帧，使用 ONNX 模型做目标检测，按规则筛出算式字符，拼接并计算结果，再对 4 取模输出答案。

`src/需求.md` 描述了当前功能目标，`src/arithmetic.cpp` 是对应的主入口实现，二者应保持一致。

## 常用命令

### 配置与构建

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

### 运行

```bash
./build/arithmetic
```

运行前需要确保：
- OpenCV 4.x 已安装且 `find_package(OpenCV REQUIRED)` 能找到
- 海康 MVS SDK 的动态库可被加载，通常要把 `include/camera_driver/lib/` 加入 `LD_LIBRARY_PATH`
- `src/arithmetic.cpp` 里的相机配置和 ONNX 模型路径可在当前机器上访问

### 测试

本仓库当前没有单独的自动化测试目标或测试框架配置。若只想验证主程序，直接重新构建后运行 `./build/arithmetic` 即可。

## 代码架构

### 可执行程序

- `src/arithmetic.cpp`：主程序入口，负责取帧、推理、筛选检测框、拼接表达式、计算结果和可视化输出。

### 本地静态库

- `include/camera_driver/`：CMake 子项目，编译为 `camera_driver` 静态库。
  - `src/camera_driver.cpp`：相机封装与取帧逻辑
  - `src/inference.cpp`：OpenCV DNN 的 ONNX 推理封装
  - `src/pnp_kalman.cpp`：PnP + 扩展卡尔曼滤波相关逻辑

### 配置与资源

- `include/camera_driver/camera_init/*.yaml`：相机初始化参数
- `include/camera_driver/lib/`：海康 SDK 动态库和相关运行时文件
- `src/best.onnx`：检测模型

## 需要注意的地方

- `src/arithmetic.cpp` 中存在硬编码绝对路径，迁移机器时通常需要同步修改。
- 当前项目的核心业务逻辑集中在 `src/arithmetic.cpp`，改动表达式解析、筛选规则或结果计算时要同时检查 `src/need.md`。
- TensorRT 相关代码在仓库里是注释状态；如果要启用，需要先补齐依赖和路径配置。
