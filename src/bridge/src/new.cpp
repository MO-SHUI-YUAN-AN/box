/**
 * @file new.cpp
 * @brief RealSense深度相机垂直线3D重建与检测
 * 
 * 本程序实现基于Intel RealSense深度相机的垂直线检测与3D空间重建。
 * 主要功能包括：深度边缘检测、霍夫直线检测、3D直线方程计算、
 * 帧间平滑滤波、异常剔除与稳帧判定。
 */

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <deque>


// 
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
// #include <algorithm>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/bridgeb.hpp>

using namespace cv;
using namespace std;

// ============================================================
// 第一部分：全局参数配置
// ============================================================

// ----- 1.1 深度边缘检测参数 -----
const int DEPTH_THRESHOLD   = 15;   // Sobel梯度阈值，越小越敏感(10~60)

// ----- 1.2 霍夫直线检测参数 -----
const int HOUGH_THRESHOLD   = 50;   // 投票阈值，越大要求线越长(40~100)
const int HOUGH_MIN_LENGTH  = 40;   // 最短有效线段长度，像素(30~80)
const int HOUGH_MAX_GAP     = 20;   // 线段断点最大容忍间隙，像素(5~30)

// ----- 1.3 垂直线筛选参数 -----
const int VERTICAL_RATIO    = 2;    // 垂直判断：dy > dx * ratio
const int MAX_VERTICAL_LINES = 1;   // 最多保留的垂直线数量

// ----- 1.4 深度滑动平均滤波参数 -----
const int FILTER_WINDOW_SIZE = 15;  // 滑动窗口大小(帧数)

// ============================================================
// 第二部分：3D直线数据处理参数（核心优化部分）
// ============================================================

// ----- 2.1 帧间低通平滑参数 -----
const int LINE3D_SMOOTH_WINDOW = 5;     // P0和方向向量滑动窗口大小
const float LINE3D_ALPHA = 0.3f;        // 低通滤波系数(0-1)，越小越平滑

// ----- 2.2 三重异常剔除阈值 -----
const float ANGLE_THRESHOLD = 15.0f;    // 角度跳变阈值（度）
const float POSITION_THRESHOLD = 0.1f;  // 位置跳变阈值（米）
const float DEPTH_THRESHOLD_3D = 0.2f;  // 深度跳变阈值（米）

// ----- 2.3 3D竖直线方向硬约束 -----
const float VERTICAL_CONSTRAINT_ANGLE = 10.0f;  // 与Y轴最大允许偏差角度

// ----- 2.4 稳帧判定参数 -----
const int STABLE_FRAME_COUNT = 10;              // 连续稳定帧数判定为稳定
const float STABLE_VARIANCE_THRESHOLD = 0.01f;  // 方差阈值

// ----- 2.5 中线多行均值深度参数 -----
const int CENTER_LINE_ROWS = 7;         // 中心线附近采样行数（奇数）

// ----- 2.6 直线距离筛选参数 -----
const float LINE_MIN_DISTANCE = 0.32f;   // 最小有效距离（米），小于此值舍弃
const float LINE_MAX_DISTANCE = 1.0f;   // 最大有效距离（米），大于此值舍弃

// ============================================================
// 第三部分：数据结构定义
// ============================================================

/**
 * @struct Point3D
 * @brief 3D空间中的点（相机坐标系）
 * 
 * 相机坐标系定义：
 * - X轴：水平向右
 * - Y轴：垂直向下
 * - Z轴：指向场景前方（深度方向）
 */
struct Point3D {
    float x, y, z;
    Point3D(float _x=0, float _y=0, float _z=0) : x(_x), y(_y), z(_z) {}
};

/**
 * @struct Line3D
 * @brief 3D空间中的直线方程: P = P0 + t * direction
 * 
 * 直线表示方法：
 * - point: 直线上一点P0（通常取中心线交点）
 * - direction: 单位方向向量d
 * - valid: 数据有效性标志
 */
struct Line3D {
    Point3D point;      // 直线上一点
    Point3D direction;  // 方向向量（单位向量）
    bool valid = false; // 是否有效
    
    // 归一化方向向量
    void normalize() {
        float len = sqrt(direction.x*direction.x + direction.y*direction.y + direction.z*direction.z);
        if (len > 0) {
            direction.x /= len;
            direction.y /= len;
            direction.z /= len;
        }
    }
    
    // 计算与Y轴的夹角（度），用于竖直约束检查
    float angleWithYAxis() const {
        float dot = abs(direction.y);  // Y轴方向向量为(0,1,0)
        return acos(dot) * 180.0f / CV_PI;
    }
};

// ============================================================
// 第四部分：滤波器类定义
// ============================================================

/**
 * @class DistanceFilter
 * @brief 简单的滑动平均值滤波器（用于深度值）
 * 
 * 工作原理：维护一个滑动窗口，输出窗口内数据的平均值
 * 用途：平滑单帧深度测量值，减少瞬时噪声
 */
class DistanceFilter {
public:
    /**
     * @brief 更新滤波器并获取当前平均值
     * @param newValue 新的深度测量值
     * @return 滤波后的平均深度值
     */
    float update(float newValue) {
        if (newValue <= 0) return getStableValue();
        buffer.push_back(newValue);
        if ((int)buffer.size() > FILTER_WINDOW_SIZE) buffer.erase(buffer.begin());
        return calculateAverage();
    }
    
    float getStableValue() const {
        if (buffer.empty()) return 0.0f;
        return calculateAverageConst();
    }
    
    void reset() { buffer.clear(); }
    
private:
    std::vector<float> buffer;  // 数据缓冲区
    
    float calculateAverage() {
        if (buffer.empty()) return 0.0f;
        float sum = 0;
        for (float v : buffer) sum += v;
        return sum / buffer.size();
    }
    
    float calculateAverageConst() const {
        if (buffer.empty()) return 0.0f;
        float sum = 0;
        for (float v : buffer) sum += v;
        return sum / buffer.size();
    }
};

/**
 * @class Line3DSmoother
 * @brief 3D直线帧间低通平滑滤波器（核心创新点）
 * 
 * 工作原理：
 * 1. 对P0点和方向向量分别进行低通滤波
 * 2. P_smooth = alpha * P_new + (1-alpha) * P_smooth
 * 3. 重新归一化方向向量
 * 4. 维护历史记录用于稳帧判定
 */
class Line3DSmoother {
public:
    /**
     * @brief 更新3D直线平滑值
     * @param newLine 新计算出的3D直线
     * @return 平滑后的3D直线
     */
    Line3D update(const Line3D& newLine) {
        if (!newLine.valid) return getSmoothedLine();
        
        // 初始化：第一帧直接赋值
        if (!initialized) {
            smoothedPoint = newLine.point;
            smoothedDirection = newLine.direction;
            initialized = true;
            return newLine;
        }
        
        // 低通滤波：加权平均
        smoothedPoint.x = LINE3D_ALPHA * newLine.point.x + (1.0f - LINE3D_ALPHA) * smoothedPoint.x;
        smoothedPoint.y = LINE3D_ALPHA * newLine.point.y + (1.0f - LINE3D_ALPHA) * smoothedPoint.y;
        smoothedPoint.z = LINE3D_ALPHA * newLine.point.z + (1.0f - LINE3D_ALPHA) * smoothedPoint.z;
        
        smoothedDirection.x = LINE3D_ALPHA * newLine.direction.x + (1.0f - LINE3D_ALPHA) * smoothedDirection.x;
        smoothedDirection.y = LINE3D_ALPHA * newLine.direction.y + (1.0f - LINE3D_ALPHA) * smoothedDirection.y;
        smoothedDirection.z = LINE3D_ALPHA * newLine.direction.z + (1.0f - LINE3D_ALPHA) * smoothedDirection.z;
        
        // 重新归一化方向向量（保持单位向量特性）
        float len = sqrt(smoothedDirection.x*smoothedDirection.x + 
                        smoothedDirection.y*smoothedDirection.y + 
                        smoothedDirection.z*smoothedDirection.z);
        if (len > 0) {
            smoothedDirection.x /= len;
            smoothedDirection.y /= len;
            smoothedDirection.z /= len;
        }
        
        // 存入历史记录用于稳帧判定
        pointHistory.push_back(smoothedPoint);
        if (pointHistory.size() > STABLE_FRAME_COUNT) pointHistory.pop_front();
        
        return getSmoothedLine();
    }
    
    Line3D getSmoothedLine() const {
        Line3D line;
        line.point = smoothedPoint;
        line.direction = smoothedDirection;
        line.valid = initialized;
        return line;
    }
    
    /**
     * @brief 稳帧判定：检查历史数据的方差是否小于阈值
     * @return true表示已稳定，false表示仍在收敛
     */
    bool isStable() const {
        if (pointHistory.size() < STABLE_FRAME_COUNT) return false;
        
        // 计算位置方差
        float meanX = 0, meanY = 0, meanZ = 0;
        for (const auto& p : pointHistory) {
            meanX += p.x; meanY += p.y; meanZ += p.z;
        }
        meanX /= pointHistory.size();
        meanY /= pointHistory.size();
        meanZ /= pointHistory.size();
        
        float varX = 0, varY = 0, varZ = 0;
        for (const auto& p : pointHistory) {
            varX += (p.x - meanX) * (p.x - meanX);
            varY += (p.y - meanY) * (p.y - meanY);
            varZ += (p.z - meanZ) * (p.z - meanZ);
        }
        varX /= pointHistory.size();
        varY /= pointHistory.size();
        varZ /= pointHistory.size();
        
        float totalVariance = varX + varY + varZ;
        return totalVariance < STABLE_VARIANCE_THRESHOLD;
    }
    
    void reset() {
        initialized = false;
        pointHistory.clear();
    }
    
private:
    Point3D smoothedPoint;           // 平滑后的P0点
    Point3D smoothedDirection;       // 平滑后的方向向量
    bool initialized = false;        // 是否已初始化
    deque<Point3D> pointHistory;     // 历史记录（用于稳帧判定）
};

// ============================================================
// 第五部分：基础图像处理函数
// ============================================================

/**
 * @brief 深度边缘检测（Sobel算子）
 * @param depthMat 深度图（16位）
 * @param depthScale 深度比例（米/单位）
 * @return 边缘图（8位二值图）
 * 
 * 处理流程：深度图→Sobel梯度计算→归一化→二值化→形态学闭运算
 */
Mat detectDepthEdges(const Mat& depthMat, float depthScale) {
    Mat depthFloat;
    depthMat.convertTo(depthFloat, CV_32F, depthScale);
    Mat gradX, gradY, gradMag;
    Sobel(depthFloat, gradX, CV_32F, 1, 0, 3);
    Sobel(depthFloat, gradY, CV_32F, 0, 1, 3);
    magnitude(gradX, gradY, gradMag);
    Mat edges;
    normalize(gradMag, edges, 0, 255, NORM_MINMAX);
    edges.convertTo(edges, CV_8U);
    threshold(edges, edges, DEPTH_THRESHOLD, 255, THRESH_BINARY);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(edges, edges, MORPH_CLOSE, kernel);
    return edges;
}

/**
 * @brief 计算直线与图像中心水平线的交点X坐标
 * @param line 2D线段(u1,v1,u2,v2)
 * @param imageHeight 图像高度
 * @return 交点的X坐标
 * 
 * 原理：线性插值计算直线在y=height/2处的x坐标
 */
int getCenterIntersectionX(const Vec4i& line, int imageHeight) {
    int x1 = line[0], y1 = line[1];
    int x2 = line[2], y2 = line[3];
    if (abs(y2 - y1) < 1) return (x1 + x2) / 2;
    int centerY = imageHeight / 2;
    float t = float(centerY - y1) / float(y2 - y1);
    return int(x1 + t * (x2 - x1));
}

/**
 * @brief 获取直线与中心线交点的深度值
 * @param line 2D线段
 * @param depth 深度帧
 * @param imageHeight 图像高度
 * @param imageWidth 图像宽度
 * @return 深度值（米），无效返回0
 */
float getLineDepth(const Vec4i& line, const rs2::depth_frame& depth, int imageHeight, int imageWidth = 640) {
    int intersectX = getCenterIntersectionX(line, imageHeight);
    int centerY = imageHeight / 2;
    intersectX = max(0, min(intersectX, imageWidth - 1));
    centerY = max(0, min(centerY, imageHeight - 1));
    return depth.get_distance(intersectX, centerY);
}

/**
 * @brief 安全的深度获取（带边界检查）
 */
float getDepthSafe(const rs2::depth_frame& depth, int x, int y, int width, int height) {
    if (x < 0 || x >= width || y < 0 || y >= height) return 0.0f;
    return depth.get_distance(x, y);
}

/**
 * @brief 将图像坐标反投影为相机坐标系下的3D点
 * @param u,v 图像坐标（像素）
 * @param depth 深度值（米）
 * @param fx,fy 相机焦距
 * @param cx,cy 相机主点
 * @return 3D点（相机坐标系）
 * 
 * 公式：X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy, Z = depth
 */
Point3D deprojectPixelToPoint(int u, int v, float depth, float fx, float fy, float cx, float cy) {
    if (depth <= 0) return Point3D(0, 0, 0);
    float X = (u - cx) * depth / fx;
    float Y = (v - cy) * depth / fy;
    float Z = depth;
    return Point3D(X, Y, Z);
}

// ============================================================
// 第六部分：3D直线计算核心函数
// ============================================================

/**
 * @brief 中线多行均值深度解算3D点（高精度方法）
 * @param line 2D线段
 * @param depth 深度帧
 * @param fx,fy,cx,cy 相机内参
 * @param imageWidth,imageHeight 图像尺寸
 * @return 3D点（中心线附近多行采样的平均值）
 * 
 * 创新点：不只用单点深度，而是在中心线附近多行采样取平均，提高精度
 */
Point3D computeCenterLinePoint3D(const Vec4i& line, const rs2::depth_frame& depth,
                                  float fx, float fy, float cx, float cy,
                                  int imageWidth, int imageHeight) {
    int centerY = imageHeight / 2;
    int halfRows = CENTER_LINE_ROWS / 2;
    
    float sumX = 0, sumY = 0, sumZ = 0;
    int validCount = 0;
    
    // 在中心线附近多行采样
    for (int offset = -halfRows; offset <= halfRows; offset++) {
        int sampleY = centerY + offset;
        if (sampleY < 0 || sampleY >= imageHeight) continue;
        
        // 计算该y坐标对应的x坐标（延长直线）
        int x1 = line[0], y1 = line[1];
        int x2 = line[2], y2 = line[3];
        
        if (abs(y2 - y1) < 1) continue;
        
        float t = float(sampleY - y1) / float(y2 - y1);
        int sampleX = int(x1 + t * (x2 - x1));
        sampleX = max(0, min(sampleX, imageWidth - 1));
        
        // 获取深度并反投影
        float d = getDepthSafe(depth, sampleX, sampleY, imageWidth, imageHeight);
        if (d > 0) {
            Point3D p = deprojectPixelToPoint(sampleX, sampleY, d, fx, fy, cx, cy);
            if (p.z > 0) {
                sumX += p.x; sumY += p.y; sumZ += p.z;
                validCount++;
            }
        }
    }
    
    if (validCount == 0) return Point3D(0, 0, 0);
    return Point3D(sumX / validCount, sumY / validCount, sumZ / validCount);
}

/**
 * @brief 优化后的3D直线计算（含竖直约束和异常剔除）
 * @param line 2D线段
 * @param depth 深度帧
 * @param fx,fy,cx,cy 相机内参
 * @param imageWidth,imageHeight 图像尺寸
 * @param prevLine 上一帧的3D直线（用于异常剔除）
 * @return 3D直线方程
 * 
 * 核心功能：
 * 1. 中线多行均值计算P0点
 * 2. 3D竖直方向硬约束（强制接近Y轴）
 * 3. 三重异常剔除（角度/位置/深度跳变）
 */
Line3D computeLine3DEquationOptimized(const Vec4i& line, const rs2::depth_frame& depth,
                                       float fx, float fy, float cx, float cy,
                                       int imageWidth, int imageHeight,
                                       const Line3D& prevLine) {
    Line3D line3D;
    
    int u1 = line[0], v1 = line[1];
    int u2 = line[2], v2 = line[3];
    
    // 边界检查
    u1 = max(0, min(u1, imageWidth - 1));
    v1 = max(0, min(v1, imageHeight - 1));
    u2 = max(0, min(u2, imageWidth - 1));
    v2 = max(0, min(v2, imageHeight - 1));
    
    // 步骤1：使用中线多行均值深度计算3D点（高精度）
    Point3D p1 = computeCenterLinePoint3D(line, depth, fx, fy, cx, cy, imageWidth, imageHeight);
    
    // 如果中线点计算失败，使用端点作为备选
    if (p1.z <= 0) {
        float d1 = getDepthSafe(depth, u1, v1, imageWidth, imageHeight);
        if (d1 > 0) p1 = deprojectPixelToPoint(u1, v1, d1, fx, fy, cx, cy);
    }
    
    // 步骤2：计算第二个点用于确定方向
    Point3D p2;
    float d2 = getDepthSafe(depth, u2, v2, imageWidth, imageHeight);
    if (d2 > 0) {
        p2 = deprojectPixelToPoint(u2, v2, d2, fx, fy, cx, cy);
    } else {
        // 如果端点深度无效，使用固定偏移采样
        int offsetY = (v2 > v1) ? 20 : -20;
        int sampleY = v1 + offsetY;
        sampleY = max(0, min(sampleY, imageHeight - 1));
        float t = float(sampleY - v1) / float(v2 - v1 + 0.001f);
        int sampleX = int(u1 + t * (u2 - u1));
        sampleX = max(0, min(sampleX, imageWidth - 1));
        float d = getDepthSafe(depth, sampleX, sampleY, imageWidth, imageHeight);
        if (d > 0) p2 = deprojectPixelToPoint(sampleX, sampleY, d, fx, fy, cx, cy);
    }
    
    if (p1.z <= 0 || p2.z <= 0) return line3D;  // 无效数据
    
    // 步骤3：计算方向向量并归一化
    line3D.point = p1;
    line3D.direction = Point3D(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    line3D.normalize();
    
    // 步骤4：3D竖直线方向硬约束
    float angleWithY = line3D.angleWithYAxis();
    if (angleWithY > VERTICAL_CONSTRAINT_ANGLE) {
        // 如果偏离Y轴太大，向Y轴方向修正
        float correctionFactor = VERTICAL_CONSTRAINT_ANGLE / angleWithY;
        line3D.direction.x *= correctionFactor;
        line3D.direction.z *= correctionFactor;
        line3D.direction.y = (line3D.direction.y > 0 ? 1.0f : -1.0f) * 
                             sqrt(1.0f - line3D.direction.x*line3D.direction.x 
                                        - line3D.direction.z*line3D.direction.z);
    }
    
    // 步骤5：三重异常剔除检查
    if (prevLine.valid) {
        // 5.1 角度跳变检查
        float angleDiff = acos(min(1.0f, abs(line3D.direction.x * prevLine.direction.x +
                                              line3D.direction.y * prevLine.direction.y +
                                              line3D.direction.z * prevLine.direction.z))) * 180.0f / CV_PI;
        if (angleDiff > ANGLE_THRESHOLD) {
            return prevLine;  // 角度跳变过大，返回上一帧结果
        }
        
        // 5.2 位置跳变检查
        float posDiff = sqrt((line3D.point.x - prevLine.point.x) * (line3D.point.x - prevLine.point.x) +
                             (line3D.point.y - prevLine.point.y) * (line3D.point.y - prevLine.point.y) +
                             (line3D.point.z - prevLine.point.z) * (line3D.point.z - prevLine.point.z));
        if (posDiff > POSITION_THRESHOLD) {
            return prevLine;  // 位置跳变过大，返回上一帧结果
        }
        
        // 5.3 深度跳变检查
        if (abs(line3D.point.z - prevLine.point.z) > DEPTH_THRESHOLD_3D) {
            return prevLine;  // 深度跳变过大，返回上一帧结果
        }
    }
    
    line3D.valid = true;
    return line3D;
}

// ============================================================
// 第七部分：直线筛选与聚类函数
// ============================================================

/**
 * @brief 聚类合并相近的垂直线
 * @param lines 所有检测到的直线
 * @param mergeThresh 合并阈值（像素）
 * @return 合并后的垂直线列表
 */
vector<Vec4i> mergeCloseVerticalLines(const vector<Vec4i>& lines, int mergeThresh = 20) {
    vector<Vec4i> verticalLines;
    
    // 步骤1：筛选垂直线（dy > dx * ratio）
    for (const auto& line : lines) {
        int dx = abs(line[2] - line[0]);
        int dy = abs(line[3] - line[1]);
        if (dy > dx * VERTICAL_RATIO)
            verticalLines.push_back(line);
    }
    
    if (verticalLines.empty()) return {};
    
    // 步骤2：按X坐标排序
    sort(verticalLines.begin(), verticalLines.end(),
         [](const Vec4i& a, const Vec4i& b) {
             return (a[0] + a[2]) < (b[0] + b[2]);
         });
    
    // 步骤3：聚类合并（X距离小于阈值的合并为一条）
    vector<Vec4i> merged;
    vector<Vec4i> cluster;
    cluster.push_back(verticalLines[0]);
    
    for (size_t i = 1; i < verticalLines.size(); i++) {
        int xCurr = (verticalLines[i][0]  + verticalLines[i][2])  / 2;
        int xPrev = (cluster.back()[0] + cluster.back()[2]) / 2;
        
        if (abs(xCurr - xPrev) <= mergeThresh) {
            cluster.push_back(verticalLines[i]);
        } else {
            // 取cluster中最长的线
            Vec4i best = *max_element(cluster.begin(), cluster.end(),
                [](const Vec4i& a, const Vec4i& b) {
                    return abs(a[3]-a[1]) < abs(b[3]-b[1]);
                });
            merged.push_back(best);
            cluster.clear();
            cluster.push_back(verticalLines[i]);
        }
    }
    
    // 处理最后一个cluster
    Vec4i best = *max_element(cluster.begin(), cluster.end(),
        [](const Vec4i& a, const Vec4i& b) {
            return abs(a[3]-a[1]) < abs(b[3]-b[1]);
        });
    merged.push_back(best);
    
    // 限制最大数量
    if ((int)merged.size() > MAX_VERTICAL_LINES)
        merged.resize(MAX_VERTICAL_LINES);
    
    return merged;
}

/**
 * @brief 按距离筛选直线（只保留在有效距离范围内的）
 * @param verticalLines 垂直线列表
 * @param depth 深度帧
 * @param imageHeight 图像高度
 * @return 筛选后的直线列表
 */
vector<Vec4i> filterLinesByDistance(const vector<Vec4i>& verticalLines, 
                                     const rs2::depth_frame& depth, 
                                     int imageHeight) {
    vector<Vec4i> filteredLines;
    for (const auto& line : verticalLines) {
        float d = getLineDepth(line, depth, imageHeight);
        // 只保留距离在[LINE_MIN_DISTANCE, LINE_MAX_DISTANCE]范围内的直线
        if (d >= LINE_MIN_DISTANCE && d <= LINE_MAX_DISTANCE) {
            filteredLines.push_back(line);
        }
    }
    return filteredLines;
}

/**
 * @brief 找距离相机最近的直线
 * @param verticalLines 垂直线列表
 * @param depth 深度帧
 * @param imageHeight 图像高度
 * @return 最近直线的索引，无效返回-1
 */
int findNearestLine(const vector<Vec4i>& verticalLines, const rs2::depth_frame& depth, int imageHeight) {
    if (verticalLines.empty()) return -1;
    
    int nearestIdx = 0;
    float minDepth = FLT_MAX;
    
    for (size_t i = 0; i < verticalLines.size(); i++) {
        float d = getLineDepth(verticalLines[i], depth, imageHeight);
        if (d > 0 && d < minDepth) {
            minDepth = d;
            nearestIdx = i;
        }
    }
    
    return minDepth > 0 ? nearestIdx : -1;
}

/* ================== 2200话题通信的类 ===================== */
class TargetPosePublisher : public rclcpp::Node {
public:
    TargetPosePublisher() : Node("target_pose_publisher") {
        pub_ = this->create_publisher<robot_interfaces::msg::Bridgeb>("bridgeb", 10);
    }

    void publishPose(const Line3D& line3d, const int &x) {
        if (!line3d.valid) {
            return;
        }
        robot_interfaces::msg::Bridgeb msg;
        msg.p.x = line3d.point.x;
        msg.p.y = line3d.point.y;
        msg.p.z = line3d.point.z;

        msg.v.x = line3d.direction.x;
        msg.v.y = line3d.direction.y;
        msg.v.z = line3d.direction.z;

        msg.c = x; // 表示数据有效

        pub_->publish(msg); // 发布话题
    }

private:
    rclcpp::Publisher<robot_interfaces::msg::Bridgeb>::SharedPtr pub_;
};


// 提取连通域
cv::Mat extractConnectedComponents(const cv::Mat& edgeImg)
{
    cv::Mat labels, stats, centroids;

    // 连通域分析
    int num = cv::connectedComponentsWithStats(
        edgeImg,      // 输入二值图
        labels,       // 输出标签图
        stats,        // 连通域信息
        centroids,    // 中心点
        8,            // 8连通
        CV_32S
    );

    // 创建输出图
    cv::Mat result = cv::Mat::zeros(edgeImg.size(), CV_8U);

    // 遍历所有连通域
    for (int i = 1; i < num; i++)  // 从1开始，0是背景
    {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);

        // 面积阈值（去掉小噪点）
        if (area > 100)
        {
            result.setTo(255, labels == i);
        }
    }
    morphologyEx(result, result, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(11, 11)));

    return result;
}


// ============================================================
// 第八部分：主函数
// ============================================================

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);

        // ----- 8.1 初始化RealSense相机 -----
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
        rs2::pipeline_profile profile = pipe.start(cfg);
        
        float depthScale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
        auto intrinsics = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        
        cout << "深度相机已启动" << endl;
        cout << "深度比例: " << depthScale << " 米/单位" << endl;
        cout << "相机内参 - fx: " << intrinsics.fx << ", fy: " << intrinsics.fy << endl;
        cout << "按 'q' 退出程序" << endl;

        rs2::pointcloud pc;
        rs2::points points;
        
        // ----- 8.2 初始化滤波器 -----
        DistanceFilter depthFilter;           // 深度滑动平均滤波器
        Line3DSmoother line3DSmoother;        // 3D直线帧间平滑器
        Line3D prevLine3D;                    // 上一帧的3D直线
        bool hasPrintedStable = false;        // 是否已输出稳定结果

        // 话题类
        auto node = std::make_shared<TargetPosePublisher>();

        // ----- 8.3 主循环 -----
        while (rclcpp::ok()) {
            // 获取深度帧
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::depth_frame depth = frames.get_depth_frame();
            points = pc.calculate(depth);

            Mat depthImage(Size(640, 480), CV_16UC1, (void*)depth.get_data());
            
            // 步骤1：深度边缘检测
            Mat depthEdges = detectDepthEdges(depthImage, depthScale);
            // 均值滤波
            // blur(depthEdges, depthEdges, Size(3,  3));
            // 中值滤波
            // medianBlur(depthEdges, depthEdges, 3);
            // 高斯滤波
            GaussianBlur(depthEdges, depthEdges, Size(3, 3), 10, 20);
            // 膨胀
            // dilate(depthEdges, depthEdges, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
            // 开闭运算
            // morphologyEx(depthEdges, depthEdges, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3, 3)));
            morphologyEx(depthEdges, depthEdges, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(7, 7)));
            // 提取连通域
            depthEdges = extractConnectedComponents(depthEdges);

            // 步骤2：霍夫直线检测
            vector<Vec4i> lines;
            HoughLinesP(depthEdges, lines, 1, CV_PI / 180,
                        HOUGH_THRESHOLD, HOUGH_MIN_LENGTH, HOUGH_MAX_GAP);
            
            // 步骤3：筛选垂直线并聚类合并
            vector<Vec4i> verticalLines = mergeCloseVerticalLines(lines);

            const int IMG_WIDTH = 640;
            const int IMG_HEIGHT = 480;
            const int CENTER_Y = IMG_HEIGHT / 2;

            // 步骤4：距离筛选（只保留0.3m-1.0m范围内的直线）
            vector<Vec4i> filteredLines = filterLinesByDistance(verticalLines, depth, IMG_HEIGHT);

            // ----- 8.4 可视化准备 -----
            Mat depthColorMap;
            depthImage.convertTo(depthColorMap, CV_8UC1, 255.0 / 10000);
            applyColorMap(depthColorMap, depthColorMap, COLORMAP_JET);
            Mat result = depthColorMap.clone();
            Mat edgeVis;
            cvtColor(depthEdges, edgeVis, COLOR_GRAY2BGR);

            // 步骤5：找距离最近的直线
            int nearestIdx = findNearestLine(filteredLines, depth, IMG_HEIGHT);
            float rawDepth = (nearestIdx >= 0) ? getLineDepth(filteredLines[nearestIdx], depth, IMG_HEIGHT) : 0;
            float filteredDepth = depthFilter.update(rawDepth);

            // ----- 8.5 绘制可视化 -----
            // 灰色：所有检测到的垂直线
            for (const auto& vl : verticalLines) {
                cv::line(result,  Point(vl[0], vl[1]), Point(vl[2], vl[3]), Scalar(128, 128, 128), 1);
                cv::line(edgeVis, Point(vl[0], vl[1]), Point(vl[2], vl[3]), Scalar(128, 128, 128), 1);
            }
            
            // 绿色/红色：通过距离筛选的直线（最近为红色）
            for (size_t i = 0; i < filteredLines.size(); i++) {
                const auto& vl = filteredLines[i];
                bool isNearest = ((int)i == nearestIdx);
                Scalar c  = isNearest ? Scalar(0, 0, 255) : Scalar(0, 255, 0);
                int   th  = isNearest ? 4 : 2;
                cv::line(result,  Point(vl[0], vl[1]), Point(vl[2], vl[3]), c, th);
                cv::line(edgeVis, Point(vl[0], vl[1]), Point(vl[2], vl[3]), c, th);
            }
            
            // 黄色：相机中心水平线
            cv::line(result, Point(0, CENTER_Y), Point(IMG_WIDTH, CENTER_Y), Scalar(0, 255, 255), 1, LINE_AA);
            cv::line(edgeVis, Point(0, CENTER_Y), Point(IMG_WIDTH, CENTER_Y), Scalar(0, 255, 255), 1, LINE_AA);

            // ----- 8.6 3D直线计算与输出 -----
            if (nearestIdx >= 0 && filteredDepth > 0) {
                Vec4i& nearestLine = verticalLines[nearestIdx];
                
                // 计算与中心线交点并绘制（蓝色圆点）
                int intersectX = getCenterIntersectionX(nearestLine, IMG_HEIGHT);
                Point intersectPoint(intersectX, CENTER_Y);
                circle(result, intersectPoint, 6, Scalar(255, 0, 0), -1);
                circle(edgeVis, intersectPoint, 6, Scalar(255, 0, 0), -1);

                // 步骤6：计算3D直线方程（含竖直约束和异常剔除）
                Line3D rawLine3D = computeLine3DEquationOptimized(nearestLine, depth,
                                                                   intrinsics.fx, intrinsics.fy,
                                                                   intrinsics.ppx, intrinsics.ppy,
                                                                   IMG_WIDTH, IMG_HEIGHT, prevLine3D);
                
                // 步骤7：帧间低通平滑
                Line3D smoothedLine3D = line3DSmoother.update(rawLine3D);
                prevLine3D = smoothedLine3D;
                node->publishPose(smoothedLine3D,1);  // 发布话题
  

                // 显示距离信息
                char distText[64];
                sprintf(distText, "Avg: %.3f m", filteredDepth);
                putText(result, distText, Point(20, 40), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);
                putText(edgeVis, distText, Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
                
                char rawText[64];
                sprintf(rawText, "Raw: %.3f m", rawDepth);
                putText(result, rawText, Point(20, 70), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
                
                char intersectText[64];
                sprintf(intersectText, "Intersect: (%d, %d)", intersectX, CENTER_Y);
                putText(result, intersectText, Point(20, 130), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 1);

                char posText[64];
                sprintf(posText, "Line: (%d,%d) - (%d,%d)",
                        nearestLine[0], nearestLine[1], nearestLine[2], nearestLine[3]);
                putText(result, posText, Point(20, 100), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
                
                // 步骤8：稳帧判定与输出
                if (line3DSmoother.isStable() && smoothedLine3D.valid) {
                    if (!hasPrintedStable) {
                        printf("\n========== 3D直线方程已稳定 ==========\n");
                        hasPrintedStable = true;
                    }
                    printf("\n[稳定帧] 3D直线方程:\n");
                    printf("  P0: (%.4f, %.4f, %.4f) m\n", 
                           smoothedLine3D.point.x, smoothedLine3D.point.y, smoothedLine3D.point.z);
                    printf("  d:  (%.4f, %.4f, %.4f)\n", 
                           smoothedLine3D.direction.x, smoothedLine3D.direction.y, smoothedLine3D.direction.z);
                    printf("  与Y轴夹角: %.2f°\n", smoothedLine3D.angleWithYAxis());
                    printf("================================\n");
                } else if (smoothedLine3D.valid) {
                    printf("[收敛中...]\n");
                }
            } else {
                putText(result, "No valid line detected", Point(20, 40),
                        FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 165, 255), 2);
                hasPrintedStable = false;  // 检测丢失时重置稳帧状态
            }

            // 显示直线统计信息
            char lineCountText[128];
            sprintf(lineCountText, "Lines: %zu | Valid: %zu | Range: %.1f-%.1fm",
                    verticalLines.size(), filteredLines.size(),
                    LINE_MIN_DISTANCE, LINE_MAX_DISTANCE);
            putText(result, lineCountText, Point(20, result.rows - 40),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);

            // 显示窗口
            imshow("深度边缘图", edgeVis);
            imshow("深度图", depthColorMap);
            imshow("间隙检测结果", result);
            // imshow("连通域", stars);

            char key = waitKey(1);
            if (key == 'q' || key == 27) break;
        }

        pipe.stop();
    } catch (const rs2::error& e) {
        cerr << "RealSense错误: " << e.what() << endl;
        return EXIT_FAILURE;
    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}