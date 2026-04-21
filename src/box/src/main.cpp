#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>

// #include <algorithm>
#include <cmath>

#include "inference.h"
#include <opencv2/core/mat.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/executors.hpp>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// #include <robot_robot_interfaces/msg/move_cmd.hpp>
#include <robot_interfaces/msg/move_cmd.hpp>
#include <robot_interfaces/msg/vis.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace cv;
using namespace std;

const int CameraID = 2;
    
/* ================== 话题通信的类 ===================== */
class TargetPosePublisher : public rclcpp::Node
{
public:
TargetPosePublisher() : Node("pnp_move")
    {
        position_pub_ = this->create_publisher<robot_interfaces::msg::Vis>(
            "pnp_move", 10);
    }

    void publishPose()
    {
        if(one){
            sleep(8);
            one = false;
        }

        robot_interfaces::msg::Vis speed;

        // speed.step_mode = step_mode;
        // speed.wheel_vel = 0;
        speed.x = vx;
        speed.y = vy;
        speed.z = vz;

        position_pub_->publish(speed);
    }
    float step_mode = 0,vx = 0,vy = 0,vz = 0;

private:
    bool one = true;
    rclcpp::Publisher<robot_interfaces::msg::Vis>::SharedPtr position_pub_;
};

/**
 * @brief 对 PnP 解算的平移向量 tvec 进行卡尔曼滤波，剔除极端值
 * 
 * 使用恒位置模型（状态 = 3 维平移向量）。每次调用时先预测，计算观测与预测的马氏距离，
 * 若超出阈值则判定为异常，仅用预测值输出；否则执行更新。
 * 
 * @param tvec_obs  输入的平移向量观测值 (3x1, CV_64F 或 CV_32F)
 * @param tvec_filt 输出滤波后的平移向量预测值 (3x1, CV_64F)
 * @param outlierThreshold 马氏距离平方阈值，默认 7.815 对应 3 自由度卡方分布 95% 置信度
 * 
 * @note 第一次调用时用观测值初始化滤波器。函数内部使用 static 变量保存状态，
 *       非线程安全，多线程使用时需外部加锁。
 */
void kalmanFilterTvec(cv::InputArray tvec_obs, cv::OutputArray tvec_filt,
                      double outlierThreshold = 7.815)
{
    // 转换为 double 类型
    cv::Mat tvec = tvec_obs.getMat();
    CV_Assert(tvec.total() == 3);
    tvec.convertTo(tvec, CV_64F);

    // ---------- 静态滤波器状态 ----------
    static bool initialized = false;
    static cv::Mat x(3, 1, CV_64F);       // 状态向量（平移）
    static cv::Mat P(3, 3, CV_64F);       // 状态协方差矩阵

    // 运动模型：恒位置 F = I，观测矩阵 H = I
    static const cv::Mat F = cv::Mat::eye(3, 3, CV_64F);
    static const cv::Mat H = cv::Mat::eye(3, 3, CV_64F);

    // 过程噪声协方差 Q (可根据运动快慢调整)
    static cv::Mat Q = cv::Mat::eye(3, 3, CV_64F) * 1e-2;   // 单位方差 0.01 (单位²)

    // 观测噪声协方差 R (反映 PnP 平移结果的噪声水平)
    static cv::Mat R = cv::Mat::eye(3, 3, CV_64F) * 1e-1;   // 单位方差 0.1

    // ---------- 初始化 ----------
    if (!initialized) {
        tvec.copyTo(x);
        P = cv::Mat::eye(3, 3, CV_64F) * 1e3;  // 初始协方差较大
        initialized = true;

        tvec_filt.create(3, 1, CV_64F);
        tvec.copyTo(tvec_filt);
        return;
    }

    // ---------- 预测 ----------
    cv::Mat x_pred = F * x;
    cv::Mat P_pred = F * P * F.t() + Q;

    // ---------- 异常检测 ----------
    cv::Mat y = tvec - H * x_pred;           // 新息 (3x1)
    cv::Mat S = H * P_pred * H.t() + R;      // 新息协方差 (3x3)
    double mahalanobis = y.dot(S.inv() * y); // 马氏距离平方

    if (mahalanobis <= outlierThreshold) {
        // 观测有效，执行更新
        cv::Mat K = P_pred * H.t() * S.inv(); // 卡尔曼增益 (3x3)
        x = x_pred + K * y;
        cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
        P = (I - K * H) * P_pred;
    } else {
        // 观测异常，仅用预测值
        x = x_pred;
        P = P_pred;
    }

    // ---------- 输出 ----------
    tvec_filt.create(3, 1, CV_64F);
    x.copyTo(tvec_filt);
}

int main(int argc, char **argv)
{
    BoxIdentify work("/home/yuan/Vscode_word/awork_mycode/box/src/box/best.onnx");

    work.pnp_parameter();

    // rs2::pipeline pipe;
    // rs2::config cfg;

    // cfg.enable_stream(
    //     RS2_STREAM_COLOR,
    //     1280,720,
    //     RS2_FORMAT_BGR8,
    //     30  //帧率
    // );

    // pipe.start(cfg);

    VideoCapture cap(CameraID);
    Mat camera;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetPosePublisher>();
    
    node->publishPose();
    node->step_mode = 2;

    while(rclcpp::ok()){
        // rs2::frameset frameset = pipe.wait_for_frames();
        // rs2::frame color = frameset.get_color_frame();

        // if(!color)  continue;


        // Mat camera(
        //     Size(1280,720),
        //     CV_8UC3,
        //     (void*)color.get_data(),
        //     Mat::AUTO_STEP
        // );
        cap >> camera;

        char c = work.runBoxIdentify(camera);
        if(c == 27) break;

        if((!work.boolrvec) || (!work.booltvec))    continue;
        
        Mat rvec,tvec;
        work.getrvec(rvec);
        work.gettvec(tvec);
        
        // kalmanFilterTvec(tvec, tvec);
        cout << "x " << tvec.at<double>(0,0) << "\t y " << tvec.at<double>(1,0) << "\t z " << tvec.at<double>(2,0) << endl;

        node->vx = tvec.at<double>(0);
        node->vy = tvec.at<double>(1);
        node->vz = tvec.at<double>(2);

        // node->vx = 0;
        // node->vy = 0;
        // node->vz = 0;

        // if(tvec.empty()){
        //     node->vz = 1.0; // 如果pnp失败，让狗以最大速度旋转
        // }
        // else if(tvec.at<double>(0,0) > 0.08){
        //     node->vz = -0.4; // 如果物块在右侧，小幅度顺时针旋转
        // }
        // else if(tvec.at<double>(0,0) < -0.08){
        //     node->vz = 0.4; // 如果物块在左侧，小幅度逆时针旋转
        // }
        // else{
        //     if(tvec.at<double>(2,0) > 2){
        //         node->vx = 1.0; // 与目标距离大于两米，让狗以最大速度移动
        //     }
        //     else if(tvec.at<double>(2,0) > 0.35){
        //         node->vx = 1-(2-tvec.at<double>(2,0) / 1.65); // 两米及以内，速度逐步降低
        //     }
        //     else{
        //         node->vx = 0;
        //     }
        // }

        // 向狗发布指令
        node->publishPose();

        rclcpp::spin_some(node);
    }
    
    rclcpp::shutdown();

    return 0;
}