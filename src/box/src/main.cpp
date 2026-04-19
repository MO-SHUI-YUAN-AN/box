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

    VideoCapture cap(2);
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
        cout << "x " << rvec.at<double>(0,0) << "\t y " << rvec.at<double>(1,0) << "\t z " << rvec.at<double>(2,0) << endl;

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