#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>

// #include <algorithm>

#include "inference.h"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
 
using namespace std;
using namespace cv;

int main()
{
    rs2::pipeline pipe;
    rs2::config cfg;
 
    cfg.enable_stream(
        RS2_STREAM_COLOR,
        1280,720,
        RS2_FORMAT_BGR8,
        30  //帧率
    );
    pipe.start(cfg);

    BoxIdentify box("/home/yuan/Vscode_word/pnp_box/best.onnx");
    box.pnp_parameter();
 
    while(true) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        cv::Mat color(Size(1280,720), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
 
        char c = box.runBoxIdentify(color);
        Mat cma;
        box.gettvec(cma);

        if(!cma.empty()){
            cout << cma.at<double>(0,0) << "\t"
                << cma.at<double>(1,0) << "\t"
                << cma.at<double>(2,0) << endl;
        }
 
        if(c == 27)    break;
    }
 
    return 0;
}