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
 
class BoxIdentify
{
public:
    BoxIdentify(const std::string &onnxModelPath, const cv::Size &modelInputShape = {640, 640}, const std::string &classesTxtFile = "", const bool &runWithCuda = true)
    {
        inf.loadInference(onnxModelPath,modelInputShape,classesTxtFile,runWithCuda);

        clahe = createCLAHE(2.0); 
    }

    char runBoxIdentify(Mat camera){
        /* ============ 对图片进行处理 ============= */
        Mat drawing = camera.clone();   // 备份图片信息
        if(drawing.empty()) return -1;

        Mat gray;       
        cvtColor(camera, gray, COLOR_BGR2GRAY);   // 转为灰度图
        GaussianBlur(gray,gray,Size(5,5),10,20);  // 高斯滤波

        // CLAHE增强
        clahe->apply(gray, gray);

        // 边缘检测
        Mat edges;
        Canny(gray, edges, 50, 150);

        /* ========= 制作掩码 ========== */
        vector<Detection> output = inf.runInference(camera);
        Mat mask = Mat::zeros(Size(camera.cols,camera.rows),CV_8UC1); // 制作掩码;
        for(const auto& detection : output){
                Rect box = detection.box;
                box.x -= 3;
                box.y -= 3;
                box.height += 6;
                box.width += 6;

                rectangle(mask,box,Scalar(255),-1);
        }

        // 进行计数引用上次成功的掩码（待开发）
        // if(countNonZero(mask) == 0 && n < 10){
        //     n++;
        //     if(!mask_copy.empty())  mask = mask.clone();
        // }
        // else if(countNonZero(mask) > 0){
        //     mask_copy = mask.clone();
        // }
        vector<Point2f> bestRectPoints;
        checkRect(edges, mask, bestRectPoints);
        
        bool success = false;
            
        // 如果有找到合适的矩形
        if(bestRectPoints.size() == 4) {
            // 对点进行排序
            bestRectPoints = sortRectanglePoints(bestRectPoints);
            
            // 绘制矩形
            for(int i = 0; i < 4; i++) {
                line(drawing, bestRectPoints[i], bestRectPoints[(i + 1) % 4], 
                Scalar(0, 255, 0), 3);
            }
            
            // 绘制角点
            for(int i = 0; i < 4; i++) {
                circle(drawing, bestRectPoints[i], 8, 
                    Scalar(0, 0, 255), -1);
                    putText(drawing, to_string(i), bestRectPoints[i] + Point2f(5, 5),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 2);
                }
                
            // PnP解算
            success = solvePnP(objectPoints, bestRectPoints, 
                cameraMatrix, distCoeffs, 
                rvec, tvec, false, SOLVEPNP_ITERATIVE
            );
            if(success){
                boolrvec = true;
                booltvec = true;
            }
            drawFrameAxes(drawing,cameraMatrix,distCoeffs,rvec,tvec,0.05);
        }
        imshow(".",drawing);
        imshow("edge",edges);
        imshow("mask",mask);
        char c = waitKey(1);
        return c;
    }
        
    void pnp_parameter(){
        // 相机参数 双目
        cameraMatrix = (Mat_<double>(3,3) << 
        775.0817798329149, 0, 657.9073666547347, 
        0, 775.3491152597893, 363.5442002041569, 
        0, 0, 1);
        
        // 相机参数 双目
        distCoeffs = (Mat_<double>(1,5) << 
        0.08020486644231081, -0.1733432710897198, 
        -0.001440808381498728, -0.0005563035195006499, 
        0.09159837936546486);
        
        objectPoints.clear();
        
        float width = 25.0f;  // 矩形宽度
        float height = 25.0f; // 矩形高度
        objectPoints.push_back(Point3f(-width/2, -height/2, 0));  // 左上
        objectPoints.push_back(Point3f(width/2, -height/2, 0));   // 右上
        objectPoints.push_back(Point3f(width/2, height/2, 0));     // 右下
        objectPoints.push_back(Point3f(-width/2, height/2, 0));   // 左下
    }
    
    void getrvec(Mat &rvec_copy){
        rvec_copy = rvec.clone();
        boolrvec = false;
    }
    
    void gettvec(Mat &tvec_copy){
        tvec_copy = tvec.clone();
        booltvec = false;
    }
    // 相机参数 双目
    Mat cameraMatrix;
    
    // 相机参数 双目
    Mat distCoeffs;
    
    // 3D点坐标（世界坐标系，单位：cm）
    vector<Point3f> objectPoints;
    
    bool boolrvec = false;
    bool booltvec = false;
private:
    vector<Point2f> checkRect(const Mat &edge,const Mat &mask,vector<Point2f> &bestRectPoints){
        // 应用掩码
        bitwise_and(edge, mask, edge);
        
        // 形态学操作
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        dilate(edge, edge, kernel);
        
        // 查找轮廓
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(edge, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        
        double bestArea = 0;
        
        for(size_t i = 0; i < contours.size(); i++) {
            // 计算轮廓面积
            double area = contourArea(contours[i]);
            if(area < 1000) continue;  // 过滤小轮廓
            
            // 多边形逼近
            vector<Point2f> approx;
            double epsilon = 0.02 * arcLength(contours[i], true);
            approxPolyDP(contours[i], approx, epsilon, true);
            
            // 必须是四边形
            if(approx.size() != 4) continue;
            
            // 必须是凸四边形
            if(!isContourConvex(approx)) continue;
            
            // 计算面积比
            double approxArea = polygonArea(approx);
            if(approxArea < 1) continue;
            
            double areaRatio = area / approxArea;
            
            // 轮廓面积与多边形面积之比应该接近1
            if(areaRatio < 0.8 || areaRatio > 1.2) continue;
            
            // 检查四边形的最小角度（避免过于尖锐）
            vector<float> angles;
            for(int j = 0; j < 4; j++) {
                Point2f a = approx[j];
                Point2f b = approx[(j + 1) % 4];
                Point2f c = approx[(j + 2) % 4];
                
                Point2f v1 = a - b;
                Point2f v2 = c - b;
                
                float dot = v1.x * v2.x + v1.y * v2.y;
                float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
                float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);
                
                if(len1 > 0 && len2 > 0) {
                    float angle = acos(dot / (len1 * len2)) * 180 / CV_PI;
                    angles.push_back(angle);
                }
            }
            
            // 检查角度是否合理
            if(!angles.empty()) {
                float minAngle = *min_element(angles.begin(), angles.end());
                float maxAngle = *max_element(angles.begin(), angles.end());
                if(minAngle < 30 || maxAngle > 150) continue;  // 避免过于尖锐或平缓
            }
            
            // 选择最大的合理矩形
            if(area > bestArea) {
                bestArea = area;
                bestRectPoints = approx; 
            }
        }
        return bestRectPoints;
    }
    // pnp结果
    Mat rvec, tvec;
    
    
    Inference inf;
    
    Ptr<CLAHE> clahe;
    
    // 改进的角点排序：按左上、右上、右下、左下顺序
    vector<Point2f> sortRectanglePoints(vector<Point2f>& points) {
        if(points.size() != 4) return points;
        
        vector<Point2f> sorted(4);
        
        // 计算中心点
        Point2f center(0, 0);
        for(const auto& p : points) center += p;
        center *= 0.25;
        
        // 按角度排序（逆时针）
        sort(points.begin(), points.end(), [center](const Point2f& a, const Point2f& b) {
            return atan2(a.y - center.y, a.x - center.x) < atan2(b.y - center.y, b.x - center.x);
        });
        
        // 找到x+y最小的点（应该是左上角）
        int topLeftIdx = 0;
        float minSum = points[0].x + points[0].y;
        for(int i = 1; i < 4; i++) {
            float sum = points[i].x + points[i].y;
            if(sum < minSum) {
                minSum = sum;
                topLeftIdx = i;
            }
        }
        
        // 重新排列：左上、右上、右下、左下
        for(int i = 0; i < 4; i++) {
            sorted[i] = points[(topLeftIdx + i) % 4];
        }
        
        return sorted;
    }
    
    // 计算四边形的面积
    double polygonArea(const vector<Point2f>& pts) {
        if(pts.size() != 4) return 0;
        
        double area = 0;
        for(int i = 0; i < 4; i++) {
            int j = (i + 1) % 4;
            area += pts[i].x * pts[j].y - pts[j].x * pts[i].y;
        }
        return abs(area) * 0.5;
    }
};

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