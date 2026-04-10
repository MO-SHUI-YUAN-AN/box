#ifndef INFERENCE_H
#define INFERENCE_H

// Cpp native
#include <fstream>
#include <vector>
#include <string>
#include <random>

// OpenCV / DNN / Inference
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

struct Detection
{
    int class_id{0};
    std::string className{};
    float confidence{0.0};
    cv::Scalar color{};
    cv::Rect box{};
};

class Inference
{
public:
    Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape = {640, 640}, const std::string &classesTxtFile = "", const bool &runWithCuda = true);

    std::vector<Detection> runInference(const cv::Mat &input);

    Inference();

    void loadInference(const std::string &onnxModelPath, const cv::Size &modelInputShape = {640, 640}, const std::string &classesTxtFile = "", const bool &runWithCuda = true);

private:
    void loadClassesFromFile();
    void loadOnnxNetwork();
    cv::Mat formatToSquare(const cv::Mat &source);

    std::string modelPath{};
    std::string classesPath{};
    bool cudaEnabled{};

 //   std::vector<std::string> classes{"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
    std::vector<std::string> classes{"instrument","pill","tool","food"};
    cv::Size2f modelShape{};

    float modelConfidenceThreshold {0.25};
    float modelScoreThreshold      {0.45};
    float modelNMSThreshold        {0.50};

    bool letterBoxForSquare = true;

    cv::dnn::Net net;
};

/* ================== pnp的类 ===================== */
class BoxIdentify
{
public:
    BoxIdentify(
        const std::string &onnxModelPath, 
        const cv::Size &modelInputShape = {640, 640}, 
        const std::string &classesTxtFile = "", const bool &runWithCuda = true
    );

    char runBoxIdentify(cv::Mat camera);
        
    void pnp_parameter();
    
    void getrvec(cv::Mat &rvec_copy);
    
    void gettvec(cv::Mat &tvec_copy);
    // 相机参数 双目
    cv::Mat cameraMatrix;
    
    // 相机参数 双目
    cv::Mat distCoeffs;
    
    // 3D点坐标（世界坐标系，单位：米）
    std::vector<cv::Point3f> objectPoints;
    
    bool boolrvec = false;
    bool booltvec = false;
private:
    std::vector<cv::Point2f> checkRect(cv::Mat &edge,const cv::Mat &mask,std::vector<cv::Point2f> &bestRectPoints);
    // pnp结果
    cv::Mat rvec, tvec;

    Inference inf;
    
    cv::Ptr<cv::CLAHE> clahe;
    
    // 改进的角点排序：按左上、右上、右下、左下顺序
    std::vector<cv::Point2f> sortRectanglePoints(std::vector<cv::Point2f>& points);
    
    // 计算四边形的面积
    double polygonArea(const std::vector<cv::Point2f>& pts);
};

#endif // INFERENCE_H
