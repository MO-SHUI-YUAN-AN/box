#include "inference.h"

BoxIdentify::BoxIdentify(const std::string &onnxModelPath, const cv::Size &modelInputShape, const std::string &classesTxtFile, const bool &runWithCuda)
{
    inf.loadInference(onnxModelPath,modelInputShape,classesTxtFile,runWithCuda);

    clahe = cv::createCLAHE(2.0); 
}

char BoxIdentify::runBoxIdentify(cv::Mat camera)
{
    /* ============ 对图片进行处理 ============= */
    cv::Mat drawing = camera.clone();   // 备份图片信息
    if(drawing.empty()) return -1;

    cv::Mat gray;       
    cv::cvtColor(camera, gray, cv::COLOR_BGR2GRAY);   // 转为灰度图
    cv::GaussianBlur(gray,gray,cv::Size(5,5),10,20);  // 高斯滤波

    // CLAHE增强
    clahe->apply(gray, gray);

    // 边缘检测
    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150);

    /* ========= 制作掩码 ========== */
    std::vector<Detection> output = inf.runInference(camera);
    cv::Mat mask = cv::Mat::zeros(cv::Size(camera.cols,camera.rows),CV_8UC1); // 制作掩码;
    for(const auto& detection : output){
            cv::Rect box = detection.box;
            box.x -= 3;
            box.y -= 3;
            box.height += 6;
            box.width += 6;

            cv::rectangle(mask,box,cv::Scalar(255),-1);
    }

    std::vector<cv::Point2f> bestRectPoints;
    checkRect(edges, mask, bestRectPoints);
    
    bool success = false;
        
    // 如果有找到合适的矩形
    if(bestRectPoints.size() == 4) {
        // 对点进行排序
        bestRectPoints = sortRectanglePoints(bestRectPoints);
        
        // 绘制矩形
        for(int i = 0; i < 4; i++) {
            cv::line(drawing, bestRectPoints[i], bestRectPoints[(i + 1) % 4], 
            cv::Scalar(0, 255, 0), 3);
        }
        
        // 绘制角点
        for(int i = 0; i < 4; i++) {
            cv::circle(drawing, bestRectPoints[i], 8, 
                cv::Scalar(0, 0, 255), -1);
                cv::putText(drawing, std::to_string(i), bestRectPoints[i] + cv::Point2f(5, 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
            }
            
        // PnP解算
        success = cv::solvePnP(objectPoints, bestRectPoints, 
            cameraMatrix, distCoeffs, 
            rvec, tvec, false, cv::SOLVEPNP_ITERATIVE
        );
        if(success){
            boolrvec = true;
            booltvec = true;
        }
        drawFrameAxes(drawing,cameraMatrix,distCoeffs,rvec,tvec,0.05);
    }
    cv::imshow(".",drawing);
    cv::imshow("edge",edges);
    cv::imshow("mask",mask);
    char c = cv::waitKey(1);
    return c;
}
    
void BoxIdentify::pnp_parameter(){
    // 相机参数 双目
    cameraMatrix = (cv::Mat_<double>(3,3) << 
    775.0817798329149, 0, 657.9073666547347, 
    0, 775.3491152597893, 363.5442002041569, 
    0, 0, 1);
    
    // 相机参数 双目
    distCoeffs = (cv::Mat_<double>(1,5) << 
    0.08020486644231081, -0.1733432710897198, 
    -0.001440808381498728, -0.0005563035195006499, 
    0.09159837936546486);
    
    objectPoints.clear();
    
    float width = 0.25f;  // 矩形宽度
    float height = 0.25f; // 矩形高度
    objectPoints.push_back(cv::Point3f(-width/2, -height/2, 0));  // 左上
    objectPoints.push_back(cv::Point3f(width/2, -height/2, 0));   // 右上
    objectPoints.push_back(cv::Point3f(width/2, height/2, 0));     // 右下
    objectPoints.push_back(cv::Point3f(-width/2, height/2, 0));   // 左下
}

void BoxIdentify::getrvec(cv::Mat &rvec_copy){
    rvec_copy = rvec.clone();
    boolrvec = false;
}

void BoxIdentify::gettvec(cv::Mat &tvec_copy){
    tvec_copy = tvec.clone();
    booltvec = false;
}

std::vector<cv::Point2f> BoxIdentify::checkRect(cv::Mat &edge,const cv::Mat &mask,std::vector<cv::Point2f> &bestRectPoints){
    // 应用掩码
    bitwise_and(edge, mask, edge);
    
    // 形态学操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(edge, edge, kernel);
    
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;  
    cv::findContours(edge, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    double bestArea = 0;
    
    for(size_t i = 0; i < contours.size(); i++) {
        // 计算轮廓面积
        double area = cv::contourArea(contours[i]);
        if(area < 1000) continue;  // 过滤小轮廓
        
        // 多边形逼近
        std::vector<cv::Point2f> approx;
        double epsilon = 0.02 * cv::arcLength(contours[i], true);
        cv::approxPolyDP(contours[i], approx, epsilon, true);
        
        // 必须是四边形
        if(approx.size() != 4) continue;
        
        // 必须是凸四边形
        if(!cv::isContourConvex(approx)) continue;
        
        // 计算面积比
        double approxArea = cv::contourArea(approx);
        if(approxArea < 1) continue;
        
        double areaRatio = area / approxArea;
        
        // 轮廓面积与多边形面积之比应该接近1
        if(areaRatio < 0.8 || areaRatio > 1.2) continue;
        
        // 检查四边形的最小角度（避免过于尖锐）
        std::vector<float> angles;
        for(int j = 0; j < 4; j++) {
            cv::Point2f a = approx[j];
            cv::Point2f b = approx[(j + 1) % 4];
            cv::Point2f c = approx[(j + 2) % 4];
            
            cv::Point2f v1 = a - b;
            cv::Point2f v2 = c - b;
            
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

// 改进的角点排序：按左上、右上、右下、左下顺序
std::vector<cv::Point2f> BoxIdentify::sortRectanglePoints(std::vector<cv::Point2f>& points) {
    if(points.size() != 4) return points;
    
    std::vector<cv::Point2f> sorted(4);
    
    // 计算中心点
    cv::Point2f center(0, 0);
    for(const auto& p : points) center += p;
    center *= 0.25;
    
    // 按角度排序（逆时针）
    std::sort(points.begin(), points.end(), [center](const cv::Point2f& a, const cv::Point2f& b) {
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
double BoxIdentify::polygonArea(const std::vector<cv::Point2f>& pts) {
    if(pts.size() != 4) return 0;
    
    double area = 0;
    for(int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        area += pts[i].x * pts[j].y - pts[j].x * pts[i].y;
    }
    return abs(area) * 0.5;
}