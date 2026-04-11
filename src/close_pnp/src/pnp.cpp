#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    /* =========================================================
     *  相机内参（来自 ROS2 camera_info）
     * ========================================================= */
    Mat cameraMatrix = (Mat_<double>(3,3) <<
        689.383961, 0.0,        620.412744,
        0.0,        690.604491, 381.697635,
        0.0,        0.0,        1.0);

    Mat distCoeffs = (Mat_<double>(1,5) <<
        0.014295, -0.034779, 0.001624, -0.000775, 0.0);

    /* =========================================================
     *  物体模型参数
     * ========================================================= */
    const float SQUARE_SIZE = 0.35f;   // 正方形边长：350 mm = 0.35 m

    /* =========================================================
     *  0. 读取模板图像
     * ========================================================= */
    Mat templ = imread(
        "/home/yuan/Vscode_word/awork_mycode/box/src/close_pnp/image/1.jpg",
        IMREAD_GRAYSCALE);

    if (templ.empty())
    {
        cout << "模板读取失败" << endl;
        return -1;
    }

    /* =========================================================
     *  1. 打开摄像头
     * ========================================================= */
    VideoCapture cap(2);   // 摄像头编号按需修改
    if (!cap.isOpened())
    {
        cout << "无法打开摄像头" << endl;
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);

    /* =========================================================
     *  2. ORB 初始化（模板特征只计算一次）
     * ========================================================= */
    Ptr<ORB> orb = ORB::create(1000);

    vector<KeyPoint> kp_templ;
    Mat des_templ;
    orb->detectAndCompute(templ, noArray(), kp_templ, des_templ);

    if (des_templ.empty())
    {
        cout << "模板 ORB 特征提取失败" << endl;
        return -1;
    }

    BFMatcher matcher(NORM_HAMMING);

    cout << "开始摄像头检测（ESC 退出）" << endl;

    /* =========================================================
     *  主循环
     * ========================================================= */
    while (true)
    {
        Mat frame, gray;
        cap >> frame;
        if (frame.empty())
            break;

        cvtColor(frame, gray, COLOR_BGR2GRAY);

        /* ----------------- 当前帧 ORB ----------------- */
        vector<KeyPoint> kp_img;
        Mat des_img;
        orb->detectAndCompute(gray, noArray(), kp_img, des_img);

        if (des_img.empty())
        {
            imshow("ORB + PnP Square", frame);
            if (waitKey(1) == 27) break;
            continue;
        }

        /* ----------------- 特征匹配 ----------------- */
        vector<DMatch> matches;
        matcher.match(des_templ, des_img, matches);

        double min_dist = 100.0;
        for (auto &m : matches)
            min_dist = min(min_dist, (double)m.distance);

        vector<DMatch> good_matches;
        for (auto &m : matches)
            if (m.distance < max(2 * min_dist, 30.0))
                good_matches.push_back(m);

        /* ----------------- 单应性 + PnP ----------------- */
        if (good_matches.size() >= 10)
        {
            vector<Point2f> pts_templ, pts_img;
            for (auto &m : good_matches)
            {
                pts_templ.push_back(kp_templ[m.queryIdx].pt);
                pts_img.push_back(kp_img[m.trainIdx].pt);
            }

            Mat H = findHomography(pts_templ, pts_img, RANSAC);

            if (!H.empty())
            {
                /* -------- 模板角点投影 -------- */
                vector<Point2f> templ_corners = {
                    Point2f(0, 0),
                    Point2f((float)templ.cols, 0),
                    Point2f((float)templ.cols, (float)templ.rows),
                    Point2f(0, (float)templ.rows)
                };

                vector<Point2f> img_corners;
                perspectiveTransform(templ_corners, img_corners, H);

                /* -------- 画正方形边框 -------- */
                for (int i = 0; i < 4; i++)
                {
                    line(frame,
                         img_corners[i],
                         img_corners[(i + 1) % 4],
                         Scalar(0, 255, 0), 2);
                }

                /* -------- solvePnP -------- */
                vector<Point3f> object_points = {
                    Point3f(-SQUARE_SIZE/2, -SQUARE_SIZE/2, 0),
                    Point3f( SQUARE_SIZE/2, -SQUARE_SIZE/2, 0),
                    Point3f( SQUARE_SIZE/2,  SQUARE_SIZE/2, 0),
                    Point3f(-SQUARE_SIZE/2,  SQUARE_SIZE/2, 0)
                };

                Mat rvec, tvec;
                bool pnp_ok = solvePnP(
                    object_points,
                    img_corners,
                    cameraMatrix,
                    distCoeffs,
                    rvec,
                    tvec,
                    false,
                    SOLVEPNP_ITERATIVE
                );

                if (pnp_ok)
                {
                    /* -------- 坐标轴可视化 -------- */
                    vector<Point3f> axis = {
                        Point3f(0, 0, 0),
                        Point3f(0.10, 0, 0),
                        Point3f(0, 0.10, 0),
                        Point3f(0, 0, -0.10)
                    };

                    vector<Point2f> img_axis;
                    projectPoints(axis, rvec, tvec,
                                  cameraMatrix, distCoeffs, img_axis);

                    line(frame, img_axis[0], img_axis[1], Scalar(0,0,255), 3);
                    line(frame, img_axis[0], img_axis[2], Scalar(0,255,0), 3);
                    line(frame, img_axis[0], img_axis[3], Scalar(255,0,0), 3);

                    /* -------- 位姿文字 -------- */
                    char text[128];
                    sprintf(text, "X: %.3f  Y: %.3f  Z: %.3f m",
                            tvec.at<double>(0),
                            tvec.at<double>(1),
                            tvec.at<double>(2));

                    putText(frame, text,
                            Point(20, 40),
                            FONT_HERSHEY_SIMPLEX,
                            0.8,
                            Scalar(0, 255, 255),
                            2);
                }
            }
        }

        imshow("ORB + PnP Square", frame);

        if (waitKey(1) == 27)
            break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}