#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include "inference.h"

#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#define MAX_SIZE 100

using namespace std;
using namespace cv;

class Calculate
{
public:
  long long runcalculate(const char *expr,int len) {
      // int len = strlen(expr);

      for (int i = 0; i < len; i++) {
          char c = expr[i];

          if (isdigit(c)) {
              // 1️⃣ 提取完整的多位数
              long long num = 0;
              while (i < len && isdigit(expr[i])) {
                  num = num * 10 + (expr[i] - '0');
                  i++;
              }
              i--;  // 外层 for 还会 i++，这里退回一位
              pushNum(num);

          } else if (c == '(') {
              // 2️⃣ 左括号直接入栈
              pushOp(c);

          } else if (c == ')') {
              // 3️⃣ 右括号：不断计算直到遇到左括号
              while (opTop >= 0 && peekOp() != '(') {
                  calculate();
              }
              popOp();  // 弹出 '('

          } else {
              // 4️⃣ 运算符：栈顶优先级 >= 当前，先算栈顶
              while (opTop >= 0 && peekOp() != '(' && priority(peekOp()) >= priority(c)) {
                  calculate();
              }
              pushOp(c);
          }
      }

      // 5️⃣ 处理栈中剩余的运算符
      while (opTop >= 0) {
          calculate();
      }

      printf("%s = %lld\n", expr, numStack[0]);

      return numStack[0];
  }
private:
  // =============== 数字栈 ===============
  long long numStack[MAX_SIZE];
  int numTop = -1;

  void pushNum(long long val) {
      numStack[++numTop] = val;
  }

  long long popNum() {
      return numStack[numTop--];
  }

  // =============== 运算符栈 ===============
  char opStack[MAX_SIZE];
  int opTop = -1;

  void pushOp(char op) {
      opStack[++opTop] = op;
  }

  char popOp() {
      return opStack[opTop--];
  }

  char peekOp() {
      return opStack[opTop];
  }

  // =============== 优先级定义 ===============
  int priority(char op) {
      if (op == '+' || op == '-') return 1;
      if (op == '*' || op == '/') return 2;
      return 0;  // '(' 优先级最低，不会被意外弹出
  }

  // =============== 取两个数计算 ===============
  void calculate() {
      long long b = popNum();  // 注意：b 先出栈（右操作数）
      long long a = popNum();  // a 后出栈（左操作数）
      char op = popOp();

      long long result;
      switch (op) {
          case '+': result = a + b; break;
          case '-': result = a - b; break;
          case '*': result = a * b; break;
          case '/': result = a / b; break;
          default:  result = 0;
      }
      pushNum(result);
  }

};

class arithmetic
{
public:
  arithmetic(const std::string &onnxModelPath, const cv::Size &modelInputShape = {640, 640}, const std::string &classesTxtFile = "", const bool &runWithCuda = true){
    inf.loadInference(onnxModelPath,modelInputShape,classesTxtFile,runWithCuda);
  }

  void drawimg(const Mat &frame,const vector<Detection> &output)
  {
    Mat picture = frame.clone();

    // 绘制检测结果
    for (const auto& detection : output)
    {
        cv::Rect box = detection.box;

        // 检测框
        cv::rectangle(picture, box, Scalar(255,255,255), 2);

        // 检测类别和置信度
        std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
        cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
        cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

        cv::rectangle(picture, textBox, Scalar(255,255,255), cv::FILLED);
        cv::putText(picture, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2);
    }
    imshow("onnx-.img",picture);
  }
  
  void GetAruthmetic(const Mat &image){
    img = image.clone();
    mask = Mat::zeros(Size(img.cols,img.rows),CV_8UC1);
    vector<Detection> output = inf.runInference(img);

    sort(output.begin(), output.end(),
         [](const Detection &a,const Detection &b){ return a.box.x < b.box.x; });

    // 类别表（确保和模型一致）
    static const std::vector<std::string> classes{
        "0","1","2","3","4","5","6","7","8","9","+","-","*","/","(",")",",","."
    };

    std::string expr;
    expr.reserve(output.size());

    for (auto &det : output) {
        int id = det.class_id;
        if (id < 0 || id >= (int)classes.size()) continue;

        std::string s = classes[id];

        //  ',' 和 '.' 也当作 * 和 /
        if (s == ",") s = "*";
        else if (s == ".") s = "/";

        expr += s;

        std::cout << id << " -> " << s << "\n";
    }

    long long result = cal.runcalculate(expr.c_str(), (int)expr.size());
    std::cout << expr << " = " << result << "\n";

    drawimg(image, output);
    imshow("jpg", image);
  }
private:
  Mat img,gray;
  Mat mask;
  Inference inf;
  Calculate cal;
};

int main(){
  Inference inf("/home/yuan/Vscode_word/arithmetic_problem/best.onnx");
  int n = 452;

  // while(1){
    Mat image;
    image = imread("/home/yuan/Vscode_word/arithmetic_problem/images/" + to_string(n) +".jpg");
    arithmetic lib("/home/yuan/Vscode_word/arithmetic_problem/best.onnx");
    lib.GetAruthmetic(image);

    char c = waitKey(0);
    // if(c == 27)  break;
    // n++;
  // }
  return 0;
}