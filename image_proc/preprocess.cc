/*!
 *  \file     preprocess.cc
 *  \brief    图像处理函数
 *  \author   lancer wong
 *  \version  1.0
 *  \date     2018/03/06
 *
 *  Change History
 *  --
 *  |    Date     |   Version   |    Author   | Description |
 *  |-------------|-------------|-------------|-------------|
 *  | 2018/03/06  |     1.0     | lancer wong | Create      |
 *
 */

#include "preprocess.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream> //< for debug

using std::vector;
using cv::Mat;
using cv::Size;
using cv::Point;
using cv::RotatedRect;
using cv::Point2f;
using cv::Scalar;
using cv::RNG;

/*!
 *  \brief    图像预处理
 *
 *  考虑到实际图像分辨率高且大部分为黑色背景区域,通过简单的阈值分割提取出
 *  前景图像
 *  \param    [in]  src    待处理的原图像
 *  \param    [in]  thresh    去除黑色背景区域的阈值,可以取小一点
 *  \return   处理后的图像
 *  \retval   Mat
 */
Mat
ImagePreprocess(const Mat& src, double thresh) {
  Mat gauss_image;
  Mat thresh_image;
  Mat morph_image;
  Mat edges;
  Mat element = getStructuringElement(cv::MORPH_RECT, Size(15, 15));

  GaussianBlur(src, gauss_image, Size(3, 3), 0);
  threshold(gauss_image, thresh_image, thresh, 255, CV_THRESH_BINARY);
  /* 图像预处理阶段,先膨胀后腐蚀去除区域内的孤立区域,使轮廓连续光滑. */
  morphologyEx(thresh_image, morph_image, CV_MOP_CLOSE, element);

  return morph_image;
}

/*!
 *  \brief    图像的进一步处理
 *  对预处理后的图像用多边形逼近其轮廓裁剪掉边角,过滤掉面积过小的轮廓
 *  \param    [in]  image   预处理后的图像
 *  \param    [in]  epsilon 曲线逼近的精度
 *  \return   无
 *  \retval   void
 */
void ImageFilter(Mat& image, int epsilon) {
  vector<vector<Point>> contours; //< 预处理图像的轮廓

  typedef vector<vector<Point>>::size_type Index;

  // 只保存最外围的轮廓的顶点信息
  findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point());
  // 寻找最大的轮廓,不考虑一张图片里面出现多个包裹的情况
  RotatedRect rotate, max_rotate;
  float max_area = 0.0f;
  Index idx;
  for (Index i = 0; i < contours.size(); ++i) {
    rotate = minAreaRect(contours[i]);
    if (rotate.size.area() > max_area) {
      max_area = rotate.size.area();
      max_rotate = rotate;
      idx = i;
    }
  }

  // 调试时画出最小外接矩形和逼近的多边形
  Mat drawing = Mat::zeros(image.size(), CV_8UC3);  //< 用来画图
  RNG rng(12345); //< 颜色随机数
  Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
  Point2f vertices[4];
  max_rotate.points(vertices);
  for (int j = 0; j < 4; ++j)
    line(drawing, vertices[j], vertices[(j + 1) % 4], color, 8);

  vector<Point> hull(contours[idx].size());  //< 轮廓的凸包
  vector<Point> poly(contours[idx].size()); //< 凸包的多边形逼近,裁剪边角
  convexHull(contours[idx], hull);
  approxPolyDP(hull, poly, epsilon, true);
  // 画出所有的轮廓
  drawContours(drawing, poly, -1, color, 8);
}
/*!
 *  \brief    Sobel边缘检测
 *
 *  采用OpenCV提供的方法进行Sobel边缘检测
 *  \param    [in]  src    待进行边缘检测的图像
 *  \return   边缘检测后的图像
 *  \retval   Mat
 */
static Mat
SobelEdgeDetection(const Mat& src) {
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  Mat grad_x, grad_y;
  Mat abs_grad_x, abs_grad_y;

  // Gradient X
  Sobel(src, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs(grad_x, abs_grad_x);
  // Gradient Y
  Sobel(src, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs(grad_y, abs_grad_y);

  return abs_grad_x + abs_grad_y;
}

/*!
 *  \brief    Canny边缘检测
 *
 *  利用OpenCV提供的方法进行Canny边缘检测
 *  \param    [in]  src    待进行边缘检测的图像
 *  \param    [in] thresh1    Canny检测阈值
 *  \param    [in] thresh2    Canny检测阈值
 *  \return   检测到的边缘图像
 *  \retval   Mat
 */
static Mat
CannyEdgeDetection(const Mat& src, double thresh1, double thresh2) {
  Mat edges;
  Canny(src, edges, thresh1, thresh2, 3);
  return edges;
}

/*!
 *  \brief    边缘检测
 *
 *  对输入图像进行边缘检测
 *  \param    [in]  src      输入图像
 *  \param    [in]  type     边缘检测算子类型
 *  \param    [in]  thresh1  Canny算子阈值
 *  \param    [in]  thresh2  Canny算子阈值
 *  \param    [out] edges    检测到的边缘图像
 *  \return   无
 *  \retval   无
 */
void
EdgeDetection(const Mat& src, int type, double thresh1, double thresh2, Mat& edges) {
  if (type == EDGE_SOBEL)
    edges = SobelEdgeDetection(src);
  else if (type == EDGE_CANNY)
    edges = CannyEdgeDetection(src, thresh1, thresh2);
}