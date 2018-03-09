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

using std::vector;
using cv::Mat;
using cv::Size;
using cv::Point;
using cv::RotatedRect;
using cv::Point2f;
using cv::Scalar;
using cv::RNG;
using cv::Vec4i;
using cv::resize;
using cv::getPerspectiveTransform;
using cv::warpPerspective;
using cv::cvtColor;
using cv::findContours;
using cv::contourArea;
using cv::drawContours;
using cv::minAreaRect;

cv::RNG rng(12345); // 颜色随机数生成

/*!
 *  \brief    图像的进一步处理
 *  对预处理后的图像用多边形逼近其轮廓裁剪掉边角,过滤掉面积过小的轮廓
 *  \param    [in]  src     预处理后的图像
 *  \param    [in]  epsilon 曲线逼近的精度
 *  \return   最大轮廓的最小外接矩形
 *  \retval   cv::RotatedRect
 */
static RotatedRect
ImageFilter(Mat& src, int epsilon) {
  vector<vector<Point>> contours; //< 预处理图像的轮廓

  typedef vector<vector<Point>>::size_type Index;

  // 只保存最外围的轮廓的顶点信息
  findContours(src, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point());
  // 寻找最大的轮廓,不考虑一张图片里面出现多个包裹的情况
  RotatedRect rotate, max_rotate;
  float max_area = 0.0f;
  Index idx;
  for (Index i = 0; i < contours.size(); ++i) {
    // opencv的实现会先计算出点集的凸包然后利用Rotating calipers算法求解
    rotate = minAreaRect(contours[i]);
    if (rotate.size.area() > max_area) {
      max_area = rotate.size.area();
      max_rotate = rotate;
      idx = i;
    }
  }

  return max_rotate;
}


/*!
 *  \brief    图像预处理
 *
 *  考虑到实际图像分辨率高且大部分为黑色背景区域,通过简单的阈值分割提取出
 *  前景图像
 *  \param    [in]  src    待处理的原图像
 *  \param    [in]  thresh    去除黑色背景区域的阈值,可以取小一点
 *  \param    [in]  epsilon   多边形逼近精度
 *  \return   处理后的图像
 *  \retval   Mat
 */
Mat
ImagePreprocess(const Mat& src, double thresh, int epsilon) {
  Mat gauss_image;
  Mat thresh_image;
  Mat morph_image;
  Mat edges;
  Mat element = getStructuringElement(cv::MORPH_RECT, Size(15, 15));
  float fx = 0.5f, fy = 0.5f;

  /* 只是粗略的处理,可以对原图缩小后处理,截取时转回原尺寸.
     当返回的外接矩形部分区域在图像外时会出现截取过多的情况
  */
  resize(src, gauss_image, Size(0, 0), fx, fy, CV_INTER_LINEAR);

  GaussianBlur(gauss_image, gauss_image, Size(3, 3), 0);
  threshold(gauss_image, thresh_image, thresh, 255, CV_THRESH_BINARY);
  /* 图像预处理阶段,先膨胀后腐蚀去除区域内的孤立区域,使轮廓连续光滑. */
  morphologyEx(thresh_image, morph_image, CV_MOP_CLOSE, element);
  resize(morph_image, morph_image, Size(0, 0), 1 / fx, 1 / fy, CV_INTER_LINEAR);

  RotatedRect r = ImageFilter(morph_image, epsilon);
  // 裁剪出旋转矩形区域并转正
  Point2f vertices[4], dst_vertices[4];

  r.points(vertices);
  /* opencv实现中points方法返回旋转矩形的4个顶点以顺时针方向存储:
     bottom -> left -> top -> right
  */
  dst_vertices[0] = Point2f(0, r.size.height);
  dst_vertices[1] = Point2f(0, 0);
  dst_vertices[2] = Point2f(r.size.width, 0);
  dst_vertices[3] = Point2f(r.size.width, r.size.height);

  Mat map_matrix = getPerspectiveTransform(vertices, dst_vertices);
  Mat roi;
  warpPerspective(src, roi, map_matrix, r.size);

  return roi;
}

/*!
 *  \brief    从输入图像中截取出快递面单部分
 *  \param    [in]    input   包含快递包裹的图像
 *  \param    [in]    edge_type 采用的边缘检测算子
 *  \param    [in]    thresh1   Canny算子阈值
 *  \param    [in]    thresh2   Canny算子阈值
 *  \return   截取出的快递面单图像
 *  \retval   cv::Mat
 */
Mat
CutWaybill(const Mat& input, int edge_type, double thresh1, double thresh2) {
  Mat waybill = input;

  // 去除灰度值小于某一阈值的点(边缘信息的强度较高
  // 考虑面单为四边形且对于X/Y方向梯度变化明显,先采用Sobel算子计算边缘
  EdgeDetection(waybill, edge_type, thresh1, thresh2, waybill);

  Mat copy = waybill.clone();
  cvtColor(copy, copy, CV_GRAY2BGR);

  GaussianBlur(waybill, waybill, Size(3, 3), 0);
  threshold(waybill, waybill, 50, 0, CV_THRESH_TOZERO);
  EdgeDetection(waybill, EDGE_CANNY, 60, 180, waybill);

  vector<vector<Point>> contours;
  findContours(waybill, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  RotatedRect rot_rect;
  Point2f vertices[4];
  for (int i = 0; i < contours.size(); ++i) {
    rot_rect = minAreaRect(contours[i]);
    if (rot_rect.size.area() > 250000) {
      rot_rect.points(vertices);
      Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
      for (int j = 0; j < 4; ++j)
        line(copy, vertices[j], vertices[(j + 1) % 4], color, 16);
    }
  }

  return waybill;
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
