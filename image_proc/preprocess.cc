/*!
 *  \file     preprocess.cc
 *  \brief    ͼ������
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
 *  \brief    ͼ��Ԥ����
 *
 *  ���ǵ�ʵ��ͼ��ֱ��ʸ��Ҵ󲿷�Ϊ��ɫ��������,ͨ���򵥵���ֵ�ָ���ȡ��
 *  ǰ��ͼ��
 *  \param    [in]  src    �������ԭͼ��
 *  \param    [in]  thresh    ȥ����ɫ�����������ֵ,����ȡСһ��
 *  \return   ������ͼ��
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
  /* ͼ��Ԥ����׶�,�����ͺ�ʴȥ�������ڵĹ�������,ʹ���������⻬. */
  morphologyEx(thresh_image, morph_image, CV_MOP_CLOSE, element);

  return morph_image;
}

/*!
 *  \brief    ͼ��Ľ�һ������
 *  ��Ԥ������ͼ���ö���αƽ��������ü����߽�,���˵������С������
 *  \param    [in]  image   Ԥ������ͼ��
 *  \param    [in]  epsilon ���߱ƽ��ľ���
 *  \return   ��
 *  \retval   void
 */
void ImageFilter(Mat& image, int epsilon) {
  vector<vector<Point>> contours; //< Ԥ����ͼ�������

  typedef vector<vector<Point>>::size_type Index;

  // ֻ��������Χ�������Ķ�����Ϣ
  findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point());
  // Ѱ����������,������һ��ͼƬ������ֶ�����������
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

  // ����ʱ������С��Ӿ��κͱƽ��Ķ����
  Mat drawing = Mat::zeros(image.size(), CV_8UC3);  //< ������ͼ
  RNG rng(12345); //< ��ɫ�����
  Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
  Point2f vertices[4];
  max_rotate.points(vertices);
  for (int j = 0; j < 4; ++j)
    line(drawing, vertices[j], vertices[(j + 1) % 4], color, 8);

  vector<Point> hull(contours[idx].size());  //< ������͹��
  vector<Point> poly(contours[idx].size()); //< ͹���Ķ���αƽ�,�ü��߽�
  convexHull(contours[idx], hull);
  approxPolyDP(hull, poly, epsilon, true);
  // �������е�����
  drawContours(drawing, poly, -1, color, 8);
}
/*!
 *  \brief    Sobel��Ե���
 *
 *  ����OpenCV�ṩ�ķ�������Sobel��Ե���
 *  \param    [in]  src    �����б�Ե����ͼ��
 *  \return   ��Ե�����ͼ��
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
 *  \brief    Canny��Ե���
 *
 *  ����OpenCV�ṩ�ķ�������Canny��Ե���
 *  \param    [in]  src    �����б�Ե����ͼ��
 *  \param    [in] thresh1    Canny�����ֵ
 *  \param    [in] thresh2    Canny�����ֵ
 *  \return   ��⵽�ı�Եͼ��
 *  \retval   Mat
 */
static Mat
CannyEdgeDetection(const Mat& src, double thresh1, double thresh2) {
  Mat edges;
  Canny(src, edges, thresh1, thresh2, 3);
  return edges;
}

/*!
 *  \brief    ��Ե���
 *
 *  ������ͼ����б�Ե���
 *  \param    [in]  src      ����ͼ��
 *  \param    [in]  type     ��Ե�����������
 *  \param    [in]  thresh1  Canny������ֵ
 *  \param    [in]  thresh2  Canny������ֵ
 *  \param    [out] edges    ��⵽�ı�Եͼ��
 *  \return   ��
 *  \retval   ��
 */
void
EdgeDetection(const Mat& src, int type, double thresh1, double thresh2, Mat& edges) {
  if (type == EDGE_SOBEL)
    edges = SobelEdgeDetection(src);
  else if (type == EDGE_CANNY)
    edges = CannyEdgeDetection(src, thresh1, thresh2);
}