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

cv::RNG rng(12345); // ��ɫ���������

/*!
 *  \brief    ͼ��Ľ�һ������
 *  ��Ԥ������ͼ���ö���αƽ��������ü����߽�,���˵������С������
 *  \param    [in]  src     Ԥ������ͼ��
 *  \param    [in]  epsilon ���߱ƽ��ľ���
 *  \return   �����������С��Ӿ���
 *  \retval   cv::RotatedRect
 */
static RotatedRect
ImageFilter(Mat& src, int epsilon) {
  vector<vector<Point>> contours; //< Ԥ����ͼ�������

  typedef vector<vector<Point>>::size_type Index;

  // ֻ��������Χ�������Ķ�����Ϣ
  findContours(src, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point());
  // Ѱ����������,������һ��ͼƬ������ֶ�����������
  RotatedRect rotate, max_rotate;
  float max_area = 0.0f;
  Index idx;
  for (Index i = 0; i < contours.size(); ++i) {
    // opencv��ʵ�ֻ��ȼ�����㼯��͹��Ȼ������Rotating calipers�㷨���
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
 *  \brief    ͼ��Ԥ����
 *
 *  ���ǵ�ʵ��ͼ��ֱ��ʸ��Ҵ󲿷�Ϊ��ɫ��������,ͨ���򵥵���ֵ�ָ���ȡ��
 *  ǰ��ͼ��
 *  \param    [in]  src    �������ԭͼ��
 *  \param    [in]  thresh    ȥ����ɫ�����������ֵ,����ȡСһ��
 *  \param    [in]  epsilon   ����αƽ�����
 *  \return   ������ͼ��
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

  /* ֻ�Ǵ��ԵĴ���,���Զ�ԭͼ��С����,��ȡʱת��ԭ�ߴ�.
     �����ص���Ӿ��β���������ͼ����ʱ����ֽ�ȡ��������
  */
  resize(src, gauss_image, Size(0, 0), fx, fy, CV_INTER_LINEAR);

  GaussianBlur(gauss_image, gauss_image, Size(3, 3), 0);
  threshold(gauss_image, thresh_image, thresh, 255, CV_THRESH_BINARY);
  /* ͼ��Ԥ����׶�,�����ͺ�ʴȥ�������ڵĹ�������,ʹ���������⻬. */
  morphologyEx(thresh_image, morph_image, CV_MOP_CLOSE, element);
  resize(morph_image, morph_image, Size(0, 0), 1 / fx, 1 / fy, CV_INTER_LINEAR);

  RotatedRect r = ImageFilter(morph_image, epsilon);
  // �ü�����ת��������ת��
  Point2f vertices[4], dst_vertices[4];

  r.points(vertices);
  /* opencvʵ����points����������ת���ε�4��������˳ʱ�뷽��洢:
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
 *  \brief    ������ͼ���н�ȡ������浥����
 *  \param    [in]    input   ������ݰ�����ͼ��
 *  \param    [in]    edge_type ���õı�Ե�������
 *  \param    [in]    thresh1   Canny������ֵ
 *  \param    [in]    thresh2   Canny������ֵ
 *  \return   ��ȡ���Ŀ���浥ͼ��
 *  \retval   cv::Mat
 */
Mat
CutWaybill(const Mat& input, int edge_type, double thresh1, double thresh2) {
  Mat waybill = input;

  // ȥ���Ҷ�ֵС��ĳһ��ֵ�ĵ�(��Ե��Ϣ��ǿ�Ƚϸ�
  // �����浥Ϊ�ı����Ҷ���X/Y�����ݶȱ仯����,�Ȳ���Sobel���Ӽ����Ե
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
