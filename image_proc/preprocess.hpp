/*!
 *  \file     preprocess.hpp
 *  \brief    ͼ����������
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
#pragma once

#ifndef __IMGPROC_HPP__
#define __IMGPROC_HPP__

#include <opencv2/core/core.hpp>

//! type of the EdgeDetection operation
enum EdgeDetectionType
{
  EDGE_SOBEL = 0, ///< Sobel����
  EDGE_CANNY ///< Canny����
};

//! ͼ��򵥶�ֵ��,��ͼ����������ĺ�ɫ��������ȥ��
cv::Mat ImagePreprocess(const cv::Mat& src, double thresh, int epsilon);

//! ͼ�����,�ҳ����������������������С��Ӿ���
cv::RotatedRect ImageFilter(cv::Mat& src, int epsilon);

//! ��Ե���,�ṩSobel��Canny����ѡ��
void EdgeDetection(const cv::Mat& src,
                   int type,
                   double thresh1,
                   double thresh2,
                   cv::Mat& edges);
#endif // __IMGPROC_HPP__