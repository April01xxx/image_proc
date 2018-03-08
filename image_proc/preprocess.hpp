/*!
 *  \file     preprocess.hpp
 *  \brief    图像处理函数声明
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
  EDGE_SOBEL = 0, ///< Sobel算子
  EDGE_CANNY ///< Canny算子
};

//! 图像简单二值化,将图像中无意义的黑色背景区域去除
cv::Mat ImagePreprocess(const cv::Mat& src, double thresh);

//! 图像过滤,对预处理得到的图像轮廓采用曲线逼近并去除面积过小的区域
void ImageFilter(cv::Mat& image, int epsilon);

//! 边缘检测,提供Sobel和Canny两种选择
void EdgeDetection(const cv::Mat& src,
                   int type,
                   double thresh1,
                   double thresh2,
                   cv::Mat& edges);
#endif // __IMGPROC_HPP__