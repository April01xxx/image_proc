/*!
 *  \file     misc.cc
 *  \brief    辅助函数实现
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

#include "misc.hpp"
#include <string>
#include <vector>
#include <opencv2/contrib/contrib.hpp>
#include<opencv2/imgproc/imgproc.hpp>

using std::string;
using std::vector;
using cv::Directory;
using cv::Mat;
using cv::Scalar;
using cv::Point;
using cv::calcHist;
using cv::line;
using cv::minMaxLoc;

/*!
 *  \brief      读取指定路径下指定后缀的文件名称
 *
 *  \param      [in]  path    文件目录
 *  \param      [in]  extent  文件扩展名
 *  \param      [out] lst     文件名称列表
 *  \return     读取成功的文件名称数
 *              - 0   目录内无指定扩展名的文件
 *              - >0  目录内指定扩展名的文件数
 *  \retval size_t
 */
size_t
GetFileList(const string& path,
            const string& extent,
            vector<string>& lst) {
  if (path.empty() || extent.empty())
    return 0;

  Directory dir;
  lst = dir.GetListFiles(path, extent, false);
  return lst.size();
}

/*!
 *  \brief    绘制灰度图的直方图
 *  \param    [in]    input   输入图像
 *  \return   无
 *  \retval   void
 */
void
DrawHistogram(const Mat& input) {
  Mat hist;
  int hist_size = 256;
  int channels = 0;
  float range[] = { 0, 256 };
  const float *hist_range = { range };
  bool uniform = true, accumulate = false;

  calcHist(&input, 1, &channels, Mat(), hist, 1, &hist_size, &hist_range, uniform, accumulate);

  int scale = 1;
  Mat dstImage(hist_size * scale, hist_size, CV_8U, Scalar(0));
  //获取最大值和最小值  
  double minValue = 0;
  double maxValue = 0;
  minMaxLoc(hist, &minValue, &maxValue, 0, 0);  //  在cv中用的是cvGetMinMaxHistValue  
                                                   //绘制出直方图  
  int hpt = static_cast<int>(0.9 * hist_size);
  for (int i = 0; i < 256; i++)
  {
    float binValue = hist.at<float>(i);           //   注意hist中是float类型      
    int realValue = static_cast<int>(binValue * hpt / maxValue);
    line(dstImage, Point(i*scale, hist_size - 1), Point((i + 1)*scale - 1, hist_size - realValue), Scalar(255));
  }
}