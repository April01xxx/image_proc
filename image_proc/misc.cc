/*!
 *  \file     misc.cc
 *  \brief    ��������ʵ��
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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

using std::string;
using std::vector;
using cv::Directory;
using cv::Mat;
using cv::Scalar;
using cv::Point;
using cv::calcHist;
using cv::line;
using cv::minMaxLoc;
using cv::polylines;

/*!
 *  \brief      ��ȡָ��·����ָ����׺���ļ�����
 *
 *  \param      [in]  path    �ļ�Ŀ¼
 *  \param      [in]  extent  �ļ���չ��
 *  \param      [out] lst     �ļ������б�
 *  \return     ��ȡ�ɹ����ļ�������
 *              - 0   Ŀ¼����ָ����չ�����ļ�
 *              - >0  Ŀ¼��ָ����չ�����ļ���
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
 *  \brief    ���ƻҶ�ͼ��ֱ��ͼ
 *  \param    [in]    input   ����ͼ��
 *  \return   ��
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
  //��ȡ���ֵ����Сֵ  
  double minValue = 0;
  double maxValue = 0;
  minMaxLoc(hist, &minValue, &maxValue, 0, 0);  //  ��cv���õ���cvGetMinMaxHistValue  
                                                   //���Ƴ�ֱ��ͼ  
  int hpt = static_cast<int>(0.9 * hist_size);
  for (int i = 0; i < 256; i++)
  {
    float binValue = hist.at<float>(i);           //   ע��hist����float����      
    int realValue = static_cast<int>(binValue * hpt / maxValue);
    line(dstImage, Point(i*scale, hist_size - 1), Point((i + 1)*scale - 1, hist_size - realValue), Scalar(255));
  }
}

void
DrawQuardrangle(Mat& image, const vector<vector<Point> >& quardrangles)
{
  for (size_t i = 0; i < quardrangles.size(); i++)
  {
    const Point* p = &quardrangles[i][0];
    int n = (int)quardrangles[i].size();
    polylines(image, &p, &n, 1, true, Scalar(255, 0, 0), 16, CV_AA);
  }
}