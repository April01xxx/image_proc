/*!
 *  \file     test.cc
 *  \brief    测试模块
 *
 *
 *  \author   lancer wong
 *  \version  0.1
 *  \date     2018/03/06
 *
 *  Change History
 *  --
 *  |    Date     |   Version   |    Author   | Description |
 *  |-------------|-------------|-------------|-------------|
 *  | 2018/03/06  |     0.1     | lancer wong | Create      |
 *
 */
#include "misc.hpp"
#include "preprocess.hpp"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using std::vector;
using std::string;
using std::cout;
using std::cin;
using std::endl;
using cv::Mat;
using cv::Point;
using cv::RotatedRect;
using cv::Point2f;
using cv::Scalar;
using cv::imread; //< imread定义在cv命名空间中

int
main(int argc, char *argv[])
{
  if (argc != 2) {
    cout << "Usage: test <path>" << endl;
    return -1;
  }

  double thresh;
  cout << "请输入图像预处理的阈值: ";
  cin >> thresh;
  int edge_type;
  double thresh1 = 70, thresh2 = 100;
  bool flag = true;

  while (flag) {
    cout << "请输入边缘检测算子类型(0-Sobel算子,1-Canny算子): ";
    cin >> edge_type;

    switch (edge_type)
    {
    case EDGE_SOBEL:
      flag = false;
      break;
    case EDGE_CANNY:
      cout << "请输入Canny算子的两个阈值,以空格区分: ";
      cin >> thresh1, thresh2;
      flag = false;
      break;
    default:
      cout << "边缘检测算子类型错误,请检测后重新输入!\n" << endl;
    }
  }
  vector<string> filenames;
  size_t n = GetFileList(argv[1], "*.jpg", filenames);
  Mat image;
  Mat dst;
  Mat waybill;
  vector<vector<Point>> quardrangles;
  double duration;
  if (n > 0) {
    for (auto i = 0; i < filenames.size(); ++i) {
      image = imread(argv[1] + filenames[i], CV_LOAD_IMAGE_GRAYSCALE);
      if (!image.data) {
        cout << "Could not find " << filenames[i] << endl;
        continue;
      }

      /* 经过查看原图的灰度直方图,发现阈值选取为15比较合适,关于全局阈值的选取可以
         考虑使用最大类间方差法cv::THRESH_OTSU和三角形算法cv::THRESH_TRIANGLE
         */
      duration = static_cast<double>(cv::getTickCount());
      dst = ImagePreprocess(image, thresh, 20);
      FindQuardrangles(dst, quardrangles);
      waybill = CutWaybill(dst, edge_type, thresh1, thresh2);
      duration = static_cast<double>(cv::getTickCount()) - duration;
      duration /= cv::getTickFrequency();	//< 以秒为单位
      cout << filenames[i] << " cost: " << duration << "s" << endl;
    }
  }
  else {
    cout << "Could not found any file with extention .jpg" << endl;
  }
  system("pause");
  return 0;
}
