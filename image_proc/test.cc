/*!
 *  \file     test.cc
 *  \brief    ����ģ��
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
using cv::imread; //< imread������cv�����ռ���

int
main(int argc, char *argv[])
{
  if (argc != 2) {
    cout << "Usage: test <path>" << endl;
    return -1;
  }

  double thresh;
  cout << "������ͼ��Ԥ�������ֵ: ";
  cin >> thresh;
  int edge_type;
  bool flag = true;

  while (flag) {
    cout << "�������Ե�����������(0-Sobel����,1-Canny����): ";
    cin >> edge_type;
    double thresh1, thresh2;

    switch (edge_type)
    {
      case EDGE_SOBEL:
        flag = false;
        break;
      case EDGE_CANNY:
        cout << "������Canny���ӵ�������ֵ,�Կո�����: ";
        cin >> thresh1, thresh2;
        flag = false;
        break;
      default:
        cout << "��Ե����������ʹ���,�������������!\n" << endl;
    }
  }
  vector<string> filenames;
  size_t n = GetFileList(argv[1], "*.jpg", filenames);
  Mat image;
  Mat dst;
  if (n > 0) {
    for (auto i = 0; i < filenames.size(); ++i) {
      image = imread(argv[1] + filenames[i], CV_LOAD_IMAGE_GRAYSCALE);
      if (!image.data) {
        cout << "Could not find " << filenames[i] << endl;
        continue;
      }
      /* ��������,threshȡֵ15����(��СҲ����),����ȫ����ֵ��ѡȡ����
         ����ʹ�������䷽�cv::THRESH_OTSU���������㷨cv::THRESH_TRIANGLE
      */
      dst = ImagePreprocess(image, thresh, 20);
    }
  }
  else {
    cout << "Could not found any file with extention .jpg" << endl;
  }
  system("pause");
  return 0;
}