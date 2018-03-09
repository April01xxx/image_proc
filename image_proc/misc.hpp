/*!
 *  \file     misc.hpp
 *  \brief    ������������
 *  \author   lancer wong
 *  \version  1.0
 *  \date     2018/03/06
 *
 *  Change History
 *  --
 *  |    Date     |   Version   |    Author   | Description |
 *  |-------------|-------------|-------------|-------------|
 *  | 2018/03/05  |     0.1     | lancer wong | Create      |
 *  | 2018/03/06  |     0.2     | lancer wong | Update      |
 */

#pragma once

#ifndef __MISC_HPP__
#define __MISC_HPP__
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>

//! ��ȡָ��·����ָ����׺�ļ������б�
size_t GetFileList(const std::string& path,
                 const std::string& extent,
                 std::vector<std::string>& lst);

//! ���ƻҶ�ֱ��ͼ
void DrawHistogram(const cv::Mat& input);
#endif // __MISC_HPP__
