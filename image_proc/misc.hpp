/*!
 *  \file     misc.hpp
 *  \brief    辅助函数声明
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

//! 获取指定路径下指定后缀文件名称列表
size_t GetFileList(const std::string& path,
                 const std::string& extent,
                 std::vector<std::string>& lst);

//! 绘制灰度直方图
void DrawHistogram(const cv::Mat& input);

//! 绘制四边形
void DrawQuardrangle(cv::Mat& image, const std::vector<std::vector<cv::Point>>& quardrangles);
#endif // __MISC_HPP__
