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

using std::string;
using std::vector;
using cv::Directory;

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