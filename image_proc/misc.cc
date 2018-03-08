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

using std::string;
using std::vector;
using cv::Directory;

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