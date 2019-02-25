//
// Created by jennings on 2019/2/22.
//

#ifndef STEREO_VO_DEPTH_FILTER_H
#define STEREO_VO_DEPTH_FILTER_H

#include <iostream>
#include <vector>
#include <fstream>
using namespace std;
#include <boost/timer.hpp>

// for sophus
#include <sophus/se3.h>
using Sophus::SE3;

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

#include "camera.h"
/**********************************************
* 本程序演示了单目相机在已知轨迹下的稠密深度估计
* 使用极线搜索 + NCC 匹配的方式，与书本的 13.2 节对应
* 请注意本程序并不完美，你完全可以改进它——我其实在故意暴露一些问题。
***********************************************/

namespace depth_filter {
// ------------------------------------------------------------------
// parameters
const int border = 20;
const int ncc_window_size = 4;	// NCC 取的窗口半宽度
const int ncc_area = (2*ncc_window_size+1)*(2*ncc_window_size+1); // NCC窗口面积
const double min_cov = 0.1;	// 收敛判定：最小方差
const double max_cov = 10;	// 发散判定：最大方差

using namespace stereo_vo;
// ------------------------------------------------------------------
// 重要的函数

// 根据新的图像更新深度估计
bool update(
        const CameraPtr cam,
        const Mat& ref,
        const Mat& curr,
        const SE3& T_C_R,
        const VecVec2d& pts_2d,
        vector<float>& depth,
        vector<float>& depth_cov,
        vector<bool>& success
);

// 极线搜索
bool epipolarSearch(
        const CameraPtr cam,
        const Mat& ref,
        const Mat& curr,
        const SE3& T_C_R,
        const Vector2d& pt_ref,
        const double& depth_mu,
        const double& depth_cov,
        Vector2d& pt_curr
);

// 更新深度滤波器
bool updateDepthFilter(
        const CameraPtr cam,
        const Vector2d& pt_ref,
        const Vector2d& pt_curr,
        const SE3& T_C_R,
        const int index,
        vector<float>& depth,
        vector<float>& depth_cov
);

// 计算 NCC 评分
double NCC( const Mat& ref, const Mat& curr, const Vector2d& pt_ref, const Vector2d& pt_curr );

// 双线性灰度插值
inline double getBilinearInterpolatedValue( const Mat& img, const Vector2d& pt ) {
  uchar* d = & img.data[ int(pt(1,0))*img.step+int(pt(0,0)) ];
  double xx = pt(0,0) - floor(pt(0,0));
  double yy = pt(1,0) - floor(pt(1,0));
  return  (( 1-xx ) * ( 1-yy ) * double(d[0]) +
           xx* ( 1-yy ) * double(d[1]) +
           ( 1-xx ) *yy* double(d[img.step]) +
           xx*yy*double(d[img.step+1]))/255.0;
}

// ------------------------------------------------------------------
// 一些小工具
// 显示估计的深度图
void plotDepth( const Mat& depth );

// 检测一个点是否在图像边框内
inline bool inside(const Vector2d& pt, const int width, const int height) {
  return pt(0,0) >= border && pt(1,0)>=border
         && pt(0,0)+border<width && pt(1,0)+border<=height;
}

// 显示极线匹配
void showEpipolarMatch( const Mat& ref, const Mat& curr, const Vector2d& px_ref, const Vector2d& px_curr );

// 显示极线
void showEpipolarLine( const Mat& ref, const Mat& curr, const Vector2d& px_ref, const Vector2d& px_min_curr, const Vector2d& px_max_curr );
// ------------------------------------------------------------------

}

#endif //STEREO_VO_DEPTH_FILTER_H
