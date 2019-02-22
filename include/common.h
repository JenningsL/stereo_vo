//
// Created by jennings on 2019/1/16.
//

#ifndef STEREO_VO_COMMON_H
#define STEREO_VO_COMMON_H

// define the commonly included file to avoid a long include list
// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SO3;
using Sophus::SE3;

// for cv
#include <opencv2/core/core.hpp>
using cv::Mat;

// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include <algorithm>

using namespace std;

inline float GetPixelValue(const cv::Mat &img, float x, float y) {
  uchar *data = &img.data[int(y) * img.step + int(x)];
  float xx = x - floor(x);
  float yy = y - floor(y);
  return float(
          (1 - xx) * (1 - yy) * data[0] +
          xx * (1 - yy) * data[1] +
          (1 - xx) * yy * data[img.step] +
          xx * yy * data[img.step + 1]
  );
}


typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> VecSE3;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVec2d;

#endif //STEREO_VO_COMMON_H
