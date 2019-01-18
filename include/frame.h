//
// Created by jennings on 2019/1/16.
//

#ifndef STEREO_VO_FRAME_H
#define STEREO_VO_FRAME_H

#include "common.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "camera.h"

namespace stereo_vo {

class Frame {
public:
  Frame(Mat left, Mat right, std::shared_ptr<Camera> cam);
  static std::shared_ptr<Frame> CreateFrame(Mat left, Mat right, std::shared_ptr<Camera> cam);
  void calDepth();
  double getDepth(cv::Point2f pt);
  void setPose(const SE3 T_c_w);
  cv::Point3f toWorldCoord(cv::Point2f pt);
  cv::Point2f toPixel(cv::Point3f pw);
  Mat left_img;
  Mat right_img;
  Mat disp;
  SE3 T_c_w_;
private:
  long unsigned int id_;
  std::shared_ptr<Camera> cam_;
};

} // namespace stereo_vo

#endif //STEREO_VO_FRAME_H
