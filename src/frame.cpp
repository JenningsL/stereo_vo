#include "frame.h"
#include <opencv2/highgui/highgui.hpp>

namespace stereo_vo {
using namespace cv;
Frame::Frame(Mat left, Mat right, std::shared_ptr<Camera> cam)
  :left_img(left), right_img(right), cam_(cam) {
  T_c_w_ = SE3(Eigen::Matrix3d::Identity(), Vector3d(0,0,0));
}

void Frame::calDepth() {
  Mat g1, g2, disp8;
  cvtColor(left_img, g1, CV_BGR2GRAY);
  cvtColor(right_img, g2, CV_BGR2GRAY);

  cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create();
  bm->compute(g1,g2,disp);
  disp.convertTo(disp, CV_32F, 1.0/16);
  normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

//  imshow("Left", left_img);
//  imshow("Right", right_img);
//  imshow("Depth map", disp8);
//  waitKey(0);
}

void Frame::setPose(const SE3 T_c_w) {
  T_c_w_ = T_c_w;
}

double Frame::getDepth(cv::Point2f pt) {
  uint32_t x = cvRound(pt.x);
  uint32_t y = cvRound(pt.y);
  // scale depth for better visualization
  return cam_->fx_ * cam_->baseline / disp.at<float>(y,x) / 100;
}

/**
 * Convert 2d point to 3d point in world coord,
 * according to the pose
 * @param pt
 * @return
 */
cv::Point3f Frame::toWorldCoord(cv::Point2f pt) {
  double d = getDepth(pt);
  if(d <= 0)
    return Point3f(0,0,0);
  Vector2d uv(pt.x, pt.y);
  Vector3d vec = cam_->pixel2world(uv, T_c_w_, d);
  return Point3f(vec[0], vec[1], vec[2]);
}

cv::Point2f Frame::toPixel(cv::Point3f pw) {
  Vector3d p_w(pw.x, pw.y, pw.z);
  Vector2d uv = cam_->world2pixel(p_w, T_c_w_);
  return Point2f(uv[0], uv[1]);
}

std::shared_ptr<Frame> Frame::CreateFrame(Mat left, Mat right, std::shared_ptr<Camera> cam) {
//  static long factory_id = 0;
  std::shared_ptr<Frame> frame(new Frame(left, right, cam));
  frame->calDepth();
  return frame;
}

}