//
// Created by jennings on 2019/1/17.
//

#include "odometry.h"
#include "g2o_types.h"
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <iterator>
#include <algorithm>
#include <random>

namespace stereo_vo {

Odometry::Odometry() : state_(INIT) {
  local_map_ = std::shared_ptr<LocalMap>(new LocalMap());
  cam_ = std::shared_ptr<Camera>(new Camera(707.0912, 707.0912, 601.8873, 183.1104));
//  cam_ = std::shared_ptr<Camera>(new Camera(518.0, 519.0, 325.5, 253.5));
}

std::shared_ptr<Frame> Odometry::addFrame(string l_img, string r_img) {
  Mat left_img = cv::imread(l_img);
  Mat right_img = cv::imread(r_img);
  cv::Mat l_gray, r_gray;
  cvtColor(left_img, l_gray, CV_BGR2GRAY);
  cvtColor(right_img, r_gray, CV_BGR2GRAY);
  std::shared_ptr<Frame> frame = Frame::CreateFrame(l_gray, r_gray, cam_);
  if(state_ == INIT) {
    frame->setReference(true);
    ref_frame_ = frame;
    extractFeature(frame);
    active_frames_.push_back(frame);
    state_ = OK;
  } else if(state_ == OK) {
    // tracking
    trackNewFrame(frame);
    // local BA
    active_frames_.push_back(frame);
    if(active_frames_.size() > 3) {
      active_frames_.erase(active_frames_.begin());
    }

    optimizeWindow();

    if(frame->getId() - ref_frame_->getId() > 5) {
      // change ref frame
      extractFeature(frame);
      ref_frame_ = frame;
    }
    // update velocity
    // last_delta_ = frame->T_c_w_ * last_frame_->T_c_w_.inverse();
  }
  last_frame_ = frame;
  return frame;
};

void Odometry::trackNewFrame(std::shared_ptr<Frame> frame) {
  VecVec2d px_ref;
  vector<double> depth_ref;
  Sophus::SE3 T21;
  // assume constant motion
  // T21 = last_delta_;

  // TODO: project map point to last frame
  int nPoints = 2000;
  int border = 50;
  VecVec2d candidates;
  last_frame_->extractFeaturePoints(candidates, nPoints, border);
  // generate pixels and depth in reference frame
  for (int i = 0; i < candidates.size(); i++) {
    float depth = last_frame_->getDepth(candidates[i]);
    if(depth <= 0 || std::isnan(depth) || std::isinf(depth)) {
      continue;
    }
    px_ref.push_back(candidates[i]);
    depth_ref.push_back(depth);
  }
  DirectPoseEstimationMultiLayer(last_frame_->left_img, frame->left_img, px_ref, depth_ref, cam_, T21);
  frame->setPose(T21*last_frame_->T_c_w_);
}

void Odometry::optimizeWindow() {
  std::shared_ptr<Frame> ref_frame = active_frames_[0];
  std::shared_ptr<Frame> cur_frame = active_frames_[active_frames_.size()-1];
  vector<uint32_t> pt_ids;
  vector<Vector3d> points;
  vector<SE3> poses;
  vector<cv::Mat> images;
  vector<float *> colors;
  vector<MapPointPtr> mpts;
  local_map_->projectToFrame(mpts, colors, ref_frame);
  for(auto& mpt:mpts) {points.push_back(mpt->pt);}

  for(int i =0; i < active_frames_.size(); i++) {
    auto frame = active_frames_[i];
    poses.push_back(frame->T_c_w_);
    images.push_back(frame->left_img);
  }

  WindowDirectBA window_ba(cam_);
  window_ba.optimize(poses, points, images, colors);
  //active_frames_[active_frames_.size()-1]->setPose(poses[poses.size()-1]);
  for(int i =0; i < active_frames_.size(); i++)
    active_frames_[i]->setPose(poses[i]);
//  for(int i =0; i < pt_ids.size(); i++)
//    local_map_->updatePoint(pt_ids[i], points[i]);

  vector<MapPointPtr> outliners = cur_frame->getOutlier(mpts, colors);
  cout << "outliners: " << outliners.size() << endl;
  local_map_->removePoints(outliners);
  // delete color data
  for (auto &c: colors) delete[] c;
}

list<cv::Point2f> Odometry::getProjectedPoints() {
  vector<Vector3d> points;
  local_map_->getAllPoints(points);
  list<cv::Point2f> pts;
  for(Vector3d p3d:points) {
    Vector2d p2d = last_frame_->toPixel(p3d);
    if(!last_frame_->isInside(p2d[0], p2d[1], 0))
      continue;
    pts.push_back(cv::Point2f(p2d[0], p2d[1]));
  }
  return pts;
}

void Odometry::extractFeature(std::shared_ptr<Frame> frame) {
  local_map_->clear();

  int border = 50;
  VecVec2d candidates;
  frame->extractFeaturePoints(candidates, 4000, border);
  // generate pixels and depth in reference frame
  for (int i = 0; i < candidates.size(); i++) {
    Vector3d map_point = frame->toWorldCoord(Vector2d(candidates[i]));
    float depth = map_point[2];
    if(depth <= 0 || std::isnan(depth) || std::isinf(depth)) {
      continue;
    }
    local_map_->addPoint(map_point, frame->getId());
  }
}

}