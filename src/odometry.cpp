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
    frame->is_keyframe = true;
    key_frames.push_back(frame);
    frame->selectCandidates();
    activateMapPoints(frame, 10);
    active_frames_.push_back(frame);
    state_ = OK;
  } else if(state_ == OK) {
    // tracking using lastest keyframe
    trackNewFrame(frame);
    frame->selectCandidates();
    // update depth
//    if(next_keyframe)
//      next_keyframe->updateCandidates(frame);
    // local BA
    /*
    active_frames_.push_back(frame);
    if(active_frames_.size() > 3) {
      active_frames_.erase(active_frames_.begin());
    }
    optimizeWindow();
     */

    if(isNewKeyFrame(frame)) {
      if(next_keyframe) {
//        activateMapPoints(next_keyframe, depth_filter::min_cov);
        activateMapPoints(next_keyframe, 10);
        key_frames.push_back(next_keyframe);
      }
      // update next_keyframe
      frame->is_keyframe = true;
      next_keyframe = frame;
    }
    // update velocity
    // last_delta_ = frame->T_c_w_ * last_frame_->T_c_w_.inverse();

    local_map_->removeInvisibleMapPoints(frame);
    local_map_->removeOutlier(key_frames.back(), frame);
  }
  all_frames.push_back(frame);
  return frame;
}

bool Odometry::isNewKeyFrame(FramePtr frame) {
  // TODO: determined based on mean square optical flow
  return frame->getId() - key_frames.back()->getId() > 3;
}

void Odometry::trackNewFrame(FramePtr frame) {
  // project map point to the latest keyframe
  vector<MapPointPtr> mpts;
  vector<Vector2d> candidates;
  // FramePtr latest_keyframe = key_frames.back();
  FramePtr latest_keyframe = all_frames.back();
  local_map_->projectToFrame(candidates, mpts, latest_keyframe);

  VecVec2d px_ref;
  vector<double> depth_ref;
  Sophus::SE3 T21;
  // assume constant motion
  // T21 = last_delta_;
  // generate pixels and depth in reference frame
  std::random_shuffle(candidates.begin(), candidates.end());
  int max_num = 2000;
  int count = 0;
  for (int i = 0; i < candidates.size() && count < max_num; i++) {
    float depth = latest_keyframe->getDepth(candidates[i]);
    if(depth <= 0 || std::isnan(depth) || std::isinf(depth)) {
      continue;
    }
    px_ref.push_back(candidates[i]);
    depth_ref.push_back(depth);
    count++;
  }
  cout << "Tracking with " << px_ref.size() << " map points" << endl;
  DirectPoseEstimationMultiLayer(latest_keyframe->left_img, frame->left_img, px_ref, depth_ref, cam_, T21);
  frame->setPose(T21*latest_keyframe->T_c_w_);
}

void Odometry::optimizeWindow() {
  std::shared_ptr<Frame> ref_frame = active_frames_[0];
  std::shared_ptr<Frame> cur_frame = active_frames_[active_frames_.size()-1];

  vector<SE3> poses;
  vector<cv::Mat> images;
  vector<float *> colors;

  vector<MapPointPtr> mpts;
  vector<Vector2d> projections;
  vector<Vector3d> points;
  local_map_->projectToFrame(projections, mpts, ref_frame);
  for(auto& mpt:mpts) {
    points.push_back(mpt->pt);
  }
  ref_frame->getKeypointColors(points, colors);

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
  for(int i =0; i < mpts.size(); i++)
    mpts[i]->update(points[i]);

  vector<MapPointPtr> outliners = cur_frame->getOutlier(mpts, colors);
  cout << "outliners: " << outliners.size() << endl;
  local_map_->removePoints(outliners);
  // delete color data
  for (auto &c: colors) delete[] c;
}

void Odometry::getProjectedPoints(vector<cv::Point2f>& pts, vector<float>& depth) {
  vector<MapPointPtr> points;
  local_map_->getAllPoints(points);
  std::random_shuffle(points.begin(), points.end());
  int max_num = 2000;
  int count = 0;
  for(MapPointPtr mp:points) {
    if(count >= max_num)
      break;
    Vector2d p2d = all_frames.back()->toPixel(mp->pt);
    if(!all_frames.back()->isInside(p2d[0], p2d[1], 0))
      continue;
    pts.push_back(cv::Point2f(p2d[0], p2d[1]));
    depth.push_back(mp->pt[2]);
    count++;
  }
//  auto it = max_element(std::begin(depth), std::end(depth));
//  cout << "max depth: " << *it << endl;
}

void Odometry::activateMapPoints(FramePtr frame, float cov_threshold) {
  // local_map_->clear();
  // vector<MapPointPtr> points;
  // local_map_->getAllPoints(points);
  // TODO: select points to add
  for(auto pair:frame->candidates) {
    MapPointPtr m_point = pair.second;
    if(m_point->conv <= cov_threshold) {
      // points.push_back(m_point);
      local_map_->addPoint(m_point);
    }
  }
//  std::random_shuffle(points.begin(), points.end());
//  local_map_->clear();
//  for(int i = 0; i < 2000 && i < points.size(); i++) {
//    local_map_->addPoint(points[i]);
//  }
}

}