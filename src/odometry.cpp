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
  map = std::shared_ptr<Map>(new Map());
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
    activateMapPoints(frame);
    active_frames_.push_back(frame);
    state_ = OK;
  } else if(state_ == OK) {
    // tracking using lastest keyframe
    trackNewFrame(frame);
    frame->selectCandidates();
    // FIXME: update depth is slow
    if(next_keyframe && false) {
      auto begin = std::chrono::steady_clock::now();
      next_keyframe->updateCandidates(frame);
      auto end = std::chrono::steady_clock::now();
      cout << "Update depth time consumed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << endl;
    }

    if(isNewKeyFrame(frame)) {
      cout << "New keyframe: " << frame->getId() << endl;
      if(next_keyframe) {
        key_frames.push_back(next_keyframe);
        activateMapPoints(next_keyframe);
        // auto begin = std::chrono::steady_clock::now();
        // optimizeWindow();
        // auto end = std::chrono::steady_clock::now();
        // cout << "Local BA time consumed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << endl;
      }
      // update next_keyframe
      frame->is_keyframe = true;
      next_keyframe = frame;
    }
    // update velocity
    last_delta_ = frame->T_c_w_ * all_frames.back()->T_c_w_.inverse();

    map->removeInvisibleMapPoints(frame);
    map->removeOutlier(key_frames.back(), frame);
  }
  all_frames.push_back(frame);
  return frame;
}

bool Odometry::isNewKeyFrame(FramePtr frame) {
  // TODO: determined based on mean square optical flow
  SE3 delta = frame->T_c_w_ * key_frames.back()->T_c_w_.inverse();
  double t = delta.translation().norm();
  cout << "t_norm: " << t << endl;
  return t > 0.03;
}

void Odometry::trackNewFrame(FramePtr frame) {
  vector<MapPointPtr> mpts;
  FramePtr last_frame = all_frames.back();

  VecVec2d px_ref;
  vector<double> depth_ref;
  Sophus::SE3 T21;
  // FIXME: assume constant motion is not working
  // T21 = last_delta_;
  // generate pixels and depth in reference frame
  map->projectToFrame(px_ref, depth_ref, mpts, last_frame, 2000);
  auto begin = std::chrono::steady_clock::now();
  cout << "Tracking with " << px_ref.size() << " map points. ";
  DirectPoseEstimationMultiLayer(last_frame->left_img, frame->left_img, px_ref, depth_ref, cam_, T21);
  frame->setPose(T21*last_frame->T_c_w_);
  auto end = std::chrono::steady_clock::now();
  cout << "Time consumed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << endl;
}

void Odometry::optimizeWindow() {
  if(key_frames.size() < 3) {return;}
  int ref_i = std::max(int(key_frames.size()-3), 0);
  std::shared_ptr<Frame> ref_frame = key_frames[ref_i];

  VecSE3 poses;
  vector<cv::Mat> images;
  vector<float *> colors;

  vector<MapPointPtr> mpts;
  VecVec2d projections;
  vector<double> depths;
  VecVec3d points;
  map->projectToFrame(projections, depths, mpts, ref_frame, 2000);
  for(auto& mpt:mpts) {
    points.push_back(mpt->pt);
  }
  ref_frame->getKeypointColors(points, colors);

  vector<FramePtr> opt_frames;
  for(int i = key_frames.size()-1; i >= key_frames.size()-3 && i >= 0; i--) {
    auto frame = key_frames[i];
    poses.insert(poses.begin(), frame->T_c_w_);
    images.insert(images.begin(), frame->left_img);
    opt_frames.insert(opt_frames.begin(), frame);
  }

  //if(poses.size() < 3) {return;}

  WindowDirectBA window_ba(cam_);
  window_ba.optimize(poses, points, images, colors);
  for(int i =0; i < opt_frames.size(); i++)
    opt_frames[i]->setPose(poses[i]);
//  for(int i =0; i < mpts.size(); i++)
//    mpts[i]->update(points[i]);

  // delete color data
  for (auto &c: colors) delete[] c;
}

void Odometry::getProjectedPoints(vector<cv::Point2f>& pts, vector<float>& depth) {
  vector<MapPointPtr> points;
  vector<MapPointPtr> visiable_points;
  map->getLocalPoints(points);
  for(MapPointPtr mp:points) {
    Vector2d p2d = all_frames.back()->toPixel(mp->pt);
    if(!all_frames.back()->isInside(p2d[0], p2d[1], 0))
      continue;
    visiable_points.push_back(mp);
  }

  std::random_shuffle(visiable_points.begin(), visiable_points.end());
  int max_num = 10000;
  int count = 0;
  for(MapPointPtr mp:points) {
    if(count >= max_num)
      break;
    FramePtr last_frame =  all_frames.back();
    Vector3d p_cam = last_frame->T_c_w_.rotation_matrix() * mp->pt + last_frame->T_c_w_.translation();
    Vector2d p2d = last_frame->toPixel(mp->pt);
    pts.push_back(cv::Point2f(p2d[0], p2d[1]));
    depth.push_back(p_cam[2]);
    count++;
  }
//  auto it = max_element(std::begin(depth), std::end(depth));
//  cout << "max depth: " << *it << endl;
}

void Odometry::activateMapPoints(FramePtr frame) {
  // TODO: select points to add
  for(auto pair:frame->candidates) {
    MapPointPtr m_point = pair.second;
    map->addPoint(m_point);
  }
}

}
