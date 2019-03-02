//
// Created by jennings on 2019/1/17.
//

#include "odometry.h"
#include "coarse_track_g2o.h"
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <iterator>
#include <algorithm>
#include <random>
#include "boost/range/irange.hpp"
#include "boost/range/algorithm_ext/push_back.hpp"

namespace stereo_vo {

Odometry::Odometry(CameraPtr cam) : state_(INIT), cam_(cam) {
  map = std::shared_ptr<Map>(new Map());
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
    frame->selectCandidates();
    activateMapPoints(frame, true);
    key_frames.push_back(frame);
    next_keyframe = frame;
    state_ = OK;
  } else if(state_ == OK) {
    // tracking using lastest keyframe
    trackNewFrame(frame);
    // FIXME: update depth is not working
    /*
    auto begin = std::chrono::steady_clock::now();
    key_frames.back()->updateCandidates(frame);
    auto end = std::chrono::steady_clock::now();
    cout << "Update depth time consumed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << endl;
    */
    if(isNewKeyFrame(frame)) {
      cout << "New keyframe: " << frame->getId() << endl;
      frame->selectCandidates();
      frame->is_keyframe = true;
      key_frames.push_back(frame);
//      auto begin = std::chrono::steady_clock::now();
//      optimizeWindow();
//      auto end = std::chrono::steady_clock::now();
//      cout << "Local BA time consumed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << endl;
      activateMapPoints(frame, false);
      // update next_keyframe
//      frame->is_keyframe = true;
//      next_keyframe = frame;
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
  // determined based on mean square optical flow
  FramePtr last_keyframe = key_frames.back();
  VecVec2d px_1;
  vector<double> depth;
  vector<MapPointPtr> mpts;
  map->projectToFrame(px_1, depth, mpts, last_keyframe, 2000);
  Vector2d p2;
  Vector2d displacement;
  float mean_square_opt_flow = 0;
  int num = 0;
  for(int i = 0; i < mpts.size(); i++) {
    p2 = frame->toPixel(mpts[i]->pt);
    if(!frame->isInside(p2[0], p2[1], 0))
      continue;
    displacement = p2-px_1[i];
    mean_square_opt_flow += displacement.norm();
    num++;
  }
  cout << "mean_square_opt_flow: " << mean_square_opt_flow / num << endl;
  return mean_square_opt_flow / num > 20;
}

void Odometry::trackNewFrame(FramePtr frame) {
  vector<MapPointPtr> mpts;
  FramePtr last_frame = all_frames.back();
  //FramePtr last_frame = key_frames.back();
  //FramePtr last_frame = next_keyframe;

  cout << "Frame gap: " << frame->getId() - last_frame->getId() << endl;

  VecVec2d px_ref;
  vector<double> depth_ref;
  Sophus::SE3 T21;
  cout << "last_delta_: " << last_delta_.matrix() << endl;
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
  cout << "T21: " << T21.matrix() << endl;
}

void Odometry::optimizeWindow() {
  const int window_size = 3;
  if(key_frames.size() < 2) {return;}
  int ref_i = std::max(int(key_frames.size()-window_size), 0);
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
  for(int i = key_frames.size()-1; i >= ref_i; i--) {
    auto frame = key_frames[i];
    poses.insert(poses.begin(), frame->T_c_w_);
    images.insert(images.begin(), frame->left_img);
    opt_frames.insert(opt_frames.begin(), frame);
  }

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
//  cout << "max depth: " << *it << endl;parameter is empty
}

void Odometry::activateMapPoints(FramePtr frame, bool is_first) {
  if(is_first) {
    for(auto& pair:frame->candidates) {map->addPoint(pair.second);}
    return;
  }

  vector<MapPointPtr> mpts;
  VecVec2d projections;
  vector<double> depths;
  map->projectToFrame(projections, depths, mpts, frame, -1);

  cv::Mat exist_points(projections.size(), 2, CV_32F);
  for(int i = 0; i < projections.size(); i++) {
    exist_points.at<float>(i,0) = projections[i][0];
    exist_points.at<float>(i,1) = projections[i][1];
  }
  cv::flann::Index Kdtree;
  Kdtree.build(exist_points,cv::flann::KDTreeIndexParams(1),cvflann::FLANN_DIST_EUCLIDEAN);

  vector<MapPointPtr> candidates;
  Mat candidate_2d;
  for(auto pair:frame->candidates) {
    MapPointPtr m_point = pair.second;
    // if(pair.second->sucess_times < frame->getId() - key_frames.back()->getId() - 1)
    //if(pair.second->sucess_times < 1)
    //  continue;
    Vector2d uv = frame->toPixel(m_point->pt);
    candidates.push_back(m_point);
    Mat row = Mat(1, 2, CV_32F);
    row.at<float>(0,0) = uv[0];
    row.at<float>(0,1) = uv[1];
    candidate_2d.push_back(row);
  }

  Mat matches(candidate_2d.size(), 1, CV_32S);
  Mat distances(candidate_2d.size(), 1, CV_32F);
  Kdtree.knnSearch(candidate_2d, matches, distances, 1, cv::flann::SearchParams(-1));
  // select top-k points that are farthest to the existing points
  std::vector<int> indices;
  boost::push_back(indices, boost::irange(0, (int)candidates.size()));
  std::sort(indices.begin(), indices.end(), [&distances](const int& a, const int& b){return distances.at<float>(a) > distances.at<float>(b);});
  int count = 0;
  for(int i = 0; i < candidates.size(); i++) {
    if(count > 2000)
      break;
    if(distances.at<float>(indices[i]) < 9)
      continue;
    // cout << "distance: " << sqrt(distances.at<float>(indices[i]));
    map->addPoint(candidates[indices[i]]);
    count++;
  }
  cout << "Add " << count << " new map points" << endl;
}

}
