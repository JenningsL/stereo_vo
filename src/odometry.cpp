//
// Created by jennings on 2019/1/17.
//

#include "odometry.h"
#include <opencv2/highgui/highgui.hpp>

namespace stereo_vo {

Odometry::Odometry() : state_(INIT) {
  local_map_ = std::unique_ptr<LocalMap>(new LocalMap());
  cam_ = std::shared_ptr<Camera>(new Camera(707.0912, 707.0912, 601.8873, 183.1104));
}

std::shared_ptr<Frame> Odometry::readImage(string l_img, string r_img) {
  Mat left_img = cv::imread(l_img);
  Mat right_img = cv::imread(r_img);
  std::shared_ptr<Frame> frame = Frame::CreateFrame(left_img, right_img, cam_);
  if(state_ == INIT) {
    extractFeature(frame);
    state_ = OK;
  } else if(state_ == OK) {
    // tracking_pts_ may decrease
    trackFeature(frame);
    SE3 T = estimatePose();
    frame->setPose(T);
    if(tracking_pts_.size() < MIN_CNT)
      extractFeature(frame);
  }
  last_frame_ = frame;
  return frame;
};

list<cv::Point2f> Odometry::getTrackingPts() {
  return tracking_pts_;
}

void Odometry::trackFeature(std::shared_ptr<Frame> frame) {
  vector<cv::Point2f> prev_pts;
  vector<cv::Point2f> cur_pts;
  for(auto pt:tracking_pts_) {
    prev_pts.push_back(pt);
  }
  vector<unsigned char> status;
  vector<float> error;
  cv::calcOpticalFlowPyrLK(last_frame_->left_img, frame->left_img, prev_pts, cur_pts, status, error);
  vector<uint32_t> tracked_ids;
  int i = 0;
  vector<cv::Point2f> match_prev_pts;
  vector<cv::Point2f> match_next_pts;
  for(auto iter=tracking_pts_.begin(); iter!=tracking_pts_.end(); i++) {
    if(status[i] == 0 || last_frame_->getDepth(*iter) < 0
      || std::fabs(last_frame_->getDepth(*iter) - frame->getDepth(cur_pts[i])) > 1) {
      // lost point
      iter = tracking_pts_.erase(iter);
      continue;
    } else {
      *iter = cur_pts[i];
      match_prev_pts.push_back(prev_pts[i]);
      match_next_pts.push_back(cur_pts[i]);
      iter++;
      tracked_ids.push_back(map_point_ids_[i]);
    }
  }
  map_point_ids_ = tracked_ids;

  // plot the tracking feature points
//  int rows = frame->left_img.rows;
//  int cols = frame->left_img.cols;
//  cv::Mat img_show ( frame->left_img.rows*2, frame->left_img.cols, CV_8UC3 );
//  last_frame_->left_img.copyTo ( img_show ( cv::Rect ( 0,0,frame->left_img.cols, frame->left_img.rows ) ) );
//  frame->left_img.copyTo ( img_show ( cv::Rect ( 0,frame->left_img.rows,frame->left_img.cols, frame->left_img.rows ) ) );
//  for (int j = 0; j < match_prev_pts.size();j++)
//  {
//    float b = 255*float ( rand() ) /RAND_MAX;
//    float g = 255*float ( rand() ) /RAND_MAX;
//    float r = 255*float ( rand() ) /RAND_MAX;
//    cv::circle ( img_show, cv::Point2d (match_prev_pts[j].x, match_prev_pts[j].y ), 2, cv::Scalar ( b,g,r ), 2 );
//    cv::circle ( img_show, cv::Point2d (match_next_pts[j].x, match_next_pts[j].y + rows ), 2, cv::Scalar ( b,g,r ), 2 );
//    cv::line ( img_show, cv::Point2d (match_prev_pts[j].x, match_prev_pts[j].y ), cv::Point2d (match_next_pts[j].x, match_next_pts[j].y + rows), cv::Scalar ( b,g,r ), 1 );
//  }
//  cv::imshow ( "result", img_show );
//  cv::waitKey ( 0 );
}

void Odometry::extractFeature(std::shared_ptr<Frame> frame) {
  vector<cv::Point2f> kps;
  cv::Mat gray;
  cvtColor(frame->left_img, gray, CV_BGR2GRAY);
  cv::goodFeaturesToTrack(gray, kps, MAX_CNT - tracking_pts_.size(), 0.01, MIN_DIST);
  for(auto kp:kps) {
    cv::Point3f map_point = frame->toWorldCoord(kp);
    if(map_point.z <= 0) {
      continue;
    }
    // add more map points
    uint32_t mpid = local_map_->addPoint(map_point);
    map_point_ids_.push_back(mpid);
    tracking_pts_.push_back(kp);
  }
}

SE3 Odometry::estimatePose() {
  // construct the 3d 2d observations
  vector<cv::Point3f> pts3d;
  vector<cv::Point2f> pts2d;
  for (auto pt2d:tracking_pts_) {
    pts2d.push_back(pt2d);
  }
  for (auto mpid:map_point_ids_) {
    pts3d.push_back(local_map_->getPointById(mpid));
  }
  Mat K = ( cv::Mat_<double> ( 3,3 ) <<
    cam_->fx_, 0, cam_->cx_,
    0, cam_->fy_, cam_->cy_,
    0,0,1
  );
  Mat rvec, tvec, inliers;
  bool success = cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
  int num_inliers_ = inliers.rows;
  cout<<"pnp inliers: "<<num_inliers_<<endl;
  cout << "rotation: " << rvec << endl;
  cout << "translation: " << tvec << endl;
  SE3 T_c_w_estimated_ = SE3 (
          SO3 (rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)),
          Vector3d (tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
  );
  return T_c_w_estimated_;
}

}