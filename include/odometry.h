//
// Created by jennings on 2019/1/16.
//

#ifndef STEREO_VO_ODEMETRY_H
#define STEREO_VO_ODEMETRY_H

#include "common.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include "frame.h"
#include "map.h"
#include "window_ba.h"
#include "direct_pose_estimate.h"
#include "map_point.h"
namespace stereo_vo {

enum VOState {INIT, OK, LOST};

class Odometry {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Odometry(CameraPtr cam);
  std::shared_ptr<Frame> addFrame(string l_img, string r_img);
  void getProjectedPoints(vector<cv::Point2f>& pts, vector<float>& depth);
  std::shared_ptr<Map> map;
  vector<std::shared_ptr<Frame>> all_frames;
  vector<std::shared_ptr<Frame>> key_frames;
  std::shared_ptr<Camera> cam_;
private:
  void activateMapPoints(FramePtr frame, bool is_first);
  bool isNewKeyFrame(FramePtr frame);
  void trackNewFrame(FramePtr frame);
  void optimizeWindow();
  VOState state_;
  std::shared_ptr<Frame> next_keyframe;
  SE3 last_delta_; // motion from k-2 to k-1
};

}


#endif //STEREO_VO_ODEMETRY_H
