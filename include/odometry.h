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
#include "local_map.h"
#include "window_ba.h"
#include "direct_pose_estimate.h"
namespace stereo_vo {

enum VOState {INIT, OK, LOST};

class Odometry {
public:
  Odometry();
//  ~Odometry();
  std::shared_ptr<Frame> addFrame(string l_img, string r_img);
  list<cv::Point2f> getProjectedPoints();
  std::unique_ptr<LocalMap> local_map_;
private:
  int MIN_CNT = 50;
  int MAX_CNT = 2000;
  int MIN_DIST = 10;
  void extractFeature(std::shared_ptr<Frame> frame);
  void trackFeature(std::shared_ptr<Frame> frame);
  void trackNewFrame(std::shared_ptr<Frame> frame);
  void optimizeWindow();
  VOState state_;
  std::shared_ptr<Frame> last_frame_;
  std::shared_ptr<Frame> ref_frame_;
  std::shared_ptr<Camera> cam_;
  vector<std::shared_ptr<Frame>> active_frames_;
};

}


#endif //STEREO_VO_ODEMETRY_H
