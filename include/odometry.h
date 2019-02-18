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
namespace stereo_vo {

enum VOState {INIT, OK, LOST};

class Odometry {
public:
  Odometry();
//  ~Odometry();
  std::shared_ptr<Frame> readImage(string l_img, string r_img);
  list<cv::Point2f> getTrackingPts();
  std::unique_ptr<LocalMap> local_map_;
  vector<uint32_t> map_point_ids_; // correspond to tracking_pts_
private:
  int MIN_CNT = 150;
  int MAX_CNT = 200;
  int MIN_DIST = 20;
  void extractFeature(std::shared_ptr<Frame> frame);
  void trackFeature(std::shared_ptr<Frame> frame);
  SE3 estimatePose(std::shared_ptr<Frame> frame);
  VOState state_;
  std::shared_ptr<Frame> last_frame_;
  std::shared_ptr<Camera> cam_;
  list<cv::Point2f> tracking_pts_; // tracking 2d points

};

}


#endif //STEREO_VO_ODEMETRY_H
