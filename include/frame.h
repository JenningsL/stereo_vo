//
// Created by jennings on 2019/1/16.
//

#ifndef STEREO_VO_FRAME_H
#define STEREO_VO_FRAME_H

#include "common.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "camera.h"
#include "map_point.h"
#include "depth_filter.h"

namespace stereo_vo {

class Frame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Frame(Mat left, Mat right, std::shared_ptr<Camera> cam, uint32_t id);
  static std::shared_ptr<Frame> CreateFrame(Mat left, Mat right, std::shared_ptr<Camera> cam);
  void calDepth();
  double getDepth(const Vector2d& pt);
  void setPose(const SE3 T_c_w);
  Vector3d toWorldCoord(const Vector2d& pt);
  Vector2d toPixel(const Vector3d& pw);
  void getKeypointColors(const vector<Vector3d>& pws, vector<float*>& colors);
  bool isInside(int x, int y, int border);
  void selectCandidates();
  uint32_t getId();
  vector<MapPointPtr> getOutlier(vector<MapPointPtr>& points, vector<float *>& colors);
  // TODO: use other frame to update candidates map points
  void updateCandidates(std::shared_ptr<Frame> frame);
  Mat left_img;
  Mat right_img;
  Mat disp;
  unordered_map<uint32_t, MapPointPtr> candidates;
  SE3 T_c_w_;
  std::shared_ptr<Camera> cam_;
  bool is_keyframe;
private:
  uint32_t id_;
};

typedef std::shared_ptr<Frame> FramePtr;

} // namespace stereo_vo

#endif //STEREO_VO_FRAME_H
