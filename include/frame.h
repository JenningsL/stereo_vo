//
// Created by jennings on 2019/1/16.
//

#ifndef STEREO_VO_FRAME_H
#define STEREO_VO_FRAME_H

#include "common.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "camera.h"

namespace stereo_vo {

class Frame {
public:
  Frame(Mat left, Mat right, std::shared_ptr<Camera> cam, uint32_t id);
  static std::shared_ptr<Frame> CreateFrame(Mat left, Mat right, std::shared_ptr<Camera> cam);
  void calDepth();
  double getDepth(Vector2d pt);
  void setPose(const SE3 T_c_w);
  Vector3d toWorldCoord(Vector2d pt);
  Vector2d toPixel(Vector3d pw);
  void getKeypointColors(const vector<Vector3d>& pws, vector<float*>& colors);
  bool isInside(int x, int y, int border);
  void extractFeaturePoints(VecVec2d& features, int nPoints, int border);
  void setReference(bool ref) {is_ref = ref;}
  bool isReference() {return is_ref;}
  uint32_t getId();
  vector<uint32_t> getOutliner(vector<uint32_t>& pt_ids, vector<Vector3d>& points, vector<float *>& colors);
  Mat left_img;
  Mat right_img;
  Mat disp;
  SE3 T_c_w_;
  std::shared_ptr<Camera> cam_;
private:
  uint32_t id_;
  bool is_ref;
};

} // namespace stereo_vo

#endif //STEREO_VO_FRAME_H
