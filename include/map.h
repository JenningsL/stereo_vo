//
// Created by jennings on 2019/1/17.
//

#ifndef STEREO_VO_LOCAL_MAP_H
#define STEREO_VO_LOCAL_MAP_H

#include "common.h"
#include "map_point.h"
#include "frame.h"

namespace stereo_vo {

class Map {
public:
  Map():map_points(), local_points() {};
  uint32_t addPoint(MapPointPtr map_point);
  /**
   * Project all mature map point to a frame, only those visible in image are kept
   * @param projections : visible projection 2d points
   * @param m_points : corresponding map points
   * @param ref_frame
   */
  void projectToFrame(VecVec2d& projections, vector<double>& depth, vector<MapPointPtr>& m_points,
                       std::shared_ptr<Frame> ref_frame, int max_num);
  void getAllPoints(vector<MapPointPtr>& points);
  void getLocalPoints(vector<MapPointPtr>& points);
  void removeInvisibleMapPoints(FramePtr frame);
  void removeOutlier(FramePtr ref, FramePtr cur);
  void clear();
private:
  unordered_map<uint32_t, MapPointPtr> map_points;
  unordered_map<uint32_t, MapPointPtr> local_points;
};

}

#endif //STEREO_VO_LOCAL_MAP_H
