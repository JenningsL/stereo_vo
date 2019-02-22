//
// Created by jennings on 2019/1/17.
//

#ifndef STEREO_VO_LOCAL_MAP_H
#define STEREO_VO_LOCAL_MAP_H

#include "common.h"
#include "map_point.h"
#include "frame.h"

namespace stereo_vo {

class LocalMap {
public:
  uint32_t addPoint(Vector3d p, uint32_t frame_id);
  /**
   * Project all mature map point to a frame, only those visible in image are kept
   * @param projections : visible projection 2d points
   * @param m_points : corresponding map points
   * @param ref_frame
   */
  void projectToFrame(vector<Vector2d>& projections, vector<MapPointPtr>& m_points, std::shared_ptr<Frame> ref_frame);
  void getAllPoints(vector<Vector3d>& points);
  void removePoints(const vector<MapPointPtr>& mpts);
  void clear();
private:
  unordered_map<uint32_t, MapPointPtr> immature_points;
  unordered_map<uint32_t, MapPointPtr> map_points;
  unordered_map<uint32_t, MapPointPtr> out_points;
};

}

#endif //STEREO_VO_LOCAL_MAP_H
