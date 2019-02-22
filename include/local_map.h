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
  void projectToFrame(vector<MapPointPtr>& mpts, vector<float *>& patches, std::shared_ptr<Frame> ref_frame);
  void getAllPoints(vector<Vector3d>& points);
  void removeFrame(uint32_t fid);
  void removePoints(const vector<MapPointPtr>& mpts);
  int countPoints();
  void clear();
private:
  unordered_map<uint32_t, MapPointPtr> immature_points;
  unordered_map<uint32_t, MapPointPtr> map_points;
};

}

#endif //STEREO_VO_LOCAL_MAP_H
