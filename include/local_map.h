//
// Created by jennings on 2019/1/17.
//

#ifndef STEREO_VO_LOCAL_MAP_H
#define STEREO_VO_LOCAL_MAP_H

#include "common.h"
namespace stereo_vo {

class LocalMap {
public:
  uint32_t addPoint(Vector3d p, uint32_t frame_id);
  void updatePoint(uint32_t id, Vector3d p);
  Vector3d getPointById(uint32_t id);
  void getAllPoints(vector<uint32_t>& ids, vector<Vector3d>& points);
  void removeFrame(uint32_t fid);
  void removePoints(const vector<uint32_t>& ids);
  int countPoints();
  void clear();
private:
  std::unordered_map<uint32_t, Vector3d> points_;
  std::unordered_map<uint32_t, vector<uint32_t>> frame_pts_;
};

}

#endif //STEREO_VO_LOCAL_MAP_H
