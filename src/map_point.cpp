//
// Created by jennings on 2019/2/21.
//

#include "map_point.h"

namespace stereo_vo {

void MapPoint::update(const Vector3d& p) {
  pt[0] = p[0];
  pt[1] = p[1];
  pt[2] = p[2];
}

std::shared_ptr<MapPoint> MapPoint::create(Vector3d p, uint32_t frame_id) {
  static uint32_t factory_id = 0;
  return std::shared_ptr<MapPoint>(new MapPoint(p, frame_id, factory_id++));
}

}