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

}