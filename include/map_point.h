//
// Created by jennings on 2019/2/21.
//

#ifndef STEREO_VO_MAP_POINT_H
#define STEREO_VO_MAP_POINT_H

#include "common.h"

namespace stereo_vo {

class MapPoint {
public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MapPoint(Vector3d p, uint32_t host, uint32_t id)
    :pt(p), host_(host), id(id), conv(0.0001), sucess_times(0) {};
  static std::shared_ptr<MapPoint> create(Vector3d p, uint32_t frame_id);
  void update(const Vector3d& p);
  Vector3d pt;
  uint32_t id;
  float conv; // covariance
  uint32_t sucess_times;
private:
  uint32_t host_;
};

typedef std::shared_ptr<MapPoint> MapPointPtr;

}


#endif //STEREO_VO_MAP_POINT_H
