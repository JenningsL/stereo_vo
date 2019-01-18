//
// Created by jennings on 2019/1/17.
//
#include "local_map.h"

namespace stereo_vo {

uint32_t LocalMap::addPoint(cv::Point3f p) {
  static uint32_t factory_id = 0;
  points_[factory_id] = p;
  return factory_id++;
}

cv::Point3f LocalMap::getPointById(uint32_t id) {
  return points_[id];
}

}
