//
// Created by jennings on 2019/1/17.
//

#ifndef STEREO_VO_LOCAL_MAP_H
#define STEREO_VO_LOCAL_MAP_H

#include "common.h"
namespace stereo_vo {

class LocalMap {
public:
  uint32_t addPoint(cv::Point3f p);
  cv::Point3f getPointById(uint32_t id);
private:
  std::unordered_map<uint32_t, cv::Point3f> points_;
};

}

#endif //STEREO_VO_LOCAL_MAP_H
