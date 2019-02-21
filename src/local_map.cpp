//
// Created by jennings on 2019/1/17.
//
#include "local_map.h"

namespace stereo_vo {

uint32_t LocalMap::addPoint(Vector3d p, uint32_t frame_id) {
  static uint32_t factory_id = 0;
  points_[factory_id] = p;
  if(frame_pts_.find(frame_id) == frame_pts_.end()) {
    frame_pts_[frame_id] = vector<uint32_t>();
  }
  frame_pts_[frame_id].push_back(factory_id);
  return factory_id++;
}

void LocalMap::updatePoint(uint32_t id, Vector3d p) {
  points_[id] = p;
}

void LocalMap::clear() {
  points_.clear();
}

void LocalMap::removeFrame(uint32_t fid) {
  for(uint32_t id:frame_pts_[fid]) {
    points_.erase(id);
  }
  frame_pts_.erase(fid);
}

Vector3d LocalMap::getPointById(uint32_t id) {
  return points_[id];
}

void LocalMap::getAllPoints(vector<uint32_t>& ids, vector<Vector3d>& points) {
  for(auto pair:points_) {
    ids.push_back(pair.first);
    points.push_back(pair.second);
  }
}

void LocalMap::removePoints(const vector<uint32_t>& ids) {
  for(uint32_t id:ids) {
    points_.erase(id);
  }
}

int LocalMap::countPoints() {
  return points_.size();
}

}
