//
// Created by jennings on 2019/1/17.
//
#include "local_map.h"

namespace stereo_vo {

uint32_t LocalMap::addPoint(Vector3d p, uint32_t frame_id) {
  static uint32_t factory_id = 0;
   map_points.insert(std::make_pair(
           factory_id,
           MapPointPtr(new MapPoint(p, frame_id, factory_id))
           ));
  //map_points.push_back(MapPoint(p, frame_id, factory_id));
  return factory_id++;
}

void LocalMap::clear() {
  if(!map_points.empty()) {
    map_points.clear();
  }
}

void LocalMap::projectToFrame(vector<MapPointPtr>& mpts, vector<float *>& patches, std::shared_ptr<Frame> ref_frame) {
  vector<Vector3d> pts;
  for(auto& pair:map_points) {
    MapPointPtr point = pair.second;
//  for(auto& point:map_points) {
    Vector2d uv = ref_frame->toPixel(point->pt);
    if(ref_frame->isInside(uv[0], uv[1], 2)) {
      mpts.push_back(point);
      pts.push_back(point->pt);
    }
  }
  ref_frame->getKeypointColors(pts, patches);
}

void LocalMap::getAllPoints(vector<Vector3d>& points) {
  for(auto pair:map_points) {
    points.push_back(pair.second->pt);
  }
//  for(auto mp:map_points)
//    points.push_back(mp.pt);
}

void LocalMap::removePoints(const vector<MapPointPtr>& mpts) {
  for(auto p:mpts) {
    map_points.erase(p->id);
  }
}

int LocalMap::countPoints() {
  return map_points.size();
}

}
