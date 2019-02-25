//
// Created by jennings on 2019/1/17.
//
#include "map.h"

namespace stereo_vo {

uint32_t Map::addPoint(MapPointPtr map_point) {
  map_points.insert(std::make_pair(map_point->id, map_point));
  local_points.insert(std::make_pair(map_point->id, map_point));
  return map_point->id;
}

void Map::clear() {
  if(!map_points.empty()) {
    map_points.clear();
    local_points.clear();
  }
}

void Map::projectToFrame(VecVec2d& projections, vector<MapPointPtr>& m_points, std::shared_ptr<Frame> ref_frame) {
  for(auto& pair:local_points) {
    MapPointPtr point = pair.second;
    Vector2d uv = ref_frame->toPixel(point->pt);
    if(ref_frame->isInside(uv[0], uv[1], 2)) {
      projections.push_back(uv);
      m_points.push_back(point);
    }
  }
}

void Map::getAllPoints(vector<MapPointPtr>& points) {
  for(auto pair:map_points) {
    points.push_back(pair.second);
  }
}

void Map::getLocalPoints(vector<MapPointPtr>& points) {
  for(auto pair:local_points) {
    points.push_back(pair.second);
  }
}

void Map::removeInvisibleMapPoints(FramePtr frame) {
  vector<MapPointPtr> to_remove;
  for(auto pair:local_points) {
    Vector2d uv = frame->toPixel(pair.second->pt);
    if(!frame->isInside(uv[0], uv[1], 0)) {
       to_remove.push_back(pair.second);
    }
  }
  // removePoints(to_remove);
  for(auto p:to_remove)
    local_points.erase(p->id);
}

void Map::removeOutlier(FramePtr ref, FramePtr cur) {
  vector<MapPointPtr> mpts;
  VecVec2d projections;
  VecVec3d points;
  vector<float *> colors;
  projectToFrame(projections, mpts, ref);
  for(auto& mpt:mpts) {
    points.push_back(mpt->pt);
  }
  ref->getKeypointColors(points, colors);
  vector<MapPointPtr> outliners = cur->getOutlier(mpts, colors);
  cout << "outliners: " << outliners.size() << endl;
  for(auto p:outliners) {
    map_points.erase(p->id);
    local_points.erase(p->id);
  }
  for (auto &c: colors) delete[] c;
}

}
