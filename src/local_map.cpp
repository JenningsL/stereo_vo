//
// Created by jennings on 2019/1/17.
//
#include "local_map.h"

namespace stereo_vo {

uint32_t LocalMap::addPoint(MapPointPtr map_point) {
  map_points.insert(std::make_pair(map_point->id, map_point));
  return map_point->id;
}

void LocalMap::clear() {
  if(!map_points.empty()) {
    map_points.clear();
  }
}

void LocalMap::projectToFrame(vector<Vector2d>& projections, vector<MapPointPtr>& m_points, std::shared_ptr<Frame> ref_frame) {
  for(auto& pair:map_points) {
    MapPointPtr point = pair.second;
    Vector2d uv = ref_frame->toPixel(point->pt);
    if(ref_frame->isInside(uv[0], uv[1], 2)) {
      projections.push_back(uv);
      m_points.push_back(point);
    }
  }
}

void LocalMap::getAllPoints(vector<MapPointPtr>& points) {
  for(auto pair:map_points) {
    points.push_back(pair.second);
  }
}

void LocalMap::removeInvisibleMapPoints(FramePtr frame) {
  int size_old = map_points.size();
  vector<MapPointPtr> to_remove;
  for(auto pair:map_points) {
    Vector2d uv = frame->toPixel(pair.second->pt);
    if(!frame->isInside(uv[0], uv[1], 0)) {
      to_remove.push_back(pair.second);
    }
  }
  removePoints(to_remove);

  // cout << "Remove invisibale points: " << map_points.size() - size_old << endl;
  cout << "Points: " << map_points.size() << endl;
}

void LocalMap::removeOutlier(FramePtr ref, FramePtr cur) {
  vector<MapPointPtr> mpts;
  vector<Vector2d> projections;
  vector<Vector3d> points;
  vector<float *> colors;
  projectToFrame(projections, mpts, ref);
  for(auto& mpt:mpts) {
    points.push_back(mpt->pt);
  }
  ref->getKeypointColors(points, colors);
  vector<MapPointPtr> outliners = cur->getOutlier(mpts, colors);
  cout << "outliners: " << outliners.size() << endl;
  removePoints(outliners);
  for (auto &c: colors) delete[] c;
}

void LocalMap::removePoints(const vector<MapPointPtr>& mpts) {
  for(auto p:mpts) {
    map_points.erase(p->id);
  }
}

}
