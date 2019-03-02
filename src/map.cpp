//
// Created by jennings on 2019/1/17.
//
#include "map.h"

namespace stereo_vo {

uint32_t Map::addPoint(MapPointPtr map_point) {
  map_points.insert({map_point->id, map_point});
  local_points.insert({map_point->id, map_point});
  return map_point->id;
}

void Map::clear() {
  if(!map_points.empty()) {
    map_points.clear();
    local_points.clear();
  }
}

void Map::projectToFrame(VecVec2d& projections, vector<double>& depth, vector<MapPointPtr>& m_points,
        std::shared_ptr<Frame> ref_frame, int max_num=-1) {
  VecVec2d all_proj;
  vector<float> all_depth;
  vector<MapPointPtr> all_mp;
  vector<int> indices;
  for(auto& pair:local_points) {
    MapPointPtr point = pair.second;
    Vector2d uv = ref_frame->toPixel(point->pt);
    Vector3d p_cam;
    if(ref_frame->isInside(uv[0], uv[1], 5)) {
      all_proj.push_back(uv);
      all_mp.push_back(point);
      p_cam = ref_frame->T_c_w_.rotation_matrix() * point->pt + ref_frame->T_c_w_.translation();
      all_depth.push_back(p_cam[2]);
      indices.push_back(indices.size());
    }
  }
  if(max_num > 0 && indices.size() > max_num)
    std::random_shuffle(indices.begin(), indices.end());
  for(int i = 0; i < indices.size(); i++) {
    if(max_num > 0 && i >= max_num)
      break;
    projections.push_back(all_proj[indices[i]]);
    depth.push_back(all_depth[indices[i]]);
    m_points.push_back(all_mp[indices[i]]);
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
  VecVec2d p1s;
  VecVec3d points;
  vector<double> depths;
  vector<float *> colors;
  projectToFrame(p1s, depths, mpts, ref);
//  for(auto& mpt:mpts) {
//    points.push_back(mpt->pt);
//  }
//  ref->getKeypointColors(points, colors);
//  vector<MapPointPtr> outliners = cur->getOutlier(mpts, colors);
//  cout << "outliners: " << outliners.size() << endl;
//  for(auto p:outliners) {
//    map_points.erase(p->id);
//    local_points.erase(p->id);
//  }
//  for (auto &c: colors) delete[] c;
  Vector2d p2;
  double ncc;
  int outlier_num = 0;
  for(int i = 0; i < mpts.size(); i++) {
    p2 = cur->toPixel(mpts[i]->pt);
    if(!cur->isInside(p2[0], p2[1], 3))
      continue;
    ncc = depth_filter::NCC(ref->left_img, cur->left_img, p1s[i], p2);
    if(ncc < 0.5) {
      map_points.erase(mpts[i]->id);
      local_points.erase(mpts[i]->id);
      outlier_num++;
    }
  }
  cout << "outliners: " << outlier_num << endl;
}

}
