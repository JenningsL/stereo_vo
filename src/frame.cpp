#include "frame.h"
#include <opencv2/highgui/highgui.hpp>
#include "depth_filter.h"

namespace stereo_vo {
using namespace cv;

Frame::Frame(Mat left, Mat right, std::shared_ptr<Camera> cam, uint32_t id)
  :left_img(left), right_img(right), cam_(cam), id_(id) {
  T_c_w_ = SE3(Eigen::Matrix3d::Identity(), Vector3d(0,0,0));
  is_keyframe = false;
}

std::shared_ptr<Frame> Frame::CreateFrame(Mat left, Mat right, std::shared_ptr<Camera> cam) {
  static uint32_t factory_id = 0;
  std::shared_ptr<Frame> frame(new Frame(left, right, cam, factory_id++));
  frame->calDepth();
  return frame;
}

void Frame::calDepth() {
  Mat disp8;

  cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create();
  bm->compute(left_img, right_img,disp);
  disp.convertTo(disp, CV_32F, 1.0/16);
  normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

//  imshow("Left", left_img);
//  imshow("Right", right_img);
//  imshow("Depth map", disp8);
//  waitKey(0);
}

void Frame::setPose(const SE3 T_c_w) {
  T_c_w_ = T_c_w;
}

double Frame::getDepth(const Vector2d& pt) {
  // return right_img.ptr<ushort> ( cvRound ( pt[1] ) ) [ cvRound ( pt[0] ) ] / 1000.0;
  uint32_t x = cvRound(pt[0]);
  uint32_t y = cvRound(pt[1]);
  // scale depth for better visualization
  return cam_->fx_ * cam_->baseline / disp.at<float>(y,x) / 100;
}

/**
 * Convert 2d point to 3d point in world coord,
 * according to the pose
 * @param pt
 * @return
 */
Vector3d Frame::toWorldCoord(const Vector2d& pt) {
  double d = getDepth(pt);
  if(d <= 0)
    return Vector3d({0,0,0});
  return cam_->pixel2world(pt, T_c_w_, d);
}

Vector2d Frame::toPixel(const Vector3d& pw) {
  return cam_->world2pixel(pw, T_c_w_);
}

void Frame::getKeypointColors(const VecVec3d& pws, vector<float*>& colors) {
  for(auto& pw:pws) {
    if(std::isnan(pw[0])) {
      throw std::invalid_argument( "Invalid 3d point!" );
    }
    Vector2d uv = toPixel(pw);
    int u = uv[0];
    int v = uv[1];
    if(!isInside(u, v, 2)) {
      throw std::invalid_argument( "Try to get image patch outsize image!" );
    }
    float* color = new float[16];
    int n = 0;
    for(int i = -2; i < 2; i++) {
      for(int j = -2; j < 2; j++) {
        color[n++] = GetPixelValue(left_img, u+i, v+j);
      }
    }
    colors.push_back(color);
  }
}

bool Frame::isInside(int x, int y, int border) {
  return (x >= border && x < left_img.cols - border
          && y >= border && y < left_img.rows - border);
}

void Frame::selectCandidates() {
  int nPoints = 4000;
  int border = 50;

  Mat grad;
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  /// Generate grad_x and grad_y
  Mat grad_x, grad_y;
  Mat abs_grad_x, abs_grad_y;

  /// Gradient X
  //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
  Sobel( left_img, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
  convertScaleAbs( grad_x, abs_grad_x );

  /// Gradient Y
  //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
  Sobel( left_img, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
  convertScaleAbs( grad_y, abs_grad_y );

  /// Total Gradient (approximate)
  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
  Mat grad_norm;
  normalize(grad, grad_norm, 0, 1, NORM_MINMAX, ddepth);

//  Mat selected_pts(cv::Size(left_img.cols, left_img.rows), CV_8U);
//  selected_pts = 0;

  const int b_num = 64;
  int b_w = left_img.cols / b_num;
  int b_h = left_img.rows / b_num;
  Mat count(cv::Size(b_num, b_num), CV_8U);
  count = 0;
  for (int x = 0; x < left_img.cols; x++) {
    for(int y = 0; y < left_img.rows; y++) {
      if(grad_norm.at<short>(y, x) < 0.05)
        continue;
      // don't pick pixels close to boarder
      if(!isInside(x, y, border))
        continue;

      int b_x = std::min(x/b_w, b_num-1);
      int b_y = std::min(y/b_h, b_num-1);
      if(count.at<int8_t>(b_x, b_y) >= 4)
        continue;

      Vector3d p_3d = toWorldCoord(Vector2d({x, y}));
      float depth = p_3d[2];
      if(depth <= 0 || std::isnan(depth) || std::isinf(depth)) {
        continue;
      }

      if(rand() / double(RAND_MAX) < 0.5)
        continue;
      MapPointPtr map_point = MapPoint::create(p_3d, id_);
      candidates.insert(std::make_pair(map_point->id, map_point));
      count.at<int8_t>(b_x, b_y) += 1;

      // selected_pts.at<int8_t>(y, x) = 255;
    }
  }
//  imshow("edge", selected_pts);
  // waitKey(0);
}

uint32_t Frame::getId() {
  return id_;
}

vector<MapPointPtr> Frame::getOutlier(vector<MapPointPtr>& points, vector<float *>& colors) {
  vector<MapPointPtr> outliner;
  for(int i = 0; i < points.size(); i++) {
    Vector2d uv = toPixel(points[i]->pt);
    int u = uv[0];
    int v = uv[1];
    if(!isInside(u, v, 2))
      continue;
    float err = 0;
    int n = 0;
    for(int x = -2; x < 2; x++) {
      for(int y = -2; y < 2; y++) {
        err += std::fabs(GetPixelValue(left_img, u+x, v+y) - colors[i][n++]);
      }
    }
    if(err > 640) {
      outliner.push_back(points[i]);
    }
  }
  return outliner;
}

void Frame::updateCandidates(FramePtr frame) {
  vector<float> depth;
  vector<float> depth_cov;
  VecVec2d pts_2d;
  vector<uint32_t> ids;
  for(auto& pair:candidates) {
    Vector2d uv = toPixel(pair.second->pt);
    // if(!isInside(uv[0], uv[1], 0)) {continue;}
    pts_2d.push_back(uv);
    depth.push_back(pair.second->pt[2]);
    depth_cov.push_back(pair.second->conv);
    ids.push_back(pair.first);
  }
  vector<bool> success;
  depth_filter::update(cam_, left_img, frame->left_img, frame->T_c_w_*T_c_w_.inverse(), pts_2d, depth, depth_cov, success);
  float total_conv = 0.0;
  int success_match = 0;
  for(int i = 0; i < ids.size(); i++) {
//     if(!success[i] || depth_cov[i] > depth_filter::max_cov) {
    if(!success[i]) {
      // remove outlier
      candidates.erase(ids[i]);
      continue;
    }
    candidates[ids[i]]->sucess_times++;
    success_match++;
    total_conv += depth_cov[i];
    candidates[ids[i]]->pt[2] = depth[i];
    candidates[ids[i]]->conv = depth_cov[i];
  }
  cout << "Success_match: " << success_match << endl;
  cout << "Average convariance: " << total_conv / success_match << endl;
}

}