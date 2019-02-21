#include "frame.h"
#include <opencv2/highgui/highgui.hpp>

namespace stereo_vo {
using namespace cv;

Frame::Frame(Mat left, Mat right, std::shared_ptr<Camera> cam, uint32_t id)
  :left_img(left), right_img(right), cam_(cam), id_(id) {
  T_c_w_ = SE3(Eigen::Matrix3d::Identity(), Vector3d(0,0,0));
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

double Frame::getDepth(Vector2d pt) {
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
Vector3d Frame::toWorldCoord(Vector2d pt) {
  double d = getDepth(pt);
  if(d <= 0)
    return Vector3d({0,0,0});
  return cam_->pixel2world(pt, T_c_w_, d);
}

Vector2d Frame::toPixel(Vector3d pw) {
  return cam_->world2pixel(pw, T_c_w_);
}

std::shared_ptr<Frame> Frame::CreateFrame(Mat left, Mat right, std::shared_ptr<Camera> cam) {
  static uint32_t factory_id = 0;
  std::shared_ptr<Frame> frame(new Frame(left, right, cam, factory_id++));
  frame->calDepth();
  return frame;
}

void Frame::getKeypointColors(const vector<Vector3d>& pws, vector<float*>& colors) {
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

void Frame::extractFeaturePoints(VecVec2d& features, int nPoints, int border) {
  vector<cv::KeyPoint> keypoints;
  cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
  detector->detect (left_img, keypoints);
  std::random_shuffle(keypoints.begin(), keypoints.end());
  Vector2d pixel;
  for (int i = 0; i < nPoints; i++) {
    int x = keypoints[i].pt.x;
    int y = keypoints[i].pt.y;
    // don't pick pixels close to boarder
    if(!isInside(x, y, border))
      continue;
    pixel = Vector2d({keypoints[i].pt.x, keypoints[i].pt.y});
    features.push_back(pixel);
  }
}

uint32_t Frame::getId() {
  return id_;
}

vector<uint32_t> Frame::getOutliner(vector<uint32_t>& pt_ids, vector<Vector3d>& points, vector<float *>& colors) {
  vector<uint32_t> outliner;
  for(int i = 0; i < pt_ids.size(); i++) {
    Vector2d uv = toPixel(points[i]);
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
      outliner.push_back(pt_ids[i]);
    }
  }
  return outliner;
}

}