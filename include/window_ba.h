//
// Created by jennings on 2019/2/18.
//

#ifndef STEREO_VO_WINDOW_BA_H
#define STEREO_VO_WINDOW_BA_H

#include "common.h"

using namespace std;

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <boost/format.hpp>
#include "camera.h"

namespace stereo_vo {

// 16x1 error, which is the errors in patch
typedef Eigen::Matrix<double,16,1> Vector16d;
typedef std::shared_ptr<Camera> CamPtr;

// g2o vertex that use sophus::SE3 as pose
class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexSophus() {}

  ~VertexSophus() {}

  bool read(std::istream &is) {}

  bool write(std::ostream &os) const {}

  virtual void setToOriginImpl() {
    _estimate = Sophus::SE3();
  }

  virtual void oplusImpl(const double *update_) {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
    setEstimate(Sophus::SE3::exp(update) * estimate());
  }
};

class EdgeDirectProjection : public g2o::BaseBinaryEdge<16, Vector16d, g2o::VertexSBAPointXYZ, VertexSophus> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeDirectProjection(float *color, cv::Mat &target, CamPtr cam) {
    this->origColor = color;
    this->targetImg = target;
    this->camera = cam;
  }

  ~EdgeDirectProjection() {}

  virtual void computeError() override {
    float fx = camera->fx_;
    float fy = camera->fy_;
    float cx = camera->cx_;
    float cy = camera->cy_;
    // compute projection error ...
    const g2o::VertexSBAPointXYZ* v_point = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
    const VertexSophus* v_pose = static_cast<const VertexSophus*>(_vertices[1]);
    Eigen::Vector3d p_world = v_point->estimate();
    Eigen::Vector3d p_cam = v_pose->estimate() * p_world;
    float u = p_cam[0] * fx / p_cam[2] + cx;
    float v = p_cam[1] * fy / p_cam[2] + cy;
    // check u,v is in the image
    if(u-3<0 || (u+3)>=targetImg.cols
       || v-3<0 || (v+3)>=targetImg.rows) {
      _error = Vector16d::Zero();
      setLevel(1);
    } else {
      int n = 0;
      for(int i = -2; i < 2; i++) {
        for(int j = -2; j < 2; j++) {
          float pixel = GetPixelValue(targetImg, u+i, v+j);
          _error[n] = origColor[n] - pixel;
          n++;
        }
      }
    }
  }

  void linearizeOplus() override {
    float fx = camera->fx_;
    float fy = camera->fy_;
    float cx = camera->cx_;
    float cy = camera->cy_;
    if(level() == 1) {
      _jacobianOplusXi = Eigen::Matrix<double,16,3>::Zero(); // w.r.t point
      _jacobianOplusXj = Eigen::Matrix<double,16,6>::Zero(); // w.r.t pose
    } else {
      const g2o::VertexSBAPointXYZ* v_point = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
      const VertexSophus* v_pose = static_cast<const VertexSophus*>(_vertices[1]);
      Eigen::Vector3d p_cam = v_pose->estimate() * v_point->estimate();
      double x = p_cam[0];
      double y = p_cam[1];
      double z = p_cam[2];
      double invz = 1.0 / z;
      double invz_2 = invz * invz;
      Eigen::Matrix<double,2,3> J_uv_pcam;
      J_uv_pcam << fx * invz, 0, -fx * x * invz_2,
              0, fy * invz, -fy * y * invz_2;
      Eigen::Matrix<double,3,6> J_pcam_ksai;
      // In g2o, lie algebra is [so(3), t]
      // But in sophus, [t, so(3)]
      J_pcam_ksai <<  1, 0, 0, 0, z, -y,
              0, 1, 0, -z, 0, x,
              0, 0, 1, y, -x, 0;
      Eigen::Matrix<double,1,2> img_grad = Eigen::Matrix<double,1,2>::Zero();
      int n = 0;
      float u = x * fx * invz + cx;
      float v = y * fy * invz + cy;
      Eigen::Matrix<double,3,3> R = v_pose->estimate().rotation_matrix();
      for(int i = -2; i < 2; i++) {
        for(int j = -2; j < 2; j++) {
          img_grad(0, 0) = (GetPixelValue(targetImg, u+i+1, v+j) - GetPixelValue(targetImg, u+i-1, v+j))/2;
          img_grad(0, 1) = (GetPixelValue(targetImg, u+i, v+j+1) - GetPixelValue(targetImg, u+i, v+j-1))/2;
          _jacobianOplusXi.block<1,3>(n, 0) = -img_grad * J_uv_pcam * R;
          _jacobianOplusXj.block<1,6>(n, 0) = -img_grad * J_uv_pcam * J_pcam_ksai;
          n++;
        }
      }
    }
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

private:
  cv::Mat targetImg;  // the target image
  float *origColor = nullptr;   // 16 floats, the color of this point
  CamPtr camera;
};

class WindowDirectBA {
public:
  WindowDirectBA(CamPtr cam):cam_(cam) {};
  ~WindowDirectBA() {};
  void optimize(VecSE3& poses, VecVec3d& points, vector<cv::Mat>& images, vector<float *>& color);
private:
  CamPtr cam_;
};


}


#endif //STEREO_VO_WINDOW_BA_H
