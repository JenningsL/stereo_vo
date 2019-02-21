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
    if(u-3<0 || (u+2)>=targetImg.cols
       || v-3<0 || (v+2)>=targetImg.rows) {
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

// project a 3d point into an image plane, the error is photometric error
// an unary edge with one vertex SE3Expmap (the pose of camera)
class EdgeSE3ProjectDirect: public g2o::BaseUnaryEdge< 1, double, g2o::VertexSE3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectDirect() {}

  EdgeSE3ProjectDirect ( Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat* image )
          : x_world_ ( point ), fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), image_ ( image )
  {}

  virtual void computeError()
  {
    const g2o::VertexSE3Expmap* v  =static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    Eigen::Vector3d x_local = v->estimate().map ( x_world_ );
    float x = x_local[0]*fx_/x_local[2] + cx_;
    float y = x_local[1]*fy_/x_local[2] + cy_;
    // check x,y is in the image
    if ( x-4<0 || ( x+4 ) >image_->cols || ( y-4 ) <0 || ( y+4 ) >image_->rows )
    {
      _error ( 0,0 ) = 0.0;
      this->setLevel ( 1 );
    }
    else
    {
      _error ( 0,0 ) = getPixelValue ( x,y ) - _measurement;
    }
  }

  // plus in manifold
  virtual void linearizeOplus( )
  {
    if ( level() == 1 )
    {
      _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
      return;
    }
    g2o::VertexSE3Expmap* vtx = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    Eigen::Vector3d xyz_trans = vtx->estimate().map ( x_world_ );   // q in book

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double invz = 1.0/xyz_trans[2];
    double invz_2 = invz*invz;

    float u = x*fx_*invz + cx_;
    float v = y*fy_*invz + cy_;

    // jacobian from se3 to u,v
    // NOTE that in g2o the Lie algebra is (\omega, \epsilon), where \omega is so(3) and \epsilon the translation
    Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

    jacobian_uv_ksai ( 0,0 ) = - x*y*invz_2 *fx_;
    jacobian_uv_ksai ( 0,1 ) = ( 1+ ( x*x*invz_2 ) ) *fx_;
    jacobian_uv_ksai ( 0,2 ) = - y*invz *fx_;
    jacobian_uv_ksai ( 0,3 ) = invz *fx_;
    jacobian_uv_ksai ( 0,4 ) = 0;
    jacobian_uv_ksai ( 0,5 ) = -x*invz_2 *fx_;

    jacobian_uv_ksai ( 1,0 ) = - ( 1+y*y*invz_2 ) *fy_;
    jacobian_uv_ksai ( 1,1 ) = x*y*invz_2 *fy_;
    jacobian_uv_ksai ( 1,2 ) = x*invz *fy_;
    jacobian_uv_ksai ( 1,3 ) = 0;
    jacobian_uv_ksai ( 1,4 ) = invz *fy_;
    jacobian_uv_ksai ( 1,5 ) = -y*invz_2 *fy_;

    Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

    jacobian_pixel_uv ( 0,0 ) = ( getPixelValue ( u+1,v )-getPixelValue ( u-1,v ) ) /2;
    jacobian_pixel_uv ( 0,1 ) = ( getPixelValue ( u,v+1 )-getPixelValue ( u,v-1 ) ) /2;

    _jacobianOplusXi = jacobian_pixel_uv*jacobian_uv_ksai;
  }

  // dummy read and write functions because we don't care...
  virtual bool read ( std::istream& in ) {}
  virtual bool write ( std::ostream& out ) const {}

protected:
  // get a gray scale value from reference image (bilinear interpolated)
  inline float getPixelValue ( float x, float y )
  {
    uchar* data = & image_->data[ int ( y ) * image_->step + int ( x ) ];
    float xx = x - floor ( x );
    float yy = y - floor ( y );
    return float (
            ( 1-xx ) * ( 1-yy ) * data[0] +
            xx* ( 1-yy ) * data[1] +
            ( 1-xx ) *yy*data[ image_->step ] +
            xx*yy*data[image_->step+1]
    );
  }
public:
  Eigen::Vector3d x_world_;   // 3D point in world frame
  float cx_=0, cy_=0, fx_=0, fy_=0; // Camera intrinsics
  cv::Mat* image_=nullptr;    // reference image
};

class WindowDirectBA {
public:
  WindowDirectBA(CamPtr cam):cam_(cam) {};
  ~WindowDirectBA() {};
  void optimize(vector<SE3>& poses, vector<Vector3d>& points, vector<cv::Mat>& images, vector<float *>& color);
  bool poseEstimationDirect(const vector<Vector3d>& points, const vector<float>& color, cv::Mat* gray, Eigen::Isometry3d& Tcw);
private:
  CamPtr cam_;
};


}


#endif //STEREO_VO_WINDOW_BA_H
