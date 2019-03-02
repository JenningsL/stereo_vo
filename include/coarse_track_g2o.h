//
// Created by jennings on 2019/1/20.
//

#ifndef STEREO_VO_G2O_TYPES_H
#define STEREO_VO_G2O_TYPES_H
#include "common.h"
#include "camera.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

using namespace g2o;

namespace stereo_vo {

// 一次测量的值，包括一个世界坐标系下三维点与一个灰度值
struct Measurement
{
  Measurement ( Eigen::Vector3d p, float g ) : pos_world ( p ), grayscale ( g ) {}
  Eigen::Vector3d pos_world;
  float grayscale;
};

// project a 3d point into an image plane, the error is photometric error
// an unary edge with one vertex SE3Expmap (the pose of camera)
class EdgeSE3ProjectDirect: public BaseUnaryEdge< 1, double, VertexSE3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectDirect() {}

  EdgeSE3ProjectDirect ( Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat* image )
          : x_world_ ( point ), fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), image_ ( image )
  {}

  virtual void computeError()
  {
    const VertexSE3Expmap* v  =static_cast<const VertexSE3Expmap*> ( _vertices[0] );
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
    VertexSE3Expmap* vtx = static_cast<VertexSE3Expmap*> ( _vertices[0] );
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

bool poseEstimationDirect ( const vector<Measurement>& measurements, cv::Mat* gray, Eigen::Matrix3f& intrinsics, Eigen::Isometry3d& Tcw );

}
#endif //STEREO_VO_G2O_TYPES_H
