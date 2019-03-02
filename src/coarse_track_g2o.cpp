//
// Created by jennings on 2019/1/20.
//

#include "coarse_track_g2o.h"

namespace stereo_vo
{

bool poseEstimationDirect ( const vector< Measurement >& measurements, cv::Mat* gray, Eigen::Matrix3f& K, Eigen::Isometry3d& Tcw )
{
  // 初始化g2o
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  // 求解的向量是6＊1的
  DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
  DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
  // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm ( solver );
  optimizer.setVerbose( true );

  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
  pose->setEstimate ( g2o::SE3Quat ( Tcw.rotation(), Tcw.translation() ) );
  pose->setId ( 0 );
  optimizer.addVertex ( pose );

  // 添加边
  int id=1;
  for ( Measurement m: measurements )
  {
    EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect (
            m.pos_world,
            K ( 0,0 ), K ( 1,1 ), K ( 0,2 ), K ( 1,2 ), gray
    );
    edge->setVertex ( 0, pose );
    edge->setMeasurement ( m.grayscale );
    edge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
    edge->setId ( id++ );
    optimizer.addEdge ( edge );
  }
  cout<<"edges in graph: "<<optimizer.edges().size() <<endl;
  optimizer.initializeOptimization();
  optimizer.optimize ( 30 );
  Tcw = pose->estimate();
}

}
