//
// Created by jennings on 2019/2/18.
//
#include "window_ba.h"
namespace stereo_vo {

bool WindowDirectBA::poseEstimationDirect(const vector<Vector3d>& points, const vector<float>& color, cv::Mat* gray, Eigen::Isometry3d& Tcw)
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

  int id=1;
  for (int i = 0; i < points.size();i++)
  {
    EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect (
            points[i],
            cam_->fx_, cam_->fy_, cam_->cx_, cam_->cy_, gray
    );
    edge->setVertex ( 0, pose );
    edge->setMeasurement (color[i]);
    edge->setInformation (Eigen::Matrix<double,1,1>::Identity());
    edge->setId (id++);
    optimizer.addEdge(edge);
  }
  cout<<"edges in graph: "<<optimizer.edges().size() <<endl;
  optimizer.initializeOptimization();
  optimizer.optimize ( 30 );
  Tcw = pose->estimate();
}

void WindowDirectBA::optimize(vector<SE3>& poses, vector<Vector3d>& points, vector<cv::Mat>& images, vector<float *>& color) {
  cout << "poses: " << poses.size() << ", points: " << points.size() << ", color: "  << color.size() << ", images: " << images.size() << endl;

  // build optimization problem
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> DirectBlock;  // 求解的向量是6＊1的
  DirectBlock::LinearSolverType *linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
  DirectBlock *solver_ptr = new DirectBlock(linearSolver);
  g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // L-M
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  // add vertices, edges into the graph optimizer
  int num_poses = poses.size();
  // add pose vertices
  for(int i = 0; i < poses.size(); i++) {
    auto pose = poses[i];
    VertexSophus* pose_vertex = new VertexSophus();
    pose_vertex->setEstimate(pose);
    pose_vertex->setId(i);
    if(i == 0) {
      pose_vertex->setFixed(true);
    }
    pose_vertex->setFixed(true);
    optimizer.addVertex(pose_vertex);
  }
  // add points vertices
  for(int j = 0; j < points.size(); j++) {
    g2o::VertexSBAPointXYZ* point_vertex = new g2o::VertexSBAPointXYZ();
    point_vertex->setEstimate(points[j]);
    point_vertex->setId(j + num_poses); // avoid conflict

    point_vertex->setFixed(true);
    point_vertex->setMarginalized(true);
    optimizer.addVertex(point_vertex);
  }
  // add edges
  for(int j = 0; j < points.size(); j++) {
    for(int i = 0; i < poses.size(); i++) {
      EdgeDirectProjection* edge = new EdgeDirectProjection(color[j], images[i], cam_);
      edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(j+num_poses)));
      edge->setVertex(1, dynamic_cast<VertexSophus*>(optimizer.vertex(i)));

      edge->setInformation(Eigen::Matrix<double,16,16>::Identity());
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      rk->setDelta(1.0);
      edge->setRobustKernel(rk);
      optimizer.addEdge(edge);
    }
  }

  cout << "Construct done." << endl;
  // perform optimization
  optimizer.initializeOptimization(0);
  optimizer.optimize(200);

  // fetch data from the optimizer
  for(int i = 0; i < poses.size(); i++) {
    VertexSophus* pose_vertex = dynamic_cast<VertexSophus*>(optimizer.vertex(i));
    poses[i] = pose_vertex->estimate();
  }
  for(int j = 0; j < points.size(); j++) {
    g2o::VertexSBAPointXYZ* point_vertex = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(j + num_poses));
    points[j] = point_vertex->estimate();
  }
}

}