#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/algorithm/clamp.hpp>

#include "odometry.h"

using namespace stereo_vo;

int main ( int argc, char** argv )
{
  Odometry vo;
  std::shared_ptr<Frame> frame;
  string data_dir = "/Users/jennings/Desktop/stereo_vo/data/08";

  cv::viz::Viz3d vis ( "Visual Odometry" );
  cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
  cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
  cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
  vis.setViewerPose ( cam_pose );

  world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
  camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
  vis.showWidget ( "World", world_coor );
  vis.showWidget ( "Camera", camera_coor );

  for(int i = 0; i < 700; i++) {
    char buffer [10];
    sprintf(buffer, "%06d.png", i);
    string fname(buffer);
    cout << data_dir+"/image_2/"+fname << endl;

    frame = vo.addFrame(data_dir+"/image_2/"+fname, data_dir+"/image_3/"+fname);
    // draw
    vector<cv::Point2f> kps;
    vector<float> depth;
    vo.getProjectedPoints(kps, depth);
    cv::Mat img_show = cv::imread(data_dir+"/image_2/"+fname);
    for(int i = 0; i < kps.size(); i++) {
      float d = boost::algorithm::clamp(depth[i]*1.5, 0.0, 1.0);
      cv::circle(img_show, kps[i], 2, cv::Scalar(d * 255, 255*(1-d), 0), CV_FILLED);
    }

    // show the map and the camera pose
    SE3 Twc = frame->T_c_w_.inverse();
    cout << "Twc: " << endl << Twc.matrix() << endl;

    cv::Affine3d M (
      cv::Affine3d::Mat3 (
        Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
        Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
        Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
      ),
      cv::Affine3d::Vec3 (
        Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
      )
    );

    cv::imshow("corners", img_show);
    cv::waitKey(1);
    vis.setWidgetPose("Camera", M);
    vis.spinOnce (1, false);
  }
  return 0;
}
