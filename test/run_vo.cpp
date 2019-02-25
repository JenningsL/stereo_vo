#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/algorithm/clamp.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

#include "odometry.h"
#include "INIReader.h"

using namespace stereo_vo;

void Draw(pangolin::View &d_cam,pangolin::OpenGlRenderState& s_cam, const VecSE3 &poses, const VecVec3d &points, const CameraPtr cam) {
  if (poses.empty() || points.empty()) {
    cerr << "parameter is empty!" << endl;
    return;
  }

  float fx = cam->fx_ / 8;
  float fy = cam->fy_ / 8;
  float cx = cam->cx_ / 8;
  float cy = cam->cy_ / 8;


  // while (pangolin::ShouldQuit() == false) {}
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  d_cam.Activate(s_cam);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

  // draw poses
  float sz = 0.01;
  int width = 1226/8, height = 370/8;
  for (auto &Tcw: poses) {
    glPushMatrix();
    Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
    glMultMatrixf((GLfloat *) m.data());
    glColor3f(1, 0, 0);
    glLineWidth(2);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glEnd();
    glPopMatrix();
  }

  // points
  glPointSize(1);
  glBegin(GL_POINTS);
  for (size_t i = 0; i < points.size(); i++) {
    // glColor3f(0.0, points[i][2]/4, 1.0-points[i][2]/4);
    // glColor3f(0.0, 1.0-(points[i][1]-0.01)*35, (points[i][1]-0.01)*35);
    glColor3f(0.7, 0.7, 0.7);
    glVertex3d(points[i][0], points[i][1], points[i][2]);
  }
  glEnd();

  pangolin::FinishFrame();
}

int main ( int argc, char** argv )
{
  INIReader config(argv[1]);
  if (config.ParseError() < 0) {
    std::cout << "Can't load config file " << argv[1] << endl;
    return 1;
  }

  // parse config
  string data_dir = config.Get("data", "path", "UNKNOWN");
  std::vector<std::string> tokens;
  string cam_str = config.Get("camera", "intrinsic", "UNKNOWN");
  boost::split(tokens, cam_str, boost::is_any_of(" "));
  vector<double> intrinsic;
  for(auto& s: tokens)
    intrinsic.push_back((double)atof(s.c_str()));
  double baseline = atof(config.Get("camera", "baseline", "UNKNOWN").c_str());
  int img_start = config.GetInteger("data", "start", 0);
  int img_end = config.GetInteger("data", "end", 0);

  CameraPtr camera(new Camera(intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3], baseline));
  Odometry vo(camera);

  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(
      1024, 768, 500,
      500, 512, 389,
      0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.3, -0.1, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
          .SetHandler(new pangolin::Handler3D(s_cam));

  FramePtr frame;
  for(int i = img_start; i < img_end; i++) {
    char buffer [10];
    sprintf(buffer, "%06d.png", i);
    string fname(buffer);
    frame = vo.addFrame(data_dir+"/image_2/"+fname, data_dir+"/image_3/"+fname);
    // draw
    vector<cv::Point2f> kps;
    vector<float> depth;
    vo.getProjectedPoints(kps, depth);
    cv::Mat img_show = cv::imread(data_dir+"/image_2/"+fname);
    for(int i = 0; i < kps.size(); i++) {
      float d = boost::algorithm::clamp(depth[i], 0.0, 1.0);
      cv::circle(img_show, kps[i], 1, cv::Scalar(d * 255, 255*(1-d), 0), CV_FILLED);
    }

    // show the map and the camera pose
    SE3 Twc = frame->T_c_w_.inverse();
    // cout << "Twc: " << endl << Twc.matrix() << endl;

    cv::imshow("Result", img_show);
    // cv::imwrite(data_dir+"/out/"+fname, img_show);
    cv::waitKey(1);

    VecSE3 poses;
    VecVec3d points;
    vector<MapPointPtr> all_points;
    vo.map->getAllPoints(all_points);
    for(FramePtr frame:vo.key_frames) {
      poses.push_back(frame->T_c_w_);
    }
    for(auto point:all_points) {
      points.push_back(point->pt);
    }

    Draw(d_cam, s_cam, poses, points, vo.cam_);
  }
  cv::waitKey(0);
  return 0;
}
