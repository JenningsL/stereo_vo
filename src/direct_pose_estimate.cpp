//
// Created by jennings on 2019/2/21.
//
#include "direct_pose_estimate.h"

namespace stereo_vo {

void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVec2d &px_ref,
        const vector<double> depth_ref,
        std::shared_ptr<Camera> cam,
        Sophus::SE3 &T21
) {

  // parameters
  int half_patch_size = 4;
  int iterations = 100;

  double cost = 0, lastCost = 0;
  int nGood = 0;  // good projections
  VecVec2d goodProjection;

  float fx = cam->fx_;
  float fy = cam->fy_;
  float cx = cam->cx_;
  float cy = cam->cy_;

  for (int iter = 0; iter < iterations; iter++) {
    nGood = 0;
    goodProjection.clear();

    // Define Hessian and bias
    Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
    Vector6d b = Vector6d::Zero();  // 6x1 bias


    for (size_t i = 0; i < px_ref.size(); i++) {
      // compute the projection in the second image
      // TODO START YOUR CODE HERE
      Eigen::Vector3d p_1(
              (px_ref[i][0] - cx) * depth_ref[i] / fx,
              (px_ref[i][1] - cy) * depth_ref[i] / fy,
              depth_ref[i]
      );
      Eigen::Vector3d p_2 = T21 * p_1; // 3d point in camera2's frame
      float u = fx*p_2[0]/p_2[2] + cx;
      float v = fy*p_2[1]/p_2[2] + cy; // 2d point in image2
      if(u-half_patch_size < 1
         || u+half_patch_size >= img2.cols-1
         || v-half_patch_size < 1
         || v+half_patch_size >= img2.rows-1) {
        continue;
      }
      nGood++;
      goodProjection.push_back(Eigen::Vector2d(u, v));

      Matrix26d J_pixel_xi = Matrix26d::Zero();   // pixel to \xi in Lie algebra
      float Z = p_2[2];
      float X = p_2[0];
      float Y = p_2[1];
      float Z_2 = Z * Z;
      float X_2 = X * X;
      float Y_2 = Y * Y;
      J_pixel_xi << fx/Z, 0, -fx*X/Z_2, -fx*X*Y/Z_2, fx+fx*X_2/Z_2, -fx*Y/Z,
              0, fy/Z, -fy*Y/Z_2, -fy-fy*Y_2/Z_2, fy*X*Y/Z_2, fy*X/Z;
      // and compute error and jacobian
      float u1 = px_ref[i][0];
      float v1 = px_ref[i][1];
      for (int x = -half_patch_size; x < half_patch_size; x++)
        for (int y = -half_patch_size; y < half_patch_size; y++) {
          float x2 = u + x;
          float y2 = v + y;
          float x1 = u1 + x;
          float y1 = v1 + y;
          double error = GetPixelValue(img1, x1, y1) - GetPixelValue(img2, x2, y2);
          Eigen::Vector2d J_img_pixel = Eigen::Vector2d::Zero();    // image gradients
          J_img_pixel << (GetPixelValue(img2, x2 + 1, y2) - GetPixelValue(img2, x2 - 1, y2)) / 2,
                  (GetPixelValue(img2, x2, y2+1) - GetPixelValue(img2, x2, y2-1)) / 2;
          // total jacobian
          Vector6d J = -J_pixel_xi.transpose() * J_img_pixel;
          H += J * J.transpose();
          b += -error * J;
          cost += error * error;
        }
      // END YOUR CODE HERE
    }

    // solve update and put it into estimation
    //Vector6d update = H.inverse() * b;
    Vector6d update = H.ldlt().solve(b);
    if (isnan(update[0])) {
      // sometimes occurred when we have a black or white patch and H is irreversible
      cout << "update is nan" << endl;
      break;
    }
    T21 = Sophus::SE3::exp(update) * T21;

    cost /= nGood;

    if (iter > 0 && cost > lastCost) {
      // cout << "cost increased: " << cost << ", " << lastCost << endl;
      break;
    }
    lastCost = cost;
    // cout << "cost = " << cost << ", good = " << nGood << endl;
  }
  cout << "good projection: " << float(nGood) / px_ref.size()<< endl;
  // cout << "T21 = \n" << T21.matrix() << endl;

  // in order to help you debug, we plot the projected pixels here
  /*
  cv::Mat img1_show, img2_show;
  cv::cvtColor(img1, img1_show, CV_GRAY2BGR);
  cv::cvtColor(img2, img2_show, CV_GRAY2BGR);
  for (auto &px: px_ref) {
      cv::rectangle(img1_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                    cv::Scalar(0, 250, 0));
  }
  for (auto &px: goodProjection) {
      cv::rectangle(img2_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                    cv::Scalar(0, 250, 0));
  }
  cv::imshow("reference", img1_show);
  cv::imshow("current", img2_show);
  cv::waitKey();
   */
}

void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVec2d &px_ref,
        const vector<double> depth_ref,
        std::shared_ptr<Camera> cam,
        Sophus::SE3 &T21
) {

  // parameters
   int pyramids = 4;
   double scales[] = {1.0, 0.5, 0.25, 0.125};
//  int pyramids = 2;
//  double scales[] = {1.0, 0.5};

  // create pyramids
  vector<cv::Mat> pyr1, pyr2; // image pyramids
  // TODO START YOUR CODE HERE
  for (int i = 0; i < pyramids; i++) {
    cv::Mat img1_scaled, img2_scaled;
    cv::resize(img1, img1_scaled, cv::Size(), scales[i], scales[i]);
    cv::resize(img2, img2_scaled, cv::Size(), scales[i], scales[i]);
    pyr1.push_back(img1_scaled);
    pyr2.push_back(img2_scaled);
  }

  float fx = cam->fx_;
  float fy = cam->fy_;
  float cx = cam->cx_;
  float cy = cam->cy_;

  double fxG = cam->fx_, fyG = cam->fy_, cxG = cam->cx_, cyG = cam->cy_;  // backup the old values
  for (int level = pyramids - 1; level >= 0; level--) {
    VecVec2d px_ref_pyr; // set the keypoints in this pyramid level
    for (auto &px: px_ref) {
      px_ref_pyr.push_back(scales[level] * px);
    }

    // TODO START YOUR CODE HERE
    // scale fx, fy, cx, cy in different pyramid levels
    fx = fxG * scales[level];
    fy = fyG * scales[level];
    cx = cxG * scales[level];
    cy = cyG * scales[level];

    // END YOUR CODE HERE
    DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, cam, T21);
  }

}

}