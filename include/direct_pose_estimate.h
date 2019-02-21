//
// Created by jennings on 2019/2/21.
//

#ifndef STEREO_VO_DIRECT_POSE_ESTIMATE_H
#define STEREO_VO_DIRECT_POSE_ESTIMATE_H

#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>
#include <chrono>

#include "common.h"
#include "camera.h"

namespace stereo_vo {

using namespace std;

// useful typedefs
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVec2d &px_ref,
        const vector<double> depth_ref,
        std::shared_ptr<Camera> cam,
        Sophus::SE3 &T21
);

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVec2d &px_ref,
        const vector<double> depth_ref,
        std::shared_ptr<Camera> cam,
        Sophus::SE3 &T21
);

}
#endif //STEREO_VO_DIRECT_POSE_ESTIMATE_H
