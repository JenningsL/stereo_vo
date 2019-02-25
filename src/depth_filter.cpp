//
// Created by jennings on 2019/2/23.
//
#include "depth_filter.h"

namespace depth_filter {

using namespace stereo_vo;

// 极线搜索
// 方法见书 13.2 13.3 两节
bool epipolarSearch(
        const CameraPtr cam,
        const Mat &ref, const Mat &curr,
        const SE3 &T_C_R, const Vector2d &pt_ref,
        const double &depth_mu, const double &depth_cov,
        Vector2d &pt_curr) {
  Vector3d f_ref = cam->pixel2camera(pt_ref);
  f_ref.normalize();
  Vector3d P_ref = f_ref * depth_mu;  // 参考帧的 P 向量

  Vector2d px_mean_curr = cam->camera2pixel(T_C_R * P_ref); // 按深度均值投影的像素
  double d_min = depth_mu - 3 * depth_cov, d_max = depth_mu + 3 * depth_cov;
  if (d_min < 0.1) d_min = 0.1;
  Vector2d px_min_curr = cam->camera2pixel(T_C_R * (f_ref * d_min));  // 按最小深度投影的像素
  Vector2d px_max_curr = cam->camera2pixel(T_C_R * (f_ref * d_max));  // 按最大深度投影的像素

  Vector2d epipolar_line = px_max_curr - px_min_curr;  // 极线（线段形式）
  Vector2d epipolar_direction = epipolar_line;    // 极线方向
  epipolar_direction.normalize();
  double half_length = 0.5 * epipolar_line.norm();  // 极线线段的半长度
  if (half_length > 100) half_length = 100;   // 我们不希望搜索太多东西

  // 取消此句注释以显示极线（线段）
  // showEpipolarLine( ref, curr, pt_ref, px_min_curr, px_max_curr );

  // 在极线上搜索，以深度均值点为中心，左右各取半长度
  double best_ncc = -1.0;
  Vector2d best_px_curr;
  for (double l = -half_length; l <= half_length; l += 0.7)  // l+=sqrt(2)
  {
    Vector2d px_curr = px_mean_curr + l * epipolar_direction;  // 待匹配点
    if (!inside(px_curr, curr.cols, curr.rows))
      continue;
    // 计算待匹配点与参考帧的 NCC
    double ncc = NCC(ref, curr, pt_ref, px_curr);
    if (ncc > best_ncc) {
      best_ncc = ncc;
      best_px_curr = px_curr;
    }
  }
  if (best_ncc < 0.85f)      // 只相信 NCC 很高的匹配
    return false;
  pt_curr = best_px_curr;
  return true;
}

// 对整个深度图进行更新
bool update(const CameraPtr cam, const Mat &ref, const Mat &curr, const SE3 &T_C_R, const VecVec2d &pts_2d, vector<float> &depth,
            vector<float> &depth_cov, vector<bool>& success) {
  #pragma omp parallel for
  for (int i = 0; i < depth.size(); i++) {
    // 遍历每个像素
    // 深度已收敛或发散
    if (depth_cov[i] < min_cov) {
      success.push_back(true);
      continue;
    }
    if(depth_cov[i] > max_cov) {
      success.push_back(false);
      continue;
    }

    // 在极线上搜索 (x,y) 的匹配
    Vector2d pt_curr;
    bool ret = epipolarSearch(cam, ref, curr, T_C_R, pts_2d[i], depth[i], sqrt(depth_cov[i]), pt_curr);

    success.push_back(ret);
    if (ret == false) // 匹配失败
      continue;

    // 取消该注释以显示匹配
    // showEpipolarMatch( ref, curr, pts_2d[i], pt_curr );

    // 匹配成功，更新深度图
    updateDepthFilter(cam, pts_2d[i], pt_curr, T_C_R, i, depth, depth_cov);
  }
}

double NCC(
        const Mat &ref, const Mat &curr,
        const Vector2d &pt_ref, const Vector2d &pt_curr
) {
  // 零均值-归一化互相关
  // 先算均值
  double mean_ref = 0, mean_curr = 0;
  vector<double> values_ref, values_curr; // 参考帧和当前帧的均值
  for (int x = -ncc_window_size; x <= ncc_window_size; x++)
    for (int y = -ncc_window_size; y <= ncc_window_size; y++) {
      double value_ref = double(ref.ptr<uchar>(int(y + pt_ref(1, 0)))[int(x + pt_ref(0, 0))]) / 255.0;
      mean_ref += value_ref;

      double value_curr = getBilinearInterpolatedValue(curr, pt_curr + Vector2d(x, y));
      mean_curr += value_curr;

      values_ref.push_back(value_ref);
      values_curr.push_back(value_curr);
    }

  mean_ref /= ncc_area;
  mean_curr /= ncc_area;

  // 计算 Zero mean NCC
  double numerator = 0, demoniator1 = 0, demoniator2 = 0;
  for (int i = 0; i < values_ref.size(); i++) {
    double n = (values_ref[i] - mean_ref) * (values_curr[i] - mean_curr);
    numerator += n;
    demoniator1 += (values_ref[i] - mean_ref) * (values_ref[i] - mean_ref);
    demoniator2 += (values_curr[i] - mean_curr) * (values_curr[i] - mean_curr);
  }
  return numerator / sqrt(demoniator1 * demoniator2 + 1e-10);   // 防止分母出现零
}

bool updateDepthFilter(
        const CameraPtr cam,
        const Vector2d &pt_ref,
        const Vector2d &pt_curr,
        const SE3 &T_C_R,
        const int index,
        vector<float> &depth,
        vector<float> &depth_cov
) {
  // 我是一只喵
  // 不知道这段还有没有人看
  // 用三角化计算深度
  SE3 T_R_C = T_C_R.inverse();
  Vector3d f_ref = cam->pixel2camera(pt_ref);
  f_ref.normalize();
  Vector3d f_curr = cam->pixel2camera(pt_curr);
  f_curr.normalize();

  // 方程
  // d_ref * f_ref = d_cur * ( R_RC * f_cur ) + t_RC
  // => [ f_ref^T f_ref, -f_ref^T f_cur ] [d_ref] = [f_ref^T t]
  //    [ f_cur^T f_ref, -f_cur^T f_cur ] [d_cur] = [f_cur^T t]
  // 二阶方程用克莱默法则求解并解之
  Vector3d t = T_R_C.translation();
  Vector3d f2 = T_R_C.rotation_matrix() * f_curr;
  Vector2d b = Vector2d(t.dot(f_ref), t.dot(f2));
  double A[4];
  A[0] = f_ref.dot(f_ref);
  A[2] = f_ref.dot(f2);
  A[1] = -A[2];
  A[3] = -f2.dot(f2);
  double d = A[0] * A[3] - A[1] * A[2];
  Vector2d lambdavec =
          Vector2d(A[3] * b(0, 0) - A[1] * b(1, 0),
                   -A[2] * b(0, 0) + A[0] * b(1, 0)) / d;
  Vector3d xm = lambdavec(0, 0) * f_ref;
  Vector3d xn = t + lambdavec(1, 0) * f2;
  Vector3d d_esti = (xm + xn) / 2.0;  // 三角化算得的深度向量
  double depth_estimation = d_esti.norm();   // 深度值

  // 计算不确定性（以一个像素为误差）
  Vector3d p = f_ref * depth_estimation;
  Vector3d a = p - t;
  double t_norm = t.norm();
  double a_norm = a.norm();
  double alpha = acos(f_ref.dot(t) / t_norm);
  double beta = acos(-a.dot(t) / (a_norm * t_norm));
  double beta_prime = beta + atan(1 / cam->fx_);
  double gamma = M_PI - alpha - beta_prime;
  double p_prime = t_norm * sin(beta_prime) / sin(gamma);
  double d_cov = p_prime - depth_estimation;
  double d_cov2 = d_cov * d_cov;

  // 高斯融合
  double mu = depth[index];
  double sigma2 = depth_cov[index];

  double mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
  double sigma_fuse2 = (sigma2 * d_cov2) / (sigma2 + d_cov2);

  if(std::fabs((depth_estimation - mu)/mu) > 0.2)
    return false;

  depth[index] = mu_fuse;
  depth_cov[index] = sigma_fuse2;

  // cout << "mu: " <<  mu << ", mu_estimate: "<< depth_estimation << "cov: " << sigma_fuse2 << endl;

  return true;
}

// 后面这些太简单我就不注释了（其实是因为懒）
void plotDepth(const Mat &depth) {
  imshow("depth", depth * 0.4);
  waitKey(1);
}

void showEpipolarMatch(const Mat &ref, const Mat &curr, const Vector2d &px_ref, const Vector2d &px_curr) {
  Mat ref_show, curr_show;
  cv::cvtColor(ref, ref_show, CV_GRAY2BGR);
  cv::cvtColor(curr, curr_show, CV_GRAY2BGR);

  cv::circle(ref_show, cv::Point2f(px_ref(0, 0), px_ref(1, 0)), 5, cv::Scalar(0, 0, 250), 2);
  cv::circle(curr_show, cv::Point2f(px_curr(0, 0), px_curr(1, 0)), 5, cv::Scalar(0, 0, 250), 2);

  imshow("ref", ref_show);
  imshow("curr", curr_show);
  waitKey(0);
}

void showEpipolarLine(const Mat &ref, const Mat &curr, const Vector2d &px_ref, const Vector2d &px_min_curr,
                      const Vector2d &px_max_curr) {

  Mat ref_show, curr_show;
  cv::cvtColor(ref, ref_show, CV_GRAY2BGR);
  cv::cvtColor(curr, curr_show, CV_GRAY2BGR);

  cv::circle(ref_show, cv::Point2f(px_ref(0, 0), px_ref(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
  cv::circle(curr_show, cv::Point2f(px_min_curr(0, 0), px_min_curr(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
  cv::circle(curr_show, cv::Point2f(px_max_curr(0, 0), px_max_curr(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
  cv::line(curr_show, Point2f(px_min_curr(0, 0), px_min_curr(1, 0)), Point2f(px_max_curr(0, 0), px_max_curr(1, 0)),
           Scalar(0, 255, 0), 1);

  imshow("ref", ref_show);
  imshow("curr", curr_show);
  waitKey(0);
}

} // end namespace depth_filter