/**
 * This file is part of the nestk library.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#include "relative_pose_estimator.h"

#include <ntk/utils/time.h>
#include <ntk/utils/stl.h>
#include <ntk/stats/histogram.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/opencv_utils.h>

using namespace cv;

namespace ntk
{

/*!
 * Compute the projection error of a 3D point cloud on a new image.
 * If feature points on the new image have depth information, it
 * will be taken into account.
 */
struct reprojection_error_3d : public ntk::CostFunction
{
  reprojection_error_3d(const Pose3D& initial_pose,
                        const std::vector<Point3f>& ref_points,
                        const std::vector<Point3f>& img_points)
      : CostFunction(6, ref_points.size()*3),
        initial_pose(initial_pose),
        ref_points(ref_points),
        img_points(img_points)
  {
    ntk_assert(ref_points.size() == img_points.size(), "Invalid matches");
  }

  virtual void evaluate (const std::vector< double > &x, std::vector< double > &fx) const
  {
    const bool use_depth = true;
    Pose3D new_pose = initial_pose;
    new_pose.applyTransformAfter(Vec3f(x[3],x[4],x[5]), cv::Vec3f(x[0],x[1],x[2]));
    int err_i = 0;
    std::fill(stl_bounds(fx), 0);
    foreach_idx(p_i, ref_points)
    {
      const Point3f& ref_point = ref_points[p_i];
      const Point3f& img_point = img_points[p_i];
      Point3f proj_p = new_pose.projectToImage(ref_point);
      fx[err_i*3] = (proj_p.x - img_point.x);
      fx[err_i*3+1] = (proj_p.y - img_point.y);
      if (use_depth && img_point.z > 1e-5 && ref_point.z > 1e-5)
        fx[err_i*3+2] = new_pose.meanFocal() * (proj_p.z - img_point.z);
      else
        fx[err_i*3+2] = 0;
      err_i = err_i + 1;
    }
  }

private:
  const Pose3D& initial_pose;
  const std::vector<Point3f>& ref_points;
  const std::vector<Point3f>& img_points;
};

// atomic mean square pose estimation.
double rms_optimize_3d(Pose3D& pose3d,
                       const std::vector<Point3f>& ref_points,
                       const std::vector<Point3f>& img_points)
{
  std::vector<double> fx;
  std::vector<double> initial(6);
  reprojection_error_3d f(pose3d, ref_points, img_points);
  LevenbergMarquartMinimizer optimizer;
  std::fill(stl_bounds(initial), 0);
  fx.resize(ref_points.size()*3);
  optimizer.minimize(f, initial);
  optimizer.diagnoseOutcome();
  f.evaluate(initial, fx);

  double error = f.outputNorm(initial);

  pose3d.applyTransformAfter(Vec3f(initial[3],initial[4],initial[5]), cv::Vec3f(initial[0], initial[1], initial[2]));
  return error;
}

double rms_optimize_ransac(Pose3D& pose3d,
                           const std::vector<Point3f>& ref_points,
                           const std::vector<Point3f>& img_points,
                           std::vector<bool>& valid_points)
{
  // One centimeter. 1000*1000 comes from the error scale factor
  // in rms_optimize.
  const double rms_err_threshold = 5;
  const double compat_err_threshold = 3;
  const int max_iterations = 30;
  const int min_support_points = 7;
  const float min_consensus_support_percent = 0.05;

  ntk_assert(ref_points.size() > 7, "Not enough points.");

  cv::RNG rgen;
  double best_error = FLT_MAX;
  Pose3D best_pose = pose3d;
  Pose3D current_pose = pose3d;
  std::vector<Point3f> current_ref_points;
  std::vector<Point3f> current_img_points;
  std::set<int> best_indices;
  std::set<int> indices;

  for (int it = 0; it < max_iterations && best_error > 0.01; ++it)
  {
    // initial set and model
    draw_k_different_numbers(rgen, indices, min_support_points, ref_points.size());

    current_pose = pose3d;
    current_ref_points.clear();
    current_img_points.clear();
    current_ref_points.reserve(indices.size());
    current_img_points.reserve(indices.size());
    foreach_const_it(it, indices, std::set<int>)
    {
      current_ref_points.push_back(ref_points[*it]);
      current_img_points.push_back(img_points[*it]);
    }

    double initial_error = rms_optimize_3d(current_pose, current_ref_points, current_img_points);
    // Base solution not good enough.
    if (initial_error > rms_err_threshold)
      continue;

    ntk_dbg_print(initial_error, 2);
    // determine consensus set.
    foreach_idx(index, ref_points)
    {
      if (indices.find(index) != indices.end())
        continue;

      Point3f proj_p = current_pose.projectToImage(ref_points[index]);
      double error = 0;
      error += ntk::math::sqr(proj_p.x - img_points[index].x);
      error += ntk::math::sqr(proj_p.y - img_points[index].y);
      if (img_points[index].z > 1e-5)
        error += ntk::math::sqr((proj_p.z - img_points[index].z)*current_pose.meanFocal());
      error = sqrt(error);

      if (error < compat_err_threshold)
        indices.insert(index);
    }

    ntk_dbg_print(indices.size(), 2);
    if (indices.size() < (min_consensus_support_percent*ref_points.size()))
      continue;

    current_ref_points.clear();
    current_img_points.clear();
    foreach_const_it(it, indices, std::set<int>)
    {
      current_ref_points.push_back(ref_points[*it]);
      current_img_points.push_back(img_points[*it]);
    }
    // current_pose = pose3d;

    double iteration_error = rms_optimize_3d(current_pose, current_ref_points, current_img_points);
    ntk_dbg_print(iteration_error, 2);
    if (iteration_error < best_error)
    {
      best_error = iteration_error;
      best_pose = current_pose;
      best_indices = indices;
    }
  } // end ransac loop

  valid_points.resize(ref_points.size(), false);
  foreach_const_it(it, best_indices, std::set<int>)
      valid_points[*it] = true;
  ntk_dbg_print(best_indices.size(), 2);
  pose3d = best_pose;
  return best_error;
}

} // ntk

namespace ntk
{

bool RelativePoseEstimatorFromFile::estimateNewPose(const RGBDImage& image)
{
  ntk_ensure(image.hasDirectory(), "Only works in fake mode!");
  m_current_pose.parseAvsFile((image.directory() + "/relative_pose.avs").c_str());
  return true;
}

bool RelativePoseEstimatorFromDelta::estimateNewPose(const RGBDImage& image)
{
  m_current_pose.applyTransformAfter(m_delta_pose);
  return true;
}

/*!
 * Compute the number of closest feature matches for each previous view.
 */
void RelativePoseEstimatorFromImage::
computeNumMatchesWithPrevious(const RGBDImage& image,
                              std::vector < FeaturePoint<FeatureData> >& image_features,
                              std::vector<int>& view_matches)
{
  // Compute the number of best matches for each past image
  view_matches.resize(m_image_data.size(), 0);
  for (int i = 0; i < image_features.size(); ++i)
  {
    std::vector<int> indices(2, -1);
    std::vector<float> dists(2, 0);
    m_feature_index->knnSearch(image_features[i].descriptor, indices, dists, 2,
                               cv::flann::SearchParams(64));
    if (indices[0] < 0 || indices[1] < 0)
      continue;
    const double dist_ratio = dists[0]/dists[1];
    if (dist_ratio > 0.9*0.9) // probably wrong match
      continue;

    int view_index = m_features[indices[0]].associated_data.image_index;
    ntk_assert(view_index >= 0  && view_index < m_image_data.size(), "Invalid index.");
    ++view_matches[view_index];
  }
}

void RelativePoseEstimatorFromImage::
computeFeaturePoints(const RGBDImage& image,
                     std::vector < FeaturePoint<FeatureData> >& points)
{
  TimeCount tc ("sift detection");
  std::vector<cv::KeyPoint> keypoints;
  std::vector<float> descriptors;
  m_detector(image.rgbAsGray(), cv::Mat(), keypoints, descriptors);
  tc.stop();
  ntk_dbg_print(keypoints.size(), 2);

  points.resize(keypoints.size());

  foreach_idx(i, keypoints)
  {
    FeaturePoint<FeatureData>& p = points[i];
    p.associated_data.image_index = newImageIndex();
    p.descriptor.resize(m_detector.descriptorSize());
    p.location = keypoints[i];
    p.depth = image.depth()(p.location.pt.y, p.location.pt.x);
    std::copy(&descriptors[m_detector.descriptorSize()*i],
              &descriptors[m_detector.descriptorSize()*(i+1)],
              p.descriptor.begin());
  }
}

struct FeatureMatch
{
  int ref_index;
  int img_index;
  double ratio;
};

template <class FeatureData>
void findBestMatches(std::vector<FeatureMatch>& matches,
                     const std::vector < FeaturePoint<FeatureData> >& image_features,
                     const std::vector < FeaturePoint<FeatureData> >& ref_features,
                     int ref_start_index,
                     int ref_end_index,
                     double max_ratio)
{
  const int nb_ref_features = ref_end_index - ref_start_index;

  // Brute force finding of best matches.
  for (int img_i = 0; img_i < image_features.size(); ++img_i)
  {
    FeatureMatch match;
    match.img_index = img_i;
    match.ref_index = -1;
    match.ratio = 1;

    float min_dist = FLT_MAX;
    float min_dist2 = FLT_MAX;

    for (int ref_i = ref_start_index; ref_i < ref_end_index; ++ref_i)
    {
      double dist = euclidian_distance(ref_features[ref_i].descriptor,
                                       image_features[img_i].descriptor,
                                       min_dist2);
      if (dist < min_dist)
      {
        min_dist2 = min_dist;
        min_dist = dist;
        match.ref_index = ref_i;
      }
      else if (dist < min_dist2)
      {
        min_dist2 = dist;
      }
    }

    match.ratio = min_dist / min_dist2;
    if (match.ratio < max_ratio)
      matches.push_back(match);
  }
}

bool RelativePoseEstimatorFromImage::
estimateDeltaPose(Pose3D& new_pose,
                  const RGBDImage& image,
                  std::vector < FeaturePoint<FeatureData> >& image_features,
                  int closest_view_index)
{
  ntk_dbg_print(new_pose, 2);
  const ImageData& ref_image_data = m_image_data[closest_view_index];

  std::vector<FeatureMatch> matches;
  findBestMatches(matches, image_features, m_features,
                  ref_image_data.first_descriptor_index,
                  ref_image_data.last_descriptor_index,
                  0.7*0.7 /* max NN dist ratio */);

  ntk_dbg_print(matches.size(), 2);
  if (matches.size() < 8)
  {
    ntk_dbg(2) << "Not enough matches";
    return false;
  }

  std::vector<Point3f> ref_points;
  std::vector<Point3f> img_points;
  std::vector<KeyPoint> ref_keypoints;
  std::vector<KeyPoint> img_keypoints;
  std::vector<DMatch> cv_matches;
  foreach_idx(i, matches)
  {
    // No depth, cannot unproject.
    if (m_features[matches[i].ref_index].depth < 1e-5)
      continue;
    Point3f ref_point (m_features[matches[i].ref_index].location.pt.x,
                       m_features[matches[i].ref_index].location.pt.y,
                       m_features[matches[i].ref_index].depth);
    ref_point = ref_image_data.pose.unprojectFromImage(ref_point);

    Point3f img_point (image_features[matches[i].img_index].location.pt.x,
                       image_features[matches[i].img_index].location.pt.y,
                       image_features[matches[i].img_index].depth);

    ref_points.push_back(ref_point);
    img_points.push_back(img_point);

    ref_keypoints.push_back(m_features[matches[i].ref_index].location);
    img_keypoints.push_back(image_features[matches[i].img_index].location);

    DMatch match(i,i,0.1);
    cv_matches.push_back(match);
  }

  cv::Mat3b debug_img;
  drawMatches(image.rgb(),
              img_keypoints,
              m_image_data[closest_view_index].color,
              ref_keypoints,
              cv_matches,
              debug_img);
  // imshow("matches", debug_img);

  ntk_dbg_print(ref_points.size(), 2);
  if (ref_points.size() < 10)
  {
    ntk_dbg(2) << "Not enough matches with depth";
    return false;
  }

  // double error = rms_optimize_3d(new_pose, ref_points, img_points);
  std::vector<bool> valid_points;
  double error = rms_optimize_ransac(new_pose, ref_points, img_points, valid_points);

  ntk_dbg_print(error, 1);
  ntk_dbg_print(new_pose, 2);

  cv_matches.clear();
  foreach_idx(i, valid_points)
  {
    if (valid_points[i])
      cv_matches.push_back(DMatch(i,i,0.1));
  }

  drawMatches(image.rgb(),
              img_keypoints,
              m_image_data[closest_view_index].color,
              ref_keypoints,
              cv_matches,
              debug_img);
  // imshow("matches_ransac", debug_img);
  // FIXME: does not work when threaded cv::waitKey(10);

  if (error < 5)
    return true;
  else
    return false;
}

bool RelativePoseEstimatorFromImage::estimateNewPose(const RGBDImage& image)
{
  ntk_ensure(image.calibration(), "Image must be calibrated.");
  if (!m_current_pose.isValid())
  {
    m_current_pose = *image.calibration()->depth_pose;
  }

  std::vector < FeaturePoint<FeatureData> > image_features;
  computeFeaturePoints(image, image_features);

  Pose3D new_pose = *image.calibration()->depth_pose;
  bool pose_ok = true;

  if (m_image_data.size() > 0)
  {
    std::vector<int> view_matches;
    computeNumMatchesWithPrevious(image, image_features, view_matches);

    // Find the view with the highest number of best matches
    int best_matches_num = -1;
    int closest_view_index = find_max(stl_bounds(view_matches), best_matches_num);
    ntk_assert(closest_view_index >= 0, "No previous views!");

    new_pose = m_image_data[closest_view_index].pose;

    if (best_matches_num > 0)
    {
      // Estimate the relative pose w.r.t the closest view.
      if (!estimateDeltaPose(new_pose, image, image_features, closest_view_index))
        pose_ok = false;
    }
    else
    {
      pose_ok = false;
    }
  }

  if (pose_ok)
  {
    ImageData image_data;
    image.rgb().copyTo(image_data.color);
    image_data.pose = new_pose;
    m_current_pose = new_pose;
    image_data.first_descriptor_index = m_features.size();
    image_data.last_descriptor_index = m_features.size() + image_features.size();
    m_features.insert(m_features.end(), stl_bounds(image_features));
    m_image_data.push_back(image_data);
    rebuildFeatureIndex();
    return true;
  }
  else
    return false;
}

void RelativePoseEstimatorFromImage::rebuildFeatureIndex()
{
  if (m_features.size() < 1)
    return;
  m_cv_features.create(m_features.size(), m_features[0].descriptor.size());
  for (int r = 0; r < m_features.size(); ++r)
  {
    for (int c = 0; c < m_cv_features.cols; ++c)
      m_cv_features(r,c) = m_features[r].descriptor[c];
  }
  if (m_feature_index)
  {
    delete m_feature_index;
    m_feature_index = 0;
  }
  m_feature_index = new cv::flann::Index(m_cv_features, cv::flann::KDTreeIndexParams(4));
}

void RelativePoseEstimatorFromImage::reset()
{
  m_features.clear();
  m_image_data.clear();
  if (m_feature_index)
  {
    delete m_feature_index;
    m_feature_index = 0;
  }
}

} // ntk

