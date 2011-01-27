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

#ifndef NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_H
#define NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_H

#include <ntk/core.h>
#include <ntk/camera/rgbd_image.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/image/sift_gpu.h>
#include <ntk/image/feature.h>

namespace ntk
{

/*!
 * Estimate the relative 3D pose between a new image and the previous ones.
 */
class RelativePoseEstimator
{
public:
  //! Estimate the relative pose for a new image
  //! \return true is pose successfully estimated, false otherwise.
  virtual bool estimateNewPose(const RGBDImage& image) = 0;

  //! Return last estimated pose.
  const Pose3D& currentPose() const { return m_current_pose; }
  Pose3D& currentPose() { return m_current_pose; }

  //! Reset the relative pose estimator.
  virtual void reset() = 0;

protected:
  Pose3D m_current_pose;
};

/*!
 * Compute relative pose information using viewXXXX/relative_pose.avs file.
 * The image should have directory information (i.e. loaded from disk)
 * and have a "relative_pose.avs" file storing the pose information.
 * This file should be readableby the Pose3D::parseAvsFile() function.
 */
class RelativePoseEstimatorFromFile : public RelativePoseEstimator
{
public:
  virtual bool estimateNewPose(const RGBDImage& image);
  virtual void reset() {}
};

/*!
 * Compute relative pose information by applying a constant delta pose.
 * This pose estimator takes an initial pose and a delta pose, and for
 * each new frame the current_pose gets multiplied by the delta_pose.
 */
class RelativePoseEstimatorFromDelta : public RelativePoseEstimator
{
public:
  RelativePoseEstimatorFromDelta(const Pose3D& initial_pose,
                                 const Pose3D& delta_pose)
    : m_initial_pose(initial_pose),
      m_delta_pose(delta_pose)
  {
    reset();
  }

  virtual bool estimateNewPose(const RGBDImage& image);

  virtual void reset() { m_current_pose = m_initial_pose; }

private:
  Pose3D m_initial_pose;
  Pose3D m_delta_pose;
};

/*!
 * Estimate relative 3D pose using feature point detection.
 * Feature matches are computed between the new image and past images,
 * allowing direct estimation of the relative pose.
 */
class RelativePoseEstimatorFromImage : public RelativePoseEstimator
{
public:
  RelativePoseEstimatorFromImage(const FeatureSetParams& params)
   : m_feature_parameters(params)
  {
    // Force feature extraction to return only features with depth.
    m_feature_parameters.only_features_with_depth = true;
    reset();
  }

  virtual bool estimateNewPose(const RGBDImage& image);
  virtual void reset();

private:
  struct ImageData
  {
    Pose3D pose;
    cv::Mat3b color;
  };

private:
  int newImageIndex() const { return m_image_data.size(); }
  int computeNumMatchesWithPrevious(const RGBDImage& image,
                                     const FeatureSet& features,
                                     std::vector<cv::DMatch>& best_matches);
  bool estimateDeltaPose(Pose3D& new_pose,
                         const RGBDImage& image,
                         const FeatureSet& features,
                         const std::vector<cv::DMatch>& best_matches,
                         int closest_view_index);

private:
  std::vector < FeatureSet > m_features;
  std::vector< ImageData > m_image_data;
  FeatureSetParams m_feature_parameters;
};

} // ntk

#endif // NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_H
