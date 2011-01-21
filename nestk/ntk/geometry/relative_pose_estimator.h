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
 * Represent a feature point, with location and descriptor.
 */
template <class DataType>
class FeaturePoint
{
public:
  cv::KeyPoint location;
  float depth;
  std::vector<float> descriptor;
  DataType associated_data;
};

/*!
 * Estimate relative 3D pose using feature point detection.
 * Feature matches are computed between the new image and past images,
 * allowing direct estimation of the relative pose.
 */
class RelativePoseEstimatorFromImage : public RelativePoseEstimator
{
public:
  RelativePoseEstimatorFromImage()
   : m_feature_index(0)
  {
    reset();
  }

  virtual bool estimateNewPose(const RGBDImage& image);
  virtual void reset();

private:
  struct FeatureData
  {
    int image_index;
  };

  struct ImageData
  {
    Pose3D pose;
    int first_descriptor_index;
    int last_descriptor_index;
    cv::Mat3b color;
  };

private:
  void rebuildFeatureIndex();
  int newImageIndex() const { return m_image_data.size(); }
  void computeFeaturePoints(const RGBDImage& image,
                            std::vector < FeaturePoint<FeatureData> >& points);
  void computeNumMatchesWithPrevious(const RGBDImage& image,
                                     std::vector < FeaturePoint<FeatureData> >& image_features,
                                     std::vector<int>& view_matches);
  bool estimateDeltaPose(Pose3D& new_pose,
                         const RGBDImage& image,
                         std::vector < FeaturePoint<FeatureData> >& image_features,
                         int closest_view_index);

private:
  cv::flann::Index* m_feature_index;
  std::vector < FeaturePoint<FeatureData> > m_features;
  std::vector<ImageData> m_image_data;
  // cv::SURF m_detector;
#if !defined(_WIN32) && !defined(__APPLE__)
  GPUSiftClient m_detector;
#else
  cv::SURF m_detector;
#endif
  cv::Mat1f m_cv_features;
};

} // ntk

#endif // NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_H
