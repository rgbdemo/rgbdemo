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

#ifndef NTK_IMAGE_FEATURE_H
#define NTK_IMAGE_FEATURE_H

#include <ntk/core.h>
#include <ntk/camera/rgbd_image.h>
#include <ntk/utils/xml_serializable.h>
#include <ntk/geometry/pose_3d.h>

namespace ntk
{

class FeatureLocation : public cv::KeyPoint
{
public:
  /*! Whether depth data was available at this position */
  bool has_depth;

  /*! Point depth in image */
  float depth;

  /*! Point in 3D space */
  cv::Point3f p3d;
};

struct FeatureSetParams
{
  /*!
   * Set feature extraction parameters.
   * \param detector_type can be SURF, FAST, SIFT, GPUSIFT, SIFTPP (Andrea Vedaldi)
   * \param descriptor_type can be SURF64, SURF128, SIFT or BRIEF
   * \param only_features_with_depth Whether feature without associated
   *                                 depth value get detected.
   * \param threshold Threshold for the feature detector. -1 means default value.
   */
  FeatureSetParams(const std::string& detector_type,
                   const std::string& extractor_type,
                   bool only_features_with_depth = false,
                   float threshold = -1)
   : detector_type(detector_type),
     descriptor_type(extractor_type),
     only_features_with_depth(only_features_with_depth),
     threshold(threshold)
  {
  }

  /*!
   * Constructor with default parameters (FAST, BRIEF64).
   */
  FeatureSetParams()
   : detector_type("FAST"),
     descriptor_type("BRIEF64"),
     only_features_with_depth(false),
     threshold(-1)
  {}

  std::string detector_type;
  std::string descriptor_type;
  bool only_features_with_depth;
  float threshold;
};

/*!
 * Represent a set of feature points with associated descriptors.
 */
class FeatureSet : public XmlSerializable
{
public:
  /*! Descriptor type */
  enum FeatureType { Feature_SIFT,
                     Feature_SURF64,
                     Feature_SURF128,
                     Feature_BRIEF32,
                     Feature_BRIEF64 };

public:
  FeatureSet() {}
  ~FeatureSet() {}

public:
  virtual void fillXmlElement(XMLNode& element) const {}
  using ntk::XmlSerializable::loadFromXmlElement;
  virtual void loadFromXmlElement(const XMLNode& element) {}

public:
  /*! Returns a unique string to identify the feature type. */
  static std::string featureTypeName(const std::string& descriptor_type);
  std::string featureTypeName() const;
  FeatureType featureType() const { return (FeatureType) m_feature_type; }

  int descriptorSize() const { return m_descriptor_size; }

  const std::vector<FeatureLocation>& locations() const { return m_locations; }
  const cv::Mat1f& descriptors() const { return m_descriptors; }

public:
  /*! Compute each feature p3d location using the given pose. */
  void compute3dLocation(const Pose3D& pose);

public:
  /*!
   * Extract descriptors from an image.
   */
  void extractFromImage(const RGBDImage& image,
                        const FeatureSetParams& params);

public:
  /*!
   * Find matches of rhs features with this feature set.
   * \param matches The output vector of matches.
   * \param ratio_threshold The maximal ration between distance
   * to the closest and distance to the second closest.
   */
  void matchWith(const FeatureSet& rhs,
                 std::vector<cv::DMatch>& matches,
                 float ratio_threshold = 0.8*0.8);

public:
  void draw(const cv::Mat3b& image, cv::Mat3b& display_image) const;
  void drawMatches(const cv::Mat3b& image,
                   const cv::Mat3b& rhs_image,
                   const FeatureSet& rhs_features,
                   const std::vector<cv::DMatch>& matches,
                   cv::Mat3b& display_image) const;

private:
  void extractFromImageUsingSiftGPU(const RGBDImage& image, const FeatureSetParams& params);
  void extractFromImageUsingSiftPP(const RGBDImage& image, const FeatureSetParams& params);
  void fillDepthData(const RGBDImage& image);
  void buildDescriptorIndex();

private:
  ntk::Ptr< cv::flann::Index> m_descriptor_index;
  char m_feature_type;
  unsigned m_descriptor_size;
  std::vector<FeatureLocation> m_locations;
  cv::Mat1f m_descriptors;
};

/*!
 * Represent a particular feature in a feature set.
 */
struct Feature
{
  Feature(const FeatureSet& set, int index)
    : set(set), index(index)
  {}

  bool comparableTo(const Feature& p2) const
  { return p2.set.featureType() == set.featureType(); }

  const FeatureSet& set;
  int index;
};

} // ntk

#endif // NTK_IMAGE_FEATURE_H
