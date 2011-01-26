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

#include "feature.h"

#include <ntk/image/sift_gpu.h>
#include <ntk/image/sift.h>
#include <ntk/utils/opencv_utils.h>

using namespace cv;

namespace ntk
{

void FeatureSet :: extractFromImage(const RGBDImage& image,
                                    const std::string& detector_type,
                                    const std::string& descriptor_type,
                                    float threshold)
{
  m_descriptor_index.release();

  if (detector_type == "GPUSIFT")
  {
    ntk_ensure(descriptor_type == "SIFT",
               "Only SIFT descriptor are supported with GPUSIFT detector");
    if (getSiftGPUInstance())
      return extractFromImageUsingSiftGPU(image);
    ntk_dbg(0) << "[WARNING] SIFT Gpu cannot be used";
  }
  else if (detector_type == "SIFTPP")
  {
    ntk_ensure(descriptor_type == "SIFT",
               "Only SIFT descriptor are supported with SIFTPP detector");
    return extractFromImageUsingSiftPP(image);
  }

  cv::FeatureDetector* detector = 0;
  if (detector_type == "FAST")
  {
    cv::FeatureDetector* fast_detector = new FastFeatureDetector(threshold > 0 ? threshold : 10,
                                                                 true/*nonmax_suppression*/);
    detector = new PyramidAdaptedFeatureDetector(fast_detector, 2);
  }
  else if (detector_type == "SIFT" || detector_type == "GPUSIFT")
  {
    detector = new SiftFeatureDetector(SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                       SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
  }
  else if (detector_type == "SURF")
  {
    detector = new SurfFeatureDetector(threshold > 0 ? threshold : 400 /*hessian_threshold*/,
                                       5/*octaves*/, 4/*octave_layers*/);
  } 
  else
  {
    fatal_error(("Point detector not supported: " + detector_type).c_str());
  }

  std::vector<cv::KeyPoint> keypoints;
  detector->detect(image.rgbAsGray(), keypoints);

  cv::DescriptorExtractor* extractor = 0;
  if (descriptor_type == "BRIEF32")
  {
    m_feature_type = Feature_BRIEF32;
    extractor = new cv::BriefDescriptorExtractor(32);
  }
  else if (descriptor_type == "BRIEF64")
  {
    m_feature_type = Feature_BRIEF64;
    extractor = new cv::BriefDescriptorExtractor(64);
  }
  else if (descriptor_type == "SIFT")
  {
    m_feature_type = Feature_SIFT;
    extractor = new cv::SiftDescriptorExtractor();
  }
  else if (descriptor_type == "SURF64")
  {
    m_feature_type = Feature_SURF64;
    extractor = new cv::SurfDescriptorExtractor(4 /* octaves */,
                                                2 /* octave layers */,
                                                false /* extended */);
  }
  else if (descriptor_type == "SURF128")
  {
    m_feature_type = Feature_SURF128;
    extractor = new cv::SurfDescriptorExtractor(4 /* octaves */,
                                                2 /* octave layers */,
                                                true /* extended */);
  }
  else
  {
    fatal_error(("Point extractor not supported: " + descriptor_type).c_str());
  }

  cv::Mat descriptors;
  extractor->compute(image.rgbAsGray(), keypoints, descriptors);
  m_descriptor_size = extractor->descriptorSize();
  switch (extractor->descriptorType())
  {
  case CV_32FC1:
    descriptors.copyTo(m_descriptors);
    break;
  case CV_8UC1:
    m_descriptors.create(descriptors.size());
    for_all_rc(descriptors)
      m_descriptors(r,c) = descriptors.at<uchar>(r,c) / 255.0;
    break;
  default:
    ntk_assert(0, "Descriptor type not supported!");
  }

  m_locations.resize(keypoints.size());
  foreach_idx(i, keypoints)
    ((KeyPoint&)m_locations[i]) = keypoints[i];
  fillDepthData(image);
}

void FeatureSet :: extractFromImageUsingSiftGPU(const RGBDImage& image)
{
  GPUSiftDetector detector;
  m_descriptor_size = 128;

  std::vector<float> descriptors;
  std::vector<KeyPoint> keypoints;

  detector(image.rgbAsGray(), Mat(), keypoints, descriptors);

  m_locations.resize(keypoints.size());
  foreach_idx(i, keypoints)
    ((KeyPoint&)m_locations[i]) = keypoints[i];
  fillDepthData(image);

  m_descriptors = cv::Mat1f(keypoints.size(), 128);
  ntk_ensure(descriptors.size() == m_descriptors.size().area(), "INvalid number of keypoints");
  std::copy(descriptors.begin(), descriptors.end(), m_descriptors.ptr<float>());
  m_feature_type = Feature_SIFT;
}

void FeatureSet :: extractFromImageUsingSiftPP(const RGBDImage& image)
{
  const int levels = 3;
  int O = -1;
  const int S = levels;
  const int omin = -1;
  float const sigman = .5 ;
  float const sigma0 = 1.6 * powf(2.0f, 1.0f / S) ;
  float threshold = 0.01; // closer to Lowe.
  float edgeThreshold  = 10.0f;
  int unnormalized = 0;
  float magnif = 3.0;

  VL::PgmBuffer buffer;
  cv::Mat1f fim(image.rgbAsGray().size());
  for_all_rc(fim) fim(r, c) = image.rgbAsGray()(r, c) / 255.0;

  if(O < 1)
  {
    O = std::max
        (int
        (std::floor
        (log2
        (std::min(fim.cols,fim.rows))) - omin -3), 1);
  }

  VL::Sift sift(fim[0], fim.cols, fim.rows,
                sigman, sigma0,
                O, S,
                omin, -1, S + 1) ;

  sift.detectKeypoints(threshold, edgeThreshold);
  sift.setNormalizeDescriptor(!unnormalized);
  sift.setMagnification(magnif);

  std::vector< std::vector<float> > descriptors;
  m_locations.clear();

  for (VL::Sift::KeypointsConstIter iter = sift.keypointsBegin();
       iter != sift.keypointsEnd(); ++iter)
  {
    FeatureLocation new_location;

    // detect orientations
    VL::float_t angles [4] ;
    int nangles = sift.computeKeypointOrientations(angles, *iter) ;

    // compute descriptors
    for (int a = 0 ; a < nangles ; ++a)
    {
      new_location.pt.x = iter->x;
      new_location.pt.y = iter->y;
      new_location.size = iter->sigma*16;
      new_location.octave = iter->o;
      new_location.angle = angles[a];

      /* compute descriptor */
      VL::float_t descr_pt [128] ;
      sift.computeKeypointDescriptor(descr_pt, *iter, angles[a]) ;

      std::vector<float> desc_vec(128);
      std::copy(descr_pt, descr_pt+128, desc_vec.begin());

      m_locations.push_back(new_location);
      descriptors.push_back(desc_vec);
    } // next angle
  } // next keypoint

  m_descriptors.create(descriptors.size(), 128);
  for_all_rc(m_descriptors)
  {
    m_descriptors(r,c) = descriptors[r][c];
  }

  fillDepthData(image);
  m_feature_type = Feature_SIFT;
  m_descriptor_size = 128;
}

void FeatureSet :: fillDepthData(const RGBDImage& image)
{
  foreach_idx(i, m_locations)
  {
    FeatureLocation& loc = m_locations[i];
    if (is_yx_in_range(image.depthMask(), loc.pt.y, loc.pt.x)
        && image.depthMask()(loc.pt.y, loc.pt.x))
    {
      loc.has_depth = true;
      loc.depth = image.depth()(loc.pt.y, loc.pt.x);
    }
  }
}

void FeatureSet :: draw(const cv::Mat3b& image, cv::Mat3b& display_image) const
{
  std::vector<KeyPoint> keypoints(m_locations.size());
  foreach_idx(i, keypoints)
    keypoints[i] = (const KeyPoint&)m_locations[i];
  cv::Mat dummy;
  image.copyTo(display_image);
  drawKeypoints(image,
                keypoints,
                display_image,
                Scalar(255,0,0,255),
                DrawMatchesFlags::DRAW_OVER_OUTIMG|DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}

void FeatureSet :: drawMatches(const cv::Mat3b& image,
                               const cv::Mat3b& rhs_image,
                               const FeatureSet& rhs_features,
                               const std::vector<cv::DMatch>& matches,
                               cv::Mat3b& display_image) const
{
  std::vector<KeyPoint> keypoints1(m_locations.size());
  foreach_idx(i, keypoints1)
    keypoints1[i] = (const KeyPoint&)m_locations[i];
  std::vector<KeyPoint> keypoints2(rhs_features.m_locations.size());
  foreach_idx(i, keypoints2)
    keypoints2[i] = (const KeyPoint&)rhs_features.m_locations[i];

  cv::drawMatches(rhs_image, keypoints2,
                  image, keypoints1,
                  matches, display_image,
                  Scalar(255,0,0,255), Scalar(255,255,0,255),
                  vector<char>());
}

void FeatureSet :: buildDescriptorIndex()
{
  if (m_locations.size() < 1)
    return;
  m_descriptor_index = new cv::flann::Index_<float>(m_descriptors,
                                                    cv::flann::KDTreeIndexParams(4));
}

void FeatureSet :: matchWith(const FeatureSet& rhs,
                             std::vector<cv::DMatch>& matches,
                             float ratio_threshold)
{
  ntk_ensure(featureType() == rhs.featureType(), "Cannot match with different feature type.");

  if (!m_descriptor_index)
    buildDescriptorIndex();

  const cv::Mat1f& rhs_descriptors = rhs.descriptors();
  for (int i = 0; i < rhs_descriptors.rows; ++i)
  {
    std::vector<int> indices(2, -1);
    std::vector<float> dists(2, 0);

    std::vector<float> query(descriptorSize());
    std::copy(rhs_descriptors.ptr<float>(i),
              rhs_descriptors.ptr<float>(i+1),
              query.begin());
    m_descriptor_index->knnSearch(query, indices, dists, 2,
                                  cv::flann::SearchParams(64));
    if (indices[0] < 0 || indices[1] < 0)
      continue;
    const double dist_ratio = dists[0]/dists[1];
    if (dist_ratio > ratio_threshold) // probably wrong match
      continue;

    DMatch m(i, indices[0], -1, dist_ratio);
    matches.push_back(m);
  }
}

} // ntk
