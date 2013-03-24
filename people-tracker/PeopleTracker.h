/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
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
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */

#ifndef PEOPLETRACKER_H
#define PEOPLETRACKER_H

#include <ntk/core.h>
#include <ntk/camera/rgbd_image.h>
#include <ntk/geometry/plane.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/utils/opencv_utils.h>

/*!
 * Store the parameters of the People Tracker.
 */
struct PeopleTrackerParameters
{
  /*! Load data from a opencv yaml file. */
  void loadFromYamlFile(const std::string& filename);

  /*! Template depth image used for correlation computing. */
  cv::Mat1f template_img;

  /*! Minimal depth value in template depth image */
  float min_template_depth;

  /*! Size of one pixels in meters in the template image. */
  double template_pixel_size;

  /*! Initial points defining the maximal working zone. */
  cv::Point3f initial_working_zone[4];

  /*! Threshold on correlation with the template image. */
  double min_correlation_threshold;

  /*! Maximal overlap ratio between two detections bounding boxes. */
  double max_bbox_overlap;

  /*! Minimal distance between too person detection, taken from
   *  their respective highest point.
   */
  double min_inter_person_dist;
};

class PeopleTracker
{
public:
  struct DetectionFilterPredicate;

  struct PersonDetection
  {
    cv::Rect bbox;
    float score;
    cv::Point3f highest_point;
    cv::Point3f highest_point_in_image;
    float person_height;
    bool enabled;

    bool operator<(const PersonDetection& rhs) const
    {
      if (highest_point_in_image.z > rhs.highest_point_in_image.z + 1e-5)
        return true;

      // Bias towards the most centered ones.
      float dcenter1 = ntk::math::sqr(highest_point_in_image.x - (bbox.x + (bbox.width / 2.))) +
          ntk::math::sqr(highest_point_in_image.y - (bbox.y + (bbox.height / 2.)));
      float dcenter2 = ntk::math::sqr(rhs.highest_point_in_image.x - (rhs.bbox.x + (rhs.bbox.width / 2.))) +
          ntk::math::sqr(highest_point_in_image.y - (rhs.bbox.y + (rhs.bbox.height / 2.)));
      return dcenter1 > dcenter2;
    }
  };

public:
    PeopleTracker(const PeopleTrackerParameters& parameters);

public:
    bool isInitialized() const { return m_virtual_top_view.isValid(); }
    void saveBackgroundInfoToFile(const std::string& filename) const;
    void loadBackgroundInfoFromFile(const std::string& filename);
    void estimateBackground(const ntk::RGBDImage& image);
    void processNewImage(const ntk::RGBDImage& image);
    const ntk::RGBDImage& foregroundImage() const { return m_foreground_image; }
    const ntk::Plane& groundPlane() const { return m_ground_plane; }
    const std::vector<PersonDetection>& detections() const { return m_current_detections; }

    /*!
     * Returns the four 3D points in the ground plane that define the working zone.
     */
    const cv::Point3f* workingZone() const { return m_working_zone; }

    /*!
     * Virtual point of view on top of the scene.
     *
     * WARNING: reprensent the camera transformation. If you want
     * to transform world point into this camera coordinates, you need
     * to apply them the inverse transformation.
     *
     * Gets calculated from the ground plane using the following:
     * - Center is the centroid of the working zone.
     * - Viewing direction is the ground plane normal.
     * - Distance from the ground plane is 3 meters.
     * - Projection is orthographic.
     */
    const ntk::Pose3D& virtualTopView() const { return m_virtual_top_view; }

protected:
    void detectPeople();
    void computeForeground();
    void computeVirtualTopView();
    void computeVirtualTopViewPose();
    void estimateGroundPlaneFromUserInput(const ntk::RGBDImage& image);
    void estimateWorkingZoneFromUserInput(const ntk::RGBDImage& image);
    void estimateInitialWorkingZone(const ntk::RGBDImage& image);

private:
    /*!
     * Compute correlation value for a given template at a given image location.
     */
    double correlationValue(const cv::Mat1f& img,
                            const cv::Mat1f& pattern,
                            const cv::Point& topleft);

private:
    PeopleTrackerParameters m_parameters;
    cv::Mat1f m_background_depth;
    ntk::Plane m_ground_plane;
    ntk::RGBDImage m_current_image;
    ntk::RGBDImage m_foreground_image;
    cv::Mat1f m_current_top_view;
    cv::Point3f m_working_zone[4];
    ntk::Pose3D m_virtual_top_view;
    int m_top_image_width;
    int m_top_image_height;
    double m_top_pixel_size;
    std::vector<PersonDetection> m_current_detections;
};

#endif // PEOPLETRACKER_H
