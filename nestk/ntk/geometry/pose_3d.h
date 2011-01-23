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

#ifndef NTK_GEOMETRY_POSE3D_H
#define NTK_GEOMETRY_POSE3D_H

# include <ntk/core.h>
# include <ntk/utils/xml_serializable.h>
# include <ntk/numeric/utils.h>

namespace ntk
{

  class Pose3D : public ntk::XmlSerializable
  {
  private:
    class PrivatePose3D;
    friend class PrivatePose3D;
    PrivatePose3D* impl;

  public:
    Pose3D();
    ~Pose3D();

    Pose3D(const Pose3D& rhs);
    Pose3D& operator=(const Pose3D& rhs);

  public:
    virtual void fillXmlElement(XMLNode& element) const;
    virtual void loadFromXmlElement(const XMLNode& element);
    virtual void saveToYaml(cv::FileStorage& yaml) const;
    virtual void loadFromYaml(cv::FileNode yaml);
    void parseBlenderFile(const char* filename, int image_width, int image_height);
    void saveToBundlerFile(const char* filename);
    void parseAvsFile(const char* filename);
    void saveToAvsFile(const char* filename);

    void toLeftCamera(const cv::Mat1d& cv_matrix,
                      const cv::Mat1d& R,
                      const cv::Mat1d& T);

    void loadFromBlenderParameters(double tx, double ty, double tz,
                                   double rx, double ry, double rz,
                                   double field_of_view,
                                   int image_width,
                                   int image_height);
    void toBlenderParameters(int image_width, int image_height,
                             double* tx, double* ty, double* tz,
                             double* rx, double* ry, double* rz,
                             double* field_of_view) const;

    Pose3D computeDeltaPoseWith(const Pose3D& new_pose) const;

  public:
    double focalX() const { return m_focal_x; }
    double focalY() const { return m_focal_y; }
    bool focalAreIdentical() const { return ntk::flt_eq(m_focal_x, m_focal_y, 1e-5); }
    double meanFocal() const { return (m_focal_x + m_focal_y)/2.0; }
    double imageCenterX() const { return m_image_center_x; }
    double imageCenterY() const { return m_image_center_y; }
    bool isValid() const { return m_has_camera_params; }
    void setOrthographic(bool ortho);
    bool isOrthographic() const { return m_orthographic; }

    void setRightCameraParametersFromOpencv(const cv::Mat1d& cv_matrix,
                                            const cv::Mat1d& R,
                                            const cv::Mat1d& T);
    void setCameraParametersFromOpencv(const cv::Mat1d& cv_matrix);
    void setCameraParameters(double fx, double fy, double cx, double cy, bool orthographic = false);

    const cv::Vec3f cvTranslation() const;
    const cv::Vec3f cvEulerRotation() const;
    const cv::Vec3f cvRodriguesRotation() const;

    void applyTransformBefore(const Pose3D& rhs_pose);
    void applyTransformAfter(const Pose3D& rhs_pose);
    void applyTransformBefore(const cv::Vec3f& cvTranslation, const cv::Vec3f& rotation_euler_angles);
    void applyTransformAfter(const cv::Vec3f& tranlation, const cv::Vec3f& rotation_euler_angles);

    void applyTransformBefore(const cv::Vec3f& cvTranslation, const cv::Mat1d& rotation_matrix);
    void applyTransformAfter(const cv::Vec3f& tranlation, const cv::Mat1d& rotation_matrix);

    /*!
     * Apply translation and rotation.
     * Rotation is given using axis angle with Rodrigues representation.
     */
    void applyTransformBeforeRodrigues(const cv::Vec3f& cvTranslation, const cv::Vec3f& rotation_rodrigues);
    void applyTransformAfterRodrigues(const cv::Vec3f& tranlation, const cv::Vec3f& rotation_rodrigues);

  public:
    // Camera transform
    void resetCameraTransform();
    cv::Point3f cameraTransform(const cv::Point3f& p) const;
    cv::Point3f invCameraTransform(const cv::Point3f& p) const;
    cv::Mat1f cvInvCameraTransform() const;
    const cv::Mat1f cvCameraTransform() const;
    cv::Mat1f cvProjectionMatrix() const;
    void setCameraTransform(const cv::Mat1d& tvec, const cv::Mat1d& rvec);
    void setCameraTransform(const cv::Mat1f& H);
    void setCameraTransformFromCvFundamentalMatrix(const cv::Mat1f& F);
    void invert();

    // Center is (0,0)
    cv::Point3f projectWithoutPrincipalPoint(const cv::Point3f& p) const;

    // Same as project but then adjust to image center.
    cv::Point3f projectToImage(const cv::Point3f& p) const;
    void projectToImage(const cv::Mat3f& voxels, const cv::Mat1b& mask, cv::Mat3f& pixels) const;

    cv::Point3f unprojectFromImage(const cv::Point2f& p, double depth) const;
    cv::Point3f unprojectFromImage(const cv::Point3f& p) const        
    { return unprojectFromImage(cv::Point2f(p.x,p.y), p.z); }

    void unprojectFromImage(const cv::Mat1f& pixels, const cv::Mat1b& mask, cv::Mat3f& voxels) const;

  private:
    double m_focal_x;
    double m_focal_y;
    double m_image_center_x;
    double m_image_center_y;
    bool m_has_camera_params;
    bool m_orthographic;
  };

  cv::Vec3f estimate_normal_from_depth(const cv::Mat1f& depth_yml, const Pose3D& pose, int r, int c);
  cv::Vec3f camera_eye_vector(const Pose3D& pose, int r, int c);

  /*!
   * Return the angle axis rotation (using Rodrigues representation) transforming
   * vector src into vector dst.
   */
  cv::Vec3f compute_axis_angle_rotation(const cv::Vec3f& src, const cv::Vec3f& dst);

  const NtkDebug& operator<<(const NtkDebug& os, const Pose3D& p);

} // ntk

#endif // NTK_GEOMETRY_POSE3D_H
