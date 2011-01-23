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

#include "pose_3d.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/eigen_utils.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>

using namespace cv;

namespace ntk
{

class Pose3D::PrivatePose3D
{
public:
  PrivatePose3D(Pose3D* iface) : iface(iface) {}
  PrivatePose3D& operator=(const PrivatePose3D& rhs)
  {
    camera_transform = rhs.camera_transform;
    inv_camera_transform = rhs.inv_camera_transform;
    intrinsics_transform = rhs.intrinsics_transform;
    project_transform = rhs.project_transform;
    inv_project_transform = rhs.inv_project_transform;
    return *this;
  }

  Pose3D* iface;

  Eigen::Isometry3d camera_transform;
  Eigen::Isometry3d inv_camera_transform;
  Eigen::Isometry3d intrinsics_transform;
  Eigen::Projective3d project_transform;
  Eigen::Projective3d inv_project_transform;

  Eigen::Isometry3d intrinsicsTransform() const
  {
    Eigen::Isometry3d m; m.setIdentity();
    m(0,0) = iface->m_focal_x;
    m(0,2) = iface->m_image_center_x;
    m(1,1) = iface->m_focal_y;
    m(1,2) = iface->m_image_center_y;
    return m;
  }

  Eigen::Matrix3d eigenRotation() const
  { return camera_transform.rotation(); }

  Eigen::Vector3d eigenTranslation() const
  { return camera_transform.translation(); }

  Eigen::Isometry3d eigenRotationTransform() const
  {
    Eigen::Isometry3d r = Eigen::Isometry3d::Identity();
    r.rotate(eigenRotation());
    return r;
  }

  /*!
   * FIXME: this function is not tested.
   */
  Eigen::Vector3d projectWithoutPrincipalPoint(const Eigen::Vector3d& p) const
  {
    ntk_assert(0, "not tested.");
    Eigen::Vector3d r = camera_transform * p;
    r(0) = -r(0) * iface->m_focal_x;
    r(1) = iface->m_focal_y;
    if (!iface->isOrthographic())
    {
      r(0) /= r(2);
      r(1) /= r(2);
    }
    if (iface->isOrthographic())
      r(2) = -(camera_transform * Eigen::Vector3d(p(0), p(1), p(2)))(2);
    r(2) *= -1;
    return r;
  }

  Eigen::Vector4d projectToImage(const Eigen::Vector4d& p) const
  {
    ntk_assert(iface->m_has_camera_params, "You need to set camera params first!");
    Eigen::Vector4d r = project_transform * p;
    if (!iface->isOrthographic())
    {
      r(0) /= r(2);
      r(1) /= r(2);
    }
    if (iface->isOrthographic())
      r(2) = -(camera_transform * Eigen::Vector3d(p(0), p(1), p(2)))(2);
    return r;
  }

  void unprojectFromImage(const Eigen::Vector4d& p, Eigen::Vector4d& output) const
  {
    ntk_assert(iface->m_has_camera_params, "You need to set camera params first!");
    if (iface->isOrthographic())
    {
      Eigen::Vector3d r ((p(0)-iface->m_image_center_x)/iface->m_focal_x,
                         -(p(1)-iface->m_image_center_y)/iface->m_focal_y,
                         -p(2));
      r = inv_camera_transform * r;
      output = Eigen::Vector4d(r(0), r(1), r(2), 1);
    }
    else
    {
      Eigen::Vector4d r (p(0)*p(2), p(1)*p(2), p(2), 1);
      output = inv_project_transform * r;
    }
  }

  void applyTransformBefore(const Eigen::Isometry3d& transform)
  {
    camera_transform = camera_transform * transform;
    computeProjectiveTransform();
  }

  void applyTransformAfter(const Eigen::Isometry3d& transform)
  {
    camera_transform = transform * camera_transform;
    computeProjectiveTransform();
  }

  void computeProjectiveTransform()
  {
    // y points downward in the image, upward in real world.
    // same for z.
    Eigen::Isometry3d to_opencv = Eigen::Isometry3d::Identity();
    to_opencv(1,1) = to_opencv(2,2) = -1;

    Eigen::Projective3d projection = Eigen::Projective3d::Identity();
    if (iface->isOrthographic())
    {
      projection(2,2) = 0;
      projection(2,3) = 1;
    }

    Eigen::Isometry3d intrinsics = intrinsicsTransform();
    inv_camera_transform = camera_transform.inverse();

    project_transform = intrinsics * projection * to_opencv * camera_transform;
    inv_project_transform = project_transform.inverse();
  }

  // http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
   PrivatePose3D(const PrivatePose3D&);
};

Pose3D :: Pose3D() :
  impl(new PrivatePose3D(this)),
  m_focal_x(1), m_focal_y(1),
  m_image_center_x(0), m_image_center_y(0),
  m_has_camera_params(false),
  m_orthographic(false)
{
  impl->camera_transform.setIdentity();
  impl->project_transform = impl->camera_transform;
  impl->inv_camera_transform = impl->camera_transform;
  impl->inv_project_transform = impl->project_transform.inverse();
}

Pose3D :: ~Pose3D()
{
  delete impl;
  impl = 0;
}

Pose3D :: Pose3D(const Pose3D& rhs)
  : XmlSerializable(rhs), impl(new PrivatePose3D(this))
{
  *this = rhs;
}

Pose3D& Pose3D :: operator=(const Pose3D& rhs)
{
  m_focal_x = rhs.m_focal_x;
  m_focal_y = rhs.m_focal_y;
  m_image_center_x = rhs.m_image_center_x;
  m_image_center_y = rhs.m_image_center_y;
  m_has_camera_params = rhs.m_has_camera_params;
  m_orthographic = rhs.m_orthographic;
  *impl = *rhs.impl;
  return *this;
}


void Pose3D :: setOrthographic(bool ortho)
{
  m_orthographic = ortho;
  impl->computeProjectiveTransform();
}

Vec3f estimate_normal_from_depth(const Mat1f& depth_yml, const Pose3D& pose, int r, int c)
{
  if (!is_yx_in_range(depth_yml, r-1, c-1) || !is_yx_in_range(depth_yml, r+1, c+1))
    return Point3f(1,0,0);

  float depth = depth_yml(r,c);

  Vec3f v1l (1.0/pose.focalX(), 0, -(depth - depth_yml(r,c-1)));
  Vec3f v1r (1.0/pose.focalX(), 0, (depth - depth_yml(r,c+1)));

  Vec3f v2l (0, 1.0/pose.focalY(), -(depth - depth_yml(r+1,c)));
  Vec3f v2r (0, 1.0/pose.focalY(), (depth - depth_yml(r-1,c)));

  Vec3f v1 = (v1l + v1r) * 0.5f;
  Vec3f v2 = (v2l + v2r) * 0.5f;

  Vec3f camera_normal = v1.cross(v2);
  normalize(camera_normal);
  return camera_normal;
}

Vec3f camera_eye_vector(const Pose3D& pose, int r, int c)
{
  double dx = c - pose.imageCenterX();
  double dy = r - pose.imageCenterY();
  cv::Vec3f eyev(dx/pose.focalX(), dy/pose.focalY(), 1);
  float norm = sqrt(eyev.dot(eyev));
  eyev *= (1.0f/norm);
  return eyev;
}

void Pose3D :: setCameraParametersFromOpencv(const cv::Mat1d& cv_matrix)
{
  double fx = cv_matrix(0,0);
  double fy = cv_matrix(1,1);
  double cx = cv_matrix(0,2);
  double cy = cv_matrix(1,2);
  setCameraParameters(fx,fy,cx,cy);
  impl->computeProjectiveTransform();
}

void Pose3D :: toLeftCamera(const cv::Mat1d& cv_matrix,
                            const cv::Mat1d& R,
                            const cv::Mat1d& T)
{
  double fx = cv_matrix(0,0);
  double fy = cv_matrix(1,1);
  double cx = cv_matrix(0,2);
  double cy = cv_matrix(1,2);
  setCameraParameters(fx,fy,cx,cy);

  cv::Mat1d to_gl_base(3,3); setIdentity(to_gl_base);
  to_gl_base(1,1) = -1;
  to_gl_base(2,2) = -1;

  cv::Mat1d new_R = to_gl_base.inv() * R * to_gl_base;
  cv::Mat1d new_T = to_gl_base * (T);

  applyTransformAfter(toVec3d(new_T), new_R);
}

void Pose3D :: setRightCameraParametersFromOpencv(const cv::Mat1d& cv_matrix,
                                                  const cv::Mat1d& R,
                                                  const cv::Mat1d& T)
{
  double fx = cv_matrix(0,0);
  double fy = cv_matrix(1,1);
  double cx = cv_matrix(0,2);
  double cy = cv_matrix(1,2);
  setCameraParameters(fx,fy,cx,cy);

  // OpenCV coords has y down and z toward scene.
  // OpenGL classical 3d coords has y up and z backwards
  // This is the transform matrix.
  cv::Mat1d to_gl_base(3,3); setIdentity(to_gl_base);
  to_gl_base(1,1) = -1;
  to_gl_base(2,2) = -1;

  cv::Mat1d new_R = to_gl_base.inv() * R.inv() * to_gl_base;
  cv::Mat1d new_T = to_gl_base * (-T);

  applyTransformBefore(toVec3d(new_T), new_R);
}

void Pose3D :: setCameraParameters(double fx, double fy, double cx, double cy, bool orthographic)
{
  m_focal_x = fx;
  m_focal_y = fy;
  m_image_center_x = cx;
  m_image_center_y = cy;
  m_has_camera_params = true;
  m_orthographic = orthographic;
  impl->computeProjectiveTransform();
}

void Pose3D :: saveToBundlerFile(const char* filename)
{
  std::ofstream f (filename);
  f << (m_focal_x+m_focal_y)/2.0 << " 0 0\n";
  Eigen::Matrix3d rmat = impl->eigenRotation();
  Vec3f t = cvTranslation();
  for (int r = 0; r < 3; ++r)
  {
    for (int c = 0; c < 3; ++c)
      f << rmat(r,c) << " ";
    f << "\n";
  }
  f << t[0] << " " << t[1] << " " << t[2] << "\n";
}

void Pose3D :: saveToYaml(FileStorage& yaml) const
{
  write_to_yaml(yaml, "rotation", cvEulerRotation());
  write_to_yaml(yaml, "translation", cvTranslation());
  write_to_yaml(yaml, "has_camera_params", m_has_camera_params);
  write_to_yaml(yaml, "focal_x", m_focal_x);
  write_to_yaml(yaml, "focal_y", m_focal_y);
  write_to_yaml(yaml, "image_center_x", m_image_center_x);
  write_to_yaml(yaml, "image_center_y", m_image_center_y);
  write_to_yaml(yaml, "orthographic", m_orthographic);
}

void Pose3D :: loadFromYaml(FileNode yaml)
{
  cv::Vec3f euler_angles;
  read_from_yaml(yaml["rotation"], euler_angles);
  cv::Vec3f translation;
  read_from_yaml(yaml["translation"], translation);

  resetCameraTransform();
  applyTransformBefore(translation, euler_angles);

  read_from_yaml(yaml["has_camera_params"], m_has_camera_params);
  read_from_yaml(yaml["focal_x"], m_focal_x);
  read_from_yaml(yaml["focal_y"], m_focal_y);
  read_from_yaml(yaml["image_center_x"], m_image_center_x);
  read_from_yaml(yaml["image_center_y"], m_image_center_y);
  read_from_yaml(yaml["orthographic"], m_orthographic);
  impl->computeProjectiveTransform();
}

void Pose3D :: fillXmlElement(XMLNode& element) const
{
  addXmlRawTextDataChild(element, "rotation", cvEulerRotation());
  addXmlRawTextDataChild(element, "translation", cvTranslation());
  setXmlAttribute(element, "has_camera_params", m_has_camera_params);
  setXmlAttribute(element, "focal_x", m_focal_x);
  setXmlAttribute(element, "focal_y", m_focal_y);
  setXmlAttribute(element, "image_center_x", m_image_center_x);
  setXmlAttribute(element, "image_center_y", m_image_center_y);
  setXmlAttribute(element, "orthographic", m_orthographic);
}

void Pose3D :: loadFromXmlElement(const XMLNode& element)
{
  cv::Vec3f euler_angles;
  loadFromXmlRawTextDataChild(element, "rotation", euler_angles);
  cv::Vec3f translation;
  loadFromXmlRawTextDataChild(element, "translation", translation);

  resetCameraTransform();
  applyTransformBefore(translation, euler_angles);

  loadFromXmlAttribute(element, "has_camera_params", m_has_camera_params);
  loadFromXmlAttribute(element, "focal_x", m_focal_x);
  loadFromXmlAttribute(element, "focal_y", m_focal_y);
  loadFromXmlAttribute(element, "image_center_x", m_image_center_x);
  loadFromXmlAttribute(element, "image_center_y", m_image_center_y);
  loadFromXmlAttribute(element, "orthographic", m_orthographic);
  impl->computeProjectiveTransform();
}

void Pose3D :: loadFromBlenderParameters(double tx, double ty, double tz,
                                         double rx, double ry, double rz,
                                         double field_of_view,
                                         int image_width,
                                         int image_height)
{
  impl->camera_transform.rotate(Eigen::AngleAxisf(rx*M_PI/180.0, Eigen::Vector3f::UnitZ())
                            * Eigen::AngleAxisf(ry*M_PI/180.0, Eigen::Vector3f::UnitY())
                            * Eigen::AngleAxisf(rz*M_PI/180.0, Eigen::Vector3f::UnitX()));
  impl->camera_transform.translate(Eigen::Vector3d(tx,ty,tz));

  double focal = image_width / (2.0*(tan(field_of_view * (M_PI / (180.0*2.0)))));
  setCameraParameters(focal, focal, image_width / 2.0, image_height / 2.0);
}

void Pose3D :: toBlenderParameters(int image_width, int image_height,
                                   double* tx, double* ty, double* tz,
                                   double* rx, double* ry, double* rz,
                                   double* field_of_view) const
{
  Vec3f tr = cvTranslation();
  *tx = tr[0];
  *ty = tr[1];
  *tz = tr[2];
  cv::Vec3f angles = cvEulerRotation();
  *rx = angles[0]*180.0/M_PI;
  *ry = angles[1]*180.0/M_PI;
  *rz = angles[2]*180.0/M_PI;
  *field_of_view = (180.0/M_PI) * 2.0*atan(image_width/(2.0*meanFocal()));
}

void Pose3D :: parseBlenderFile(const char* filename, int image_width, int image_height)
{
  std::ifstream f (filename);
  double tx,ty,tz,rx,ry,rz,fov;
  f >> fov >> rx >> ry >> rz >> tx >> ty >> tz;
  loadFromBlenderParameters(tx,ty,tz,rx,ry,rz,fov,image_width,image_height);
}

void Pose3D :: parseAvsFile(const char* filename)
{
  *this = Pose3D();
  std::ifstream f (filename);
  ntk_throw_exception_if(f.fail(), "Could not open" + filename);
  double fx,fy,cx,cy,tx,ty,tz,rx,ry,rz;
  f >> fx >> fy >> cx >> cy >> tx >> ty >> tz >> rx >> ry >> rz;
  setCameraParameters(fx, fy, cx, cy);  
  applyTransformBefore(Vec3f(tx,ty,tz), Vec3f(rx*M_PI/180.0, ry*M_PI/180.0, rz*M_PI/180.0));
  ntk_dbg_print(filename, 1);
  ntk_dbg_print(*this, 1);
}

void Pose3D :: saveToAvsFile(const char* filename)
{
  std::ofstream f (filename);
  cv::Vec3f angles = cvEulerRotation();
  cv::Vec3f translation = cvTranslation();

  f << m_focal_x << " " << m_focal_y << std::endl
    << m_image_center_x << " " << m_image_center_y << std::endl
    << translation[0] << " " << translation[1] << " " << translation[2] << std::endl
    << angles[0]*180.0/M_PI << " " << angles[1]*180.0/M_PI << " " << angles[2]*180.0/M_PI << std::endl;
}

void Pose3D :: resetCameraTransform()
{
  impl->camera_transform.setIdentity();
  impl->computeProjectiveTransform();
}

Pose3D Pose3D :: computeDeltaPoseWith(const Pose3D& new_pose) const
{
  ntk_assert(0, "not implemented");
  return Pose3D();
#if 0
  Pose3D delta_pose = new_pose;
  delta_pose.resetCameraTransform();
  ntk_dbg_print(new_pose.translation() - translation(), 1);
  delta_pose.applyTransformBefore(new_pose.translation() - translation(),
                                  new_pose.rotation() * rotation().inverse());
  return delta_pose;
#endif
}

/*!
 * Compute 3D rotation and translation from a fundamental matrix.
 * Assumes both images come from the same camera.
 * See http://en.wikipedia.org/wiki/Essential_matrix for details.
 * WARNING 1: the translation is known up to a scale.
 * WARNING 2: this implementation is not tested and is probably wrong.
 */
void Pose3D :: setCameraTransformFromCvFundamentalMatrix(const cv::Mat1f& F)
{
  ntk_assert(0, "Not tested, does not work.");

  cv::Mat1f to_open_cv (3,3);
  setIdentity(to_open_cv);
  to_open_cv(1,1) = -1;
  to_open_cv(2,2) = -1;

  cv::Mat1f intrinsics3x3 (3,3);
  intrinsics3x3 << m_focal_x, 0, m_image_center_x,
                   0, m_focal_y, m_image_center_y,
                   0, 0, 1;

  // essential matrix
  cv::Mat1f E(3,3);
  E = intrinsics3x3.t() * F * intrinsics3x3;
  // E = to_open_cv * E * to_open_cv.inv();

  cv::Mat1f U(3,3), SIGMA(3,3), Vt(3,3);
  cv::SVD svd(E);
  SIGMA << svd.w.at<float>(0,0), 0, 0,
           0, svd.w.at<float>(1,0), 0,
           0, 0, svd.w.at<float>(2,0);
  U = svd.u;
  Vt = svd.vt;

  ntk_dbg_print(SIGMA, 1);
  ntk_dbg_print(U, 1);
  ntk_dbg_print(Vt, 1);

  cv::Mat1f Z(3,3);
  Z << 0, -1, 0,
       1, 0, 0,
       0, 0, 0;

  cv::Mat1f W(3,3);
  W << 0, -1, 0,
       1, 0, 0,
       0, 0, 1;

  cv::Mat1f R(3,3), Tx(3,3);
  R = U * W.t() * Vt;
  // Tx = Vt.t() * Z * Vt;
  Tx = Vt.t() * W * SIGMA * Vt;
  ntk_dbg_print(Tx, 1);
  cv::Vec3f cv_t(Tx(2,1), Tx(0, 2), Tx(1, 0));
  cv_t = Vec3f(0,0,0); // FIXME: temp

  // R = to_open_cv.inv() * R * to_open_cv;

  cv::Mat1d Rd(3,3);
  copyMatWithCast(Rd, R);
  ntk_dbg_print(R, 1);
  ntk_dbg_print(Rd, 1);
  resetCameraTransform();
  applyTransformBefore(cv_t, Rd);
}

void Pose3D :: setCameraTransform(const cv::Mat1d& tvec, const cv::Mat1d& rvec)
{
  cv::Mat1d to_open_cv (4,4);
  setIdentity(to_open_cv);
  to_open_cv(1,1) = -1;
  to_open_cv(2,2) = -1;
  cv::Mat1d from_open_cv = to_open_cv.inv();

  CvMat c_rvec = rvec;
  cv::Mat1d rot(3,3); CvMat c_rot = rot;
  cvRodrigues2(&c_rvec, &c_rot);

  cv::Mat1d H = cv::Mat1d(4,4);
  setIdentity(H);
  cv::Mat1d H_rot = H(Rect(0,0,3,3));
  rot.copyTo(H_rot);
  H(0,3) = tvec(0,0);
  H(1,3) = tvec(1,0);
  H(2,3) = tvec(2,0);
  ntk_dbg_print(H, 1);
  H = from_open_cv * H * to_open_cv;

  cv::Mat1f Hf(4,4);
  for_all_rc(Hf)
      Hf(r,c) = H(r,c);
  setCameraTransform(Hf);
}

void Pose3D :: setCameraTransform(const cv::Mat1f& H)
{
  for_all_rc(H)
      impl->camera_transform(r,c) = H(r,c);
  impl->computeProjectiveTransform();
}

cv::Point3f Pose3D :: cameraTransform(const cv::Point3f& p) const
{
  Eigen::Vector3d ep; toEigen(p, ep);
  ep = impl->camera_transform * ep;
  return toVec3f(ep);
}

cv::Mat1f Pose3D :: cvInvCameraTransform() const
{
  cv::Mat1f m(4,4);
  Eigen::Matrix4f eigen_m = impl->inv_camera_transform.matrix();
  ntk::toOpencv(eigen_m, m);
  return m;
}

const cv::Vec3f Pose3D :: cvTranslation() const
{ return toVec3f((Eigen::Vector3d)impl->camera_transform.translation()); }

const cv::Vec3f Pose3D :: cvRodriguesRotation() const
{
  Eigen::AngleAxisd r;
  r.fromRotationMatrix(impl->camera_transform.rotation().matrix());
  cv::Vec3f rodrigues (r.axis()(0), r.axis()(1), r.axis()(2));
  rodrigues *= r.angle();
  return rodrigues;
}

const cv::Vec3f Pose3D :: cvEulerRotation() const
{
  cv::Vec3f angles;
  Eigen::Matrix3d rotM =  impl->camera_transform.rotation().matrix().transpose();
  double xy = sqrt(double(math::sqr(rotM(0,0)) + math::sqr(rotM(0,1))));
  if (xy > std::numeric_limits<double>::epsilon() * 8.0)
  {
    angles(0) = atan2(double(rotM(1,2)), double(rotM(2,2)));
    angles(1) = atan2(double(-rotM(0,2)), double(xy));
    angles(2) = atan2(double(rotM(0,1)), double(rotM(0,0)));
  }
  else
  {
    angles(0) = atan2(double(-rotM(2,1)), double(rotM(1,1)));
    angles(1) = atan2(double(-rotM(0,2)), double(xy));
    angles(2) = 0;
  }
  return angles;
  // Eigen::Vector3d coeffs = impl->camera_transform.rotation().eulerAngles(0, 1, 2);
  // return toVec3f(coeffs);
}

const cv::Mat1f Pose3D :: cvCameraTransform() const
{
  cv::Mat1f m(4,4);
  toOpencv(impl->camera_transform.matrix(), m);
  return m;
}

cv::Mat1f Pose3D :: cvProjectionMatrix() const
{
  cv::Mat1f m(4,4);
  toOpencv(impl->project_transform.matrix(), m);
  return m;
}

cv::Point3f Pose3D :: invCameraTransform(const cv::Point3f& p) const
{
  Eigen::Vector3d ep; toEigen(p, ep);
  ep = impl->inv_camera_transform * ep;
  return toVec3f(ep);
}

void Pose3D :: invert()
{
  impl->camera_transform = impl->camera_transform.inverse();
  impl->computeProjectiveTransform();
}

cv::Point3f Pose3D :: projectWithoutPrincipalPoint(const cv::Point3f& p) const
{
  Eigen::Vector3d ep; toEigen(p, ep);
  return toVec3f(impl->projectWithoutPrincipalPoint(ep));
}



cv::Point3f Pose3D :: projectToImage(const cv::Point3f& p) const
{
  Eigen::Vector4d ep; toEigen(p, ep);
  return toVec3f(impl->projectToImage(ep));
}

void Pose3D :: projectToImage(const cv::Mat3f& voxels, const cv::Mat1b& mask, cv::Mat3f& pixels) const
{
  Eigen::Vector4d epix;
  Eigen::Vector4d evox;
  evox(3) = 1; // w does not change.

  for (int r = 0; r < voxels.rows; ++r)
  {
    const Vec3f* voxels_data = voxels.ptr<Vec3f>(r);
    const uchar* mask_data = mask.ptr<uchar>(r);
    Vec3f* pixels_data = pixels.ptr<Vec3f>(r);
    for (int c = 0; c < voxels.cols; ++c)
    {
      if (!mask[c])
        continue;
      evox(0) = voxels_data[c][0];
      evox(1) = voxels_data[c][1];
      evox(2) = voxels_data[c][2];
      epix = impl->project_transform * evox;
      pixels_data[c][0] = epix(0)/epix(2);
      pixels_data[c][1] = epix(1)/epix(2);
      pixels_data[c][2] = epix(2);
    }
  }
}

void Pose3D :: unprojectFromImage(const cv::Mat1f& pixels, const cv::Mat1b& mask, cv::Mat3f& voxels) const
{
  Eigen::Vector4d epix;
  Eigen::Vector4d evox;

  epix(3) = 1; // w does not change.

  for (int r = 0; r < pixels.rows; ++r)
  {
    const float* pixels_data = pixels.ptr<float>(r);
    const uchar* mask_data = mask.ptr<uchar>(r);
    Vec3f* voxels_data = voxels.ptr<Vec3f>(r);
    for (int c = 0; c < pixels.cols; ++c)
    {
      if (!mask[c])
        continue;
      const float d = pixels_data[c];
      epix(0) = c*d;
      epix(1) = r*d;
      epix(2) = d;
      evox = impl->inv_project_transform * epix;
      voxels_data[c][0] = evox(0);
      voxels_data[c][1] = evox(1);
      voxels_data[c][2] = evox(2);
    }
  }
}

cv::Point3f Pose3D :: unprojectFromImage(const cv::Point2f& p, double depth) const
{
  Eigen::Vector4d ep (p.x, p.y, depth, 1);
  Eigen::Vector4d output;
  impl->unprojectFromImage(ep, output);
  return toVec3f(output);
}

void Pose3D :: applyTransformBefore(const Pose3D& rhs_pose)
{
  impl->camera_transform = impl->camera_transform * rhs_pose.impl->camera_transform;
  impl->computeProjectiveTransform();
}

void Pose3D :: applyTransformAfter(const Pose3D& rhs_pose)
{
  impl->camera_transform = rhs_pose.impl->camera_transform * impl->camera_transform;
  impl->computeProjectiveTransform();
}

void Pose3D :: applyTransformAfter(const cv::Vec3f& translation, const cv::Mat1d& rotation_matrix)
{
  Eigen::Matrix3d emat; toEigen(rotation_matrix, emat);
  impl->camera_transform.translate(toEigenVector3d(translation));
  impl->camera_transform.rotate(emat);
  impl->computeProjectiveTransform();
}

void Pose3D :: applyTransformBefore(const cv::Vec3f& translation, const cv::Mat1d& rotation_matrix)
{
  Eigen::Matrix3d emat; toEigen(rotation_matrix, emat);
  impl->camera_transform.prerotate(emat);
  impl->camera_transform.pretranslate(toEigenVector3d(translation));
  impl->computeProjectiveTransform();
}

void Pose3D :: applyTransformBefore(const cv::Vec3f& translation, const cv::Vec3f& rotation_euler_angles)
{
  impl->camera_transform.translate(toEigenVector3d(translation));
  impl->camera_transform.rotate(Eigen::AngleAxisd(rotation_euler_angles[2], Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(rotation_euler_angles[1], Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(rotation_euler_angles[0], Eigen::Vector3d::UnitX()));
  impl->computeProjectiveTransform();
}

void Pose3D :: applyTransformAfter(const cv::Vec3f& translation, const cv::Vec3f& rotation_euler_angles)
{
  impl->camera_transform.prerotate(Eigen::AngleAxisd(rotation_euler_angles[2], Eigen::Vector3d::UnitZ())
                               * Eigen::AngleAxisd(rotation_euler_angles[1], Eigen::Vector3d::UnitY())
                               * Eigen::AngleAxisd(rotation_euler_angles[0], Eigen::Vector3d::UnitX()));
  impl->camera_transform.pretranslate(toEigenVector3d(translation));
  impl->computeProjectiveTransform();
}

void Pose3D :: applyTransformBeforeRodrigues(const cv::Vec3f& cvTranslation, const cv::Vec3f& rotation_rodrigues)
{
  Eigen::Vector3d axis = toEigenVector3d(rotation_rodrigues);
  axis.normalize();

  impl->camera_transform.translate(toEigenVector3d(cvTranslation));
  impl->camera_transform.rotate(Eigen::AngleAxisd(norm(rotation_rodrigues),
                                                  axis));
  impl->computeProjectiveTransform();
}

void Pose3D :: applyTransformAfterRodrigues(const cv::Vec3f& cvTranslation, const cv::Vec3f& rotation_rodrigues)
{
  Eigen::Vector3d axis = toEigenVector3d(rotation_rodrigues);
  axis.normalize();
  impl->camera_transform.prerotate(Eigen::AngleAxisd(norm(rotation_rodrigues),
                                                     axis));
  impl->camera_transform.pretranslate(toEigenVector3d(cvTranslation));
  impl->computeProjectiveTransform();
}

const NtkDebug& operator<<(const NtkDebug& os, const Pose3D& p)
{
  os << "fx=" << p.focalX()
     << " fy=" << p.focalY()
     << " cx=" << p.imageCenterX()
     << " cy=" << p.imageCenterY()
     << " t=" << p.cvTranslation()
     << " r=" << p.cvEulerRotation();
  return os;
}

cv::Vec3f compute_axis_angle_rotation(const cv::Vec3f& src, const cv::Vec3f& dst)
{
  cv::Vec3f norm_src = src;
  normalize(norm_src);

  cv::Vec3f norm_dst = dst;
  normalize(norm_dst);

  cv::Vec3f w = norm_src.cross(norm_dst);
  float norm_w = norm(w);
  float magn = asin(norm_w);
  w *= magn / norm_w;
  return w;
}

} // end of ntk
