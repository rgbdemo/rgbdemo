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

#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H

#include <ntk/core.h>
#include <ntk/utils/debug.h>
#include <ntk/utils/serializable.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ntk
{

template < typename _Scalar, int N>
const NtkDebug& operator<<(const NtkDebug& output, const Eigen::Matrix<_Scalar,N,1>& array)
{
  output << "[";
  for (int i = 0; i < N; ++i)
    output << array(i) << " ";
  output << "]";
  return output;
};

template < typename StreamType, typename _Scalar, int N>
StreamType& operator>>(StreamType& input, Eigen::Matrix<_Scalar,N,1>& array)
{
  for (int i = 0; i < N; ++i)
  {
    if (ntk::isStreamCorrupted(input)) ntk_throw_exception("Corrupted stream.");
    input >> array(i);
  }
  return input;
};

template < typename StreamType, typename _Scalar, int N>
StreamType& operator<<(StreamType& output, const Eigen::Matrix<_Scalar,N,1>& array)
{
  for (int i = 0; i < N; ++i)
    output << array(i) << ntk::sep();
  return output;
};

template <typename CvScalarType, typename EScalarType>
inline void toEigen(const cv::Point3_<CvScalarType>& p, Eigen::Matrix<EScalarType,3,1>& ep)
{
  ep(0) = p.x;
  ep(1) = p.y;
  ep(2) = p.z;
}

template <typename CvScalarType, typename EScalarType>
inline void toEigen(const cv::Vec<CvScalarType,3>& p, Eigen::Matrix<EScalarType,3,1>& ep)
{
  ep(0) = p[0];
  ep(1) = p[1];
  ep(2) = p[2];
}

template <typename CvScalarType, typename EScalarType>
inline void toEigen(const cv::Vec<CvScalarType,3>& p, Eigen::Matrix<EScalarType,4,1>& ep)
{
  ep(0) = p[0];
  ep(1) = p[1];
  ep(2) = p[2];
  ep(3) = 1;
}

template <typename CvScalarType, typename EScalarType>
inline void toEigen(const cv::Point3_<CvScalarType>& p, Eigen::Matrix<EScalarType,4,1>& ep)
{
  ep(0) = p.x;
  ep(1) = p.y;
  ep(2) = p.z;
  ep(3) = 1;
}

#ifdef _MSC_VER
inline void toOpencv(const Eigen::Matrix4d& ep,
                     cv::Mat1f& mat)
{
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      mat(r,c) = ep(r,c);
}
#else
template <typename CvScalarType, typename EigenScalarType, int H, int W>
inline void toOpencv(const Eigen::Matrix<EigenScalarType,H,W>& ep,
                         cv::Mat_<CvScalarType>& mat)
{
  for (int r = 0; r < H; ++r)
    for (int c = 0; c < W; ++c)
      mat(r,c) = ep(r,c);
}
#endif

#ifdef _MSC_VER
inline void toEigen(const cv::Mat1d& mat, Eigen::Matrix3d& ep)
{
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      ep(r,c) = mat(r,c);
}
#else
template <typename CvScalarType, typename EScalarType, int H, int W>
inline void toEigen(const cv::Mat_<CvScalarType>& mat, Eigen::Matrix<EScalarType,H,W>& ep)
{
  for (int r = 0; r < H; ++r)
    for (int c = 0; c < W; ++c)
      ep(r,c) = mat(r,c);
}
#endif

template <typename EScalarType>
inline cv::Vec3f toVec3f(const Eigen::Matrix<EScalarType,3,1>& v)
{
  return cv::Vec3f(v(0),v(1),v(2));
}

template <typename EScalarType>
inline cv::Vec3f toVec3f(const Eigen::Matrix<EScalarType,4,1>& v)
{
  // FIXME: divide by v(3) ?
  return cv::Vec3f(v(0),v(1),v(2));
}

inline Eigen::Vector2d toEigenVector2d(const cv::Point2f& v)
{ Eigen::Vector2d r (v.x, v.y); return r; }

inline Eigen::Vector3d toEigenVector3d(const cv::Vec3f& v)
{ Eigen::Vector3d r; toEigen(v, r); return r; }

inline Eigen::Vector4d toEigenVector4d(const cv::Vec3f& v)
{ Eigen::Vector4d r; toEigen(v, r); return r; }

template<typename _Scalar> class EulerAngles
{
public:
  enum { Dim = 3 };
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<Scalar,3,3> Matrix3;
  typedef Eigen::Matrix<Scalar,3,1> Vector3;
  typedef Eigen::Quaternion<Scalar> QuaternionType;

protected:

  Vector3 m_angles;

public:

  EulerAngles() {}
  inline EulerAngles(Scalar a0, Scalar a1, Scalar a2) : m_angles(a0, a1, a2) {}
  inline EulerAngles(const QuaternionType& q) { *this = q; }

  const Vector3& coeffs() const { return m_angles; }
  Vector3& coeffs() { return m_angles; }

  EulerAngles& operator=(const QuaternionType& q)
  {
    Matrix3 m = q.toRotationMatrix();
    return *this = m;
  }

  EulerAngles& operator=(const Matrix3& m)
  {
    // mat =  cy*cz          -cy*sz           sy
    //        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
    //       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
    m_angles.coeffRef(1) = std::asin(m.coeff(0,2));
    m_angles.coeffRef(0) = std::atan2(-m.coeff(1,2),m.coeff(2,2));
    m_angles.coeffRef(2) = std::atan2(-m.coeff(0,1),m.coeff(0,0));
    return *this;
  }

  Matrix3 toRotationMatrix(void) const
  {
    Vector3 c = m_angles.array().cos();
    Vector3 s = m_angles.array().sin();
    Matrix3 res;
    res <<  c.y()*c.z(),                    -c.y()*s.z(),                   s.y(),
        c.z()*s.x()*s.y()+c.x()*s.z(),  c.x()*c.z()-s.x()*s.y()*s.z(),  -c.y()*s.x(),
        -c.x()*c.z()*s.y()+s.x()*s.z(), c.z()*s.x()+c.x()*s.y()*s.z(),  c.x()*c.y();
    return res;
  }

  operator QuaternionType() { return QuaternionType(toRotationMatrix()); }
};

} // ntk

#endif // EIGEN_UTILS_H
