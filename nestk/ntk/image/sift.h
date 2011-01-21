// file:        sift.hpp
// author:      Andrea Vedaldi
// description: Sift declaration

// AUTORIGHTS
// Copyright (c) 2006 The Regents of the University of California
// All Rights Reserved.
//
// Created by Andrea Vedaldi (UCLA VisionLab)
//
// Permission to use, copy, modify, and distribute this software and its
// documentation for educational, research and non-profit purposes,
// without fee, and without a written agreement is hereby granted,
// provided that the above copyright notice, this paragraph and the
// following three paragraphs appear in all copies.
//
// This software program and documentation are copyrighted by The Regents
// of the University of California. The software program and
// documentation are supplied "as is", without any accompanying services
// from The Regents. The Regents does not warrant that the operation of
// the program will be uninterrupted or error-free. The end-user
// understands that the program was developed for research purposes and
// is advised not to rely exclusively on the program for any reason.
//
// This software embodies a method for which the following patent has
// been issued: "Method and apparatus for identifying scale invariant
// features in an image and use of same for locating an object in an
// image," David G. Lowe, US Patent 6,711,293 (March 23,
// 2004). Provisional application filed March 8, 1999. Asignee: The
// University of British Columbia.
//
// IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY
// FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
// INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND
// ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF CALIFORNIA HAS BEEN
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. THE UNIVERSITY OF
// CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS"
// BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATIONS TO PROVIDE
// MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

#ifndef VL_SIFT_HPP
#define VL_SIFT_HPP

#include <ntk/core.h>

#include<valarray>
#include<vector>
#include<ostream>
#include<cmath>
#include<limits>

#if defined (VL_USEFASTMATH)
#if defined (VL_MAC)
#define VL_FASTFLOAT float
#else
#define VL_FASTFLOAT double
#endif
#else
#define VL_FASTFLOAT float
#endif

#define VL_XEAS(x) #x
#define VL_EXPAND_AND_STRINGIFY(x) VL_XEAS(x)

/** @brief VisionLab namespace */
namespace VL {

/** @brief Pixel data type */
typedef float pixel_t ;

/** @brief Floating point data type
 **
 ** Although floats are precise enough for this applicatgion, on Intel
 ** based architecture using doubles for floating point computations
 ** turns out to be much faster.
 **/
typedef VL_FASTFLOAT float_t ;

/** @brief 32-bit floating data type */
typedef float float32_t ;

/** @brief 64-bit floating data type */
typedef double float64_t ;

/** @brief 32-bit integer data type */
typedef int int32_t ;

/** @brief 64-bit integer data type */
typedef long long int int64_t ;

/** @brief 32-bit unsigned integer data type */
typedef int unsigned_t ;

/** @brief 8-bit unsigned integer data type */
typedef char unsigned uint8_t ;

/** @name Fast math
 **
 ** We provide approximate mathematical functions. These are usually
 ** rather faster than the corresponding standard library functions.
 **/
/*@{*/
float   fast_resqrt(float x) ;
double  fast_resqrt(double x) ;
float_t fast_expn(float_t x) ;
float_t fast_abs(float_t x) ;
float_t fast_mod_2pi(float_t x) ;
float_t fast_atan2(float_t y, float_t x) ;
float_t fast_sqrt(float_t x) ;
int32_t fast_floor(float_t x) ;
/*@}*/

/** @brief Generic exception */
struct Exception
{
  /** @brief Build generic exception with message
   **
   ** The message can be accessed as the Exception::msg data member.
   **
   ** @param _msg message.
   **/
  Exception(std::string _msg) : msg(_msg) { }

  /** Exception message */
  std::string msg ;
} ;

/** @brief Throw generic exception
 **
 ** The macro executes the stream operations @a x to obtain
 ** an error messages. The message is then wrapped in a
 ** generic exception VL::Exception and thrown.
 **
 ** @param x sequence of stream operations.
 **/
#define VL_THROW(x)                             \
  {                                             \
    std::ostringstream oss ;                    \
    oss << x ;                                  \
    throw VL::Exception(oss.str()) ;            \
  }

/** @name PGM input/output */
/*@{*/
/** @brief PGM buffer descriptor
 **
 ** The structure describes a gray scale image and it is used by the
 ** PGM input/output functions. The fileds are self-explanatory.
 **/
struct PgmBuffer
{
  int width ;     ///< Image width
  int height ;    ///< Image hegith
  pixel_t* data ; ///< Image data
} ;
std::ostream& insertPgm(std::ostream&, pixel_t const* im, int width, int height) ;
std::istream& extractPgm(std::istream&, PgmBuffer& buffer) ;
/*@}*/

/** @brief SIFT filter
 **
 ** This class is a filter computing the Scale Invariant Feature
 ** Transform (SIFT).
 **/
class Sift
{

public:

  /** @brief SIFT keypoint
   **
   ** A SIFT keypoint is charactedized by a location x,y and a scale
   ** @c sigma. The scale is obtained from the level index @c s and
   ** the octave index @c o through a simple formula (see the PDF
   ** documentation).
   **
   ** In addition to the location, scale indexes and scale, we also
   ** store the integer location and level. The integer location is
   ** unnormalized, i.e. relative to the resolution of the octave
   ** containing the keypoint (octaves are downsampled).
   **/
  struct Keypoint
  {
    int o ;    ///< Keypoint octave index

    int ix ;   ///< Keypoint integer X coordinate (unnormalized)
    int iy ;   ///< Keypoint integer Y coordinate (unnormalized)
    int is ;   ///< Keypoint integer scale indiex

    float_t x  ;  ///< Keypoint fractional X coordinate
    float_t y  ;  ///< Keypoint fractional Y coordinate
    float_t s ;   ///< Keypoint fractional scale index

    float_t sigma ;  ///< Keypoint scale
  } ;

  typedef std::vector<Keypoint>     Keypoints ;          ///< Keypoint list datatype
  typedef Keypoints::iterator       KeypointsIter ;      ///< Keypoint list iter datatype
  typedef Keypoints::const_iterator KeypointsConstIter ; ///< Keypoint list const iter datatype

  /** @brief Constructors and destructors */
  /*@{*/
  Sift(const pixel_t* _im_pt, int _width, int _height,
       float_t _sigman,
       float_t _sigma0,
       int _O, int _S,
       int _omin, int _smin, int _smax) ;
  ~Sift() ;
  /*@}*/

  void process(const pixel_t* _im_pt, int _width, int _height) ;

  /** @brief Querying the Gaussian scale space */
  /*@{*/
  VL::pixel_t* getOctave(int o) ;
  VL::pixel_t* getLevel(int o, int s) ;
  int          getWidth() const ;
  int          getHeight() const ;
  int          getOctaveWidth(int o) const ;
  int          getOctaveHeight(int o) const ;
  VL::float_t  getOctaveSamplingPeriod(int o) const ;
  VL::float_t  getScaleFromIndex(VL::float_t o, VL::float_t s) const ;
  Keypoint     getKeypoint(VL::float_t x, VL::float_t y, VL::float_t s) const ;
  /*@}*/

  /** @brief Descriptor parameters */
  /*@{*/
  bool getNormalizeDescriptor() const ;
  void setNormalizeDescriptor(bool) ;
  void setMagnification(VL::float_t) ;
  VL::float_t getMagnification() const ;
  /*@}*/

  /** @brief Detector and descriptor */
  /*@{*/
  void detectKeypoints(VL::float_t threshold, VL::float_t edgeThreshold) ;
  int computeKeypointOrientations(VL::float_t angles [4], Keypoint keypoint) ;
  void computeKeypointDescriptor(VL::float_t* descr_pt, Keypoint keypoint, VL::float_t angle) ;
  KeypointsIter keypointsBegin() ;
  KeypointsIter keypointsEnd() ;
  /*@}*/

private:
  void prepareBuffers() ;
  void freeBuffers() ;
  void smooth(VL::pixel_t       * dst,
	      VL::pixel_t       * temp,
              VL::pixel_t const * src, int width, int height,
              VL::float_t s) ;

  void prepareGrad(int o) ;

  // scale space parameters
  VL::float_t sigman ;
  VL::float_t sigma0 ;
  VL::float_t sigmak ;

  int O ;
  int S ;
  int omin ;
  int smin ;
  int smax ;

  int width ;
  int height ;

  // descriptor parameters
  VL::float_t magnif ;
  bool        normalizeDescriptor ;

  // buffers
  VL::pixel_t*  temp ;
  int           tempReserved ;
  bool          tempIsGrad  ;
  int           tempOctave ;
  VL::pixel_t** octaves ;

  VL::pixel_t*  filter ;
  int           filterReserved ;

  Keypoints keypoints ;
} ;


}

#include<iostream>
#include<cassert>

namespace VL
{

namespace Detail
{
extern int const expnTableSize ;
extern VL::float_t const expnTableMax ;
extern VL::float_t expnTable [] ;
}

/** @brief Get width of source image
 ** @result width.
 **/
inline
int
Sift::getWidth() const
{
  return width ;
}

/** @brief Get height of source image
 ** @result height.
 **/
inline
int
Sift::getHeight() const
{
  return height ;
}

/** @brief Get width of an octave
 ** @param o octave index.
 ** @result width of octave @a o.
 **/
inline
int
Sift::getOctaveWidth(int o) const
{
  assert( omin <= o && o < omin + O ) ;
  return (o >= 0) ? (width >> o) : (width << -o) ;
}

/** @brief Get height of an octave
 ** @param o octave index.
 ** @result height of octave @a o.
 **/
inline
int
Sift::getOctaveHeight(int o) const
{
  assert( omin <= o && o < omin + O ) ;
  return (o >= 0) ? (height >> o) : (height << -o) ;
}

/** @brief Get octave
 ** @param o octave index.
 ** @return pointer to octave @a o.
 **/
inline
VL::pixel_t *
Sift::getOctave(int o)
{
  assert( omin <= o && o < omin + O ) ;
  return octaves[o-omin] ;
}

/** @brief Get level
 ** @param o octave index.
 ** @param s level index.
 ** @result pointer to level @c (o,s).
 **/
inline
VL::pixel_t *
Sift::getLevel(int o, int s)
{
  assert( omin <= o && o <  omin + O ) ;
  assert( smin <= s && s <= smax     ) ;
  return octaves[o - omin] +
    getOctaveWidth(o)*getOctaveHeight(o) * (s-smin) ;
}

/** @brief Get octave sampling period
 ** @param o octave index.
 ** @result Octave sampling period (in pixels).
 **/
inline
VL::float_t
Sift::getOctaveSamplingPeriod(int o) const
{
  return (o >= 0) ? (1 << o) : 1.0f / (1 << -o) ;
}

/** @brief Convert index into scale
 ** @param o octave index.
 ** @param s scale index.
 ** @return scale.
 **/
inline
VL::float_t
Sift::getScaleFromIndex(VL::float_t o, VL::float_t s) const
{
  return sigma0 * powf( 2.0f, o + s / S ) ;
}

/** @brief Get keypoint list begin
 ** @return iterator to the beginning.
 **/
inline
Sift::KeypointsIter
Sift::keypointsBegin()
{
  return keypoints.begin() ;
}

/** @brief Get keypoint list end
 ** @return iterator to the end.
 **/
inline
Sift::KeypointsIter
Sift::keypointsEnd()
{
  return keypoints.end() ;
}

/** @brief Set normalize descriptor flag */
inline
void
Sift::setNormalizeDescriptor(bool flag)
{
  normalizeDescriptor = flag ;
}

/** @brief Get normalize descriptor flag */
inline
bool
Sift::getNormalizeDescriptor() const
{
  return normalizeDescriptor ;
}

/** @brief Set descriptor magnification */
inline
void
Sift::setMagnification(VL::float_t _magnif)
{
  magnif = _magnif ;
}

/** @brief Get descriptor magnification */
inline
VL::float_t
Sift::getMagnification() const
{
  return magnif ;
}

/** @brief Fast @ exp(-x)
 **
 ** The argument must be in the range 0-25.0 (bigger arguments may be
 ** truncated to zero).
 **
 ** @param x argument.
 ** @return @c exp(-x)
 **/
inline
VL::float_t
fast_expn(VL::float_t x)
{
  assert(VL::float_t(0) <= x && x <= Detail::expnTableMax) ;
#ifdef VL_USEFASTMATH
  x *= Detail::expnTableSize / Detail::expnTableMax ;
  VL::int32_t i = fast_floor(x) ;
  VL::float_t r = x - i ;
  VL::float_t a = VL::Detail::expnTable[i] ;
  VL::float_t b = VL::Detail::expnTable[i+1] ;
  return a + r * (b - a) ;
#else
  return exp(-x) ;
#endif
}

/** @brief Fast @c mod(x,2pi)
 **
 ** The function quickly computes the value @c mod(x,2pi).
 **
 ** @remark The computation is fast only for arguments @a x which are
 ** small in modulus.
 **
 ** @remark For negative arguments, the semantic of the function is
 ** not equivalent to the standard library @c fmod function.
 **
 ** @param x function argument.
 ** @return @c mod(x,2pi)
 **/
inline
VL::float_t
fast_mod_2pi(VL::float_t x)
{
#ifdef VL_USEFASTMATH
  while(x < VL::float_t(0)      ) x += VL::float_t(2*M_PI) ;
  while(x > VL::float_t(2*M_PI) ) x -= VL::float_t(2*M_PI) ;
  return x ;
#else
  return (x>=0) ? std::fmod(x, VL::float_t(2*M_PI))
    : 2*M_PI + std::fmod(x, VL::float_t(2*M_PI)) ;
#endif
}

/** @brief Fast @c (int) floor(x)
 ** @param x argument.
 ** @return @c float(x)
 **/
inline
int32_t
fast_floor(VL::float_t x)
{
#ifdef VL_USEFASTMATH
  return (x>=0)? int32_t(x) : std::floor(x) ;
  //  return int32_t( x - ((x>=0)?0:1) ) ;
#else
  return int32_t( std::floor(x) ) ;
#endif
}

/** @brief Fast @c abs(x)
 ** @param x argument.
 ** @return @c abs(x)
 **/
inline
VL::float_t
fast_abs(VL::float_t x)
{
#ifdef VL_USEFASTMATH
  return (x >= 0) ? x : -x ;
#else
  return std::fabs(x) ;
#endif
}

/** @brief Fast @c atan2
 ** @param x argument.
 ** @param y argument.
 ** @return Approximation of @c atan2(x).
 **/
inline
VL::float_t
fast_atan2(VL::float_t y, VL::float_t x)
{
#ifdef VL_USEFASTMATH

  /*
    The function f(r)=atan((1-r)/(1+r)) for r in [-1,1] is easier to
    approximate than atan(z) for z in [0,inf]. To approximate f(r) to
    the third degree we may solve the system

     f(+1) = c0 + c1 + c2 + c3 = atan(0) = 0
     f(-1) = c0 - c1 + c2 - c3 = atan(inf) = pi/2
     f(0)  = c0                = atan(1) = pi/4

    which constrains the polynomial to go through the end points and
    the middle point.

    We still miss a constrain, which might be simply a constarint on
    the derivative in 0. Instead we minimize the Linf error in the
    range [0,1] by searching for an optimal value of the free
    parameter. This turns out to correspond to the solution

     c0=pi/4, c1=-0.9675, c2=0, c3=0.1821

    which has maxerr = 0.0061 rad = 0.35 grad.
  */

  VL::float_t angle, r ;
  VL::float_t const c3 = 0.1821 ;
  VL::float_t const c1 = 0.9675 ;
  VL::float_t abs_y    = fast_abs(y) + VL::float_t(1e-10) ;

  if (x >= 0) {
    r = (x - abs_y) / (x + abs_y) ;
    angle = VL::float_t(M_PI/4.0) ;
  } else {
    r = (x + abs_y) / (abs_y - x) ;
    angle = VL::float_t(3*M_PI/4.0) ;
  }
  angle += (c3*r*r - c1) * r ;
  return (y < 0) ? -angle : angle ;
#else
  return std::atan2(y,x) ;
#endif
}

/** @brief Fast @c resqrt
 ** @param x argument.
 ** @return Approximation to @c resqrt(x).
 **/
inline
float
fast_resqrt(float x)
{
#ifdef VL_USEFASTMATH
  // Works if VL::float_t is 32 bit ...
  union {
    float x ;
    VL::int32_t i ;
  } u ;
  float xhalf = float(0.5) * x ;
  u.x = x ;                               // get bits for floating value
  u.i = 0x5f3759df - (u.i>>1);            // gives initial guess y0
  //u.i = 0xdf59375f - (u.i>>1);          // gives initial guess y0
  u.x = u.x*(float(1.5) - xhalf*u.x*u.x); // Newton step (may repeat)
  u.x = u.x*(float(1.5) - xhalf*u.x*u.x); // Newton step (may repeat)
  return u.x ;
#else
  return float(1.0) / std::sqrt(x) ;
#endif
}

/** @brief Fast @c resqrt
 ** @param x argument.
 ** @return Approximation to @c resqrt(x).
 **/
inline
double
fast_resqrt(double x)
{
#ifdef VL_USEFASTMATH
  // Works if double is 64 bit ...
  union {
    double x ;
    VL::int64_t i ;
  } u ;
  double xhalf = double(0.5) * x ;
  u.x = x ;                                // get bits for floating value
  u.i = 0x5fe6ec85e7de30daLL - (u.i>>1);   // gives initial guess y0
  u.x = u.x*(double(1.5) - xhalf*u.x*u.x); // Newton step (may repeat)
  u.x = u.x*(double(1.5) - xhalf*u.x*u.x); // Newton step (may repeat)
  return u.x ;
#else
  return double(1.0) / std::sqrt(x) ;
#endif
}

/** @brief Fast @c sqrt
 ** @param x argument.
 ** @return Approximation to @c sqrt(x).
 **/
inline
VL::float_t
fast_sqrt(VL::float_t x)
{
#ifdef VL_USEFASTMATH
  return (x < 1e-8) ? 0 : x * fast_resqrt(x) ;
#else
  return std::sqrt(x) ;
#endif
}

}


template<typename T>
void
normalize(T* filter, int W)
{
  T  acc  = 0 ;
  T* iter = filter ;
  T* end  = filter + 2*W+1 ;
  while(iter != end) acc += *iter++ ;

  iter = filter ;
  while(iter != end) *iter++ /= acc ;
}

template<typename T>
void
convolve(T*       dst_pt,
         const T* src_pt, int M, int N,
         const T* filter_pt, int W)
{
  typedef T const TC ;
  // convolve along columns, save transpose
  // image is M by N
  // buffer is N by M
  // filter is (2*W+1) by 1
  for(int j = 0 ; j < N ; ++j) {

    int i = 0 ;

    // top
    for(; i <= std::min(W-1, M-1) ; ++i) {
      TC* start = src_pt ;
      TC* stop  = src_pt    + std::min(i+W, M-1) + 1 ;
      TC* g     = filter_pt + W-i ;
      T   acc = 0.0 ;
      while(stop != start) acc += (*g++) * (*start++) ;
      *dst_pt = acc ;
      dst_pt += N ;
    }

    // middle
    // run this for W <= i <= M-1-W, only if M >= 2*W+1
    for(; i <= M-1-W ; ++i) {
      TC* start = src_pt    + i-W ;
      TC* stop  = src_pt    + i+W + 1 ;
      TC* g     = filter_pt ;
      T   acc = 0.0 ;
      while(stop != start) acc += (*g++) * (*start++) ;
      *dst_pt = acc ;
      dst_pt += N ;
    }

    // bottom
    // run this for M-W <= i <= M-1, only if M >= 2*W+1
    for(; i <= M-1 ; ++i) {
      TC* start = src_pt    + i-W ;
      TC* stop  = src_pt    + std::min(i+W, M-1) + 1 ;
      TC* g     = filter_pt ;
      T   acc   = 0.0 ;
      while(stop != start) acc += (*g++) * (*start++) ;
      *dst_pt = acc ;
      dst_pt += N ;
    }

    // next column
    src_pt += M ;
    dst_pt -= M*N - 1 ;
  }
}

// works with symmetric filters only
template<typename T>
void
nconvolve(T*       dst_pt,
          const T* src_pt, int M, int N,
          const T* filter_pt, int W,
          T*       scratch_pt )
{
  typedef T const TC ;

  for(int i = 0 ; i <= W ; ++i) {
    T   acc = 0.0 ;
    TC* iter = filter_pt + std::max(W-i,  0) ;
    TC* stop = filter_pt + std::min(M-1-i,W) + W + 1 ;
    while(iter != stop) acc += *iter++ ;
    scratch_pt [i] = acc ;
  }

 for(int j = 0 ; j < N ; ++j) {

   int i = 0 ;
   // top margin
   for(; i <= std::min(W, M-1) ; ++i) {
     TC* start = src_pt ;
     TC* stop  = src_pt    + std::min(i+W, M-1) + 1 ;
     TC* g     = filter_pt + W-i ;
     T   acc = 0.0 ;
     while(stop != start) acc += (*g++) * (*start++) ;
     *dst_pt = acc / scratch_pt [i] ;
     dst_pt += N ;
   }

   // middle
   for(; i <= M-1-W ; ++i) {
     TC* start = src_pt    + i-W ;
     TC* stop  = src_pt    + i+W + 1 ;
     TC* g     = filter_pt ;
     T   acc = 0.0 ;
     while(stop != start) acc += (*g++) * (*start++) ;
     *dst_pt = acc ;
     dst_pt += N ;
   }

   // bottom
   for(; i <= M-1 ; ++i) {
     TC* start = src_pt    + i-W ;
     TC* stop  = src_pt    + std::min(i+W, M-1) + 1 ;
     TC* g     = filter_pt ;
     T   acc   = 0.0 ;
     while(stop != start) acc += (*g++) * (*start++) ;
     *dst_pt = acc / scratch_pt [M-1-i];
     dst_pt += N ;
   }

   // next column
   src_pt += M ;
   dst_pt -= M*N - 1 ;
 }
}

template<typename T>
void
econvolve(T*       dst_pt,
	  const T* src_pt,    int M, int N,
	  const T* filter_pt, int W)
{
  typedef T const TC ;
  // convolve along columns, save transpose
  // image is M by N
  // buffer is N by M
  // filter is (2*W+1) by 1
  for(int j = 0 ; j < N ; ++j) {
    for(int i = 0 ; i < M ; ++i) {
      T   acc = 0.0 ;
      TC* g = filter_pt ;
      TC* start = src_pt + (i-W) ;
      TC* stop  ;
      T   x ;

      // beginning
      stop = src_pt + std::max(0, i-W) ;
      x    = *stop ;
      while( start <= stop ) { acc += (*g++) * x ; start++ ; }

      // middle
      stop =  src_pt + std::min(M-1, i+W) ;
      while( start <  stop ) acc += (*g++) * (*start++) ;

      // end
      x  = *start ;
      stop = src_pt + (i+W) ;
      while( start <= stop ) { acc += (*g++) * x ; start++ ; }

      // save
      *dst_pt = acc ;
      dst_pt += N ;

      assert( g - filter_pt == 2*W+1 ) ;

    }
    // next column
    src_pt += M ;
    dst_pt -= M*N - 1 ;
  }
}

// VL_SIFT_HPP
#endif
