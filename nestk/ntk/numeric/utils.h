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

#ifndef   	NTK_UTILS_MATH_H_
# define   	NTK_UTILS_MATH_H_

# include <ntk/core.h>

#define NTK_UNSAFE_MAX(a,b) ((a)<(b)?(b):(a))

namespace ntk {

  // Float comparisons.

  inline bool
  flt_eq (float lhs, float rhs,
          float epsilon = std::numeric_limits<float>::epsilon())
  {
    return std::abs(lhs - rhs) <= epsilon;
  }

  inline bool
  flt_leq (float lhs, float rhs,
           float epsilon = std::numeric_limits<float>::epsilon())
  {
    return lhs <= (rhs+epsilon);
  }

  inline bool
  flt_geq (float lhs, float rhs,
           float epsilon = std::numeric_limits<float>::epsilon())
  {
    return (lhs+epsilon) >= rhs;
  }

  template <class T, class U>
  inline bool
  in_range(T lower, U data, T upper)
  {
    return (data >= lower) && (data <= upper);
  }

  inline unsigned
  add_and_saturate(unsigned a, unsigned b, unsigned max_value)
  {
    if ((a + b) > max_value)
      return max_value;
    return a + b;
  }

  long greater_power_of_two(double value);

  double normalize_unoriented_angle_0_pi(double angle);
  double normalize_angle_0_2pi(double angle);
  inline double rad_to_deg(double rad_angle) { return (rad_angle*180.0)/M_PI; }
  inline double deg_to_rad(double deg_angle) { return (deg_angle*M_PI)/180.0; }

  double erfcx(double x);
#ifdef _MSC_VER
  inline double erfc(double x)
  { abort(); /* not implemented. */; return 0; }
#endif

  // From codecogs : http://www.codecogs.com/
  double erfinv(double y);
  inline double erfcinv(double x) { return erfinv(1.0-x); }

  inline double norminv(double p) { return -sqrt(2.0) * erfcinv(2.0*p); }
  inline double norminv(double p, double mean, double dev) { return norminv(p)*dev + mean; }
  double normcdf(double x, double mean, double std_dev);
  double normpdf(double x, double mean, double dev);
  double lnnormcdf(double x, double mean, double std_dev);
  double lnnormicdf(double x, double mean, double std_dev);

  double lnchoosek(unsigned n, unsigned k);

  inline double sigmoid(double x) { return 1.0/(1.0+exp(-x)); }

  // Returns an upper bound of log(B(k, n, p))
  double binomial_logicdf_hoeffding_estimate(unsigned k, double p, unsigned n);

  template <class T>
  T saturate_to_range(T value, T min_value, T max_value)
  {
    if (value < min_value) value = min_value;
    else if (value > max_value) value = max_value;
    return value;
  }

#ifdef USE_GSL
  double binomial_logcdf_gsl(unsigned k, double p, unsigned n);
  double binomial_logicdf_gsl(unsigned k, double p, unsigned n);
  double binomial_logicdf_best_estimate(unsigned k, double p, unsigned n);
#endif // def USE_GSL

} // end of ntk

namespace ntk
{

  namespace math
  {
    //: pi, e and all that
    static  const double e               =2.7182818284590452354;
    static  const double log2e           =1.4426950408889634074;
    static  const double log10e          =0.43429448190325182765;
    static  const double ln2             =0.69314718055994530942;
    static  const double ln10            =2.30258509299404568402;
    static  const double pi              =3.14159265358979323846;
    static  const double pi_over_2       =1.57079632679489661923;
    static  const double pi_over_4       =0.78539816339744830962;
    static  const double one_over_pi     =0.31830988618379067154;
    static  const double two_over_pi     =0.63661977236758134308;
    static  const double two_over_sqrtpi =1.12837916709551257390;
    static  const double sqrt2           =1.41421356237309504880;
    static  const double sqrt1_2         =0.70710678118654752440;

    //: IEEE double machine precision
    static  const double eps             =2.2204460492503131e-16;
    static  const double sqrteps         =1.490116119384766e-08;
    //: IEEE single machine precision
    static  const float float_eps        =1.192092896e-07f;
    static  const float float_sqrteps    =3.4526698307e-4f;

    // rnd (rounding; 0.5 rounds up)
    // Use C99 functions, which GCC implements as an intrinsic
    // Or in simpler terms - is at least 3 times faster.
#ifdef _MSC_VER
	inline int rnd(float  x) { return floor(x + 0.5f); }
    inline int rnd(double x) { return floor(x + 0.5); }
#else
	inline int rnd(float  x) { return lroundf(x); }
    inline int rnd(double x) { return lround(x); }
#endif 

    // floor -- round towards minus infinity
    inline int floor(float  x) { return static_cast<int>(x>=0.f?x:(x==static_cast<int>(x)?x:x-1.f)); }
    inline int floor(double x) { return static_cast<int>(x>=0.0?x:(x==static_cast<int>(x)?x:x-1.0)); }
    inline int floor(int x) { return x; }

    // ceil -- round towards plus infinity
    inline int ceil(float  x) { return static_cast<int>(x<0.f?x:(x==static_cast<int>(x)?x:x+1.f)); }
    inline int ceil(double x) { return static_cast<int>(x<0.0?x:(x==static_cast<int>(x)?x:x+1.0)); }
    inline int ceil(int x) { return x; }

    // abs
    inline bool           abs(bool x)          { return x; }
    inline unsigned char  abs(unsigned char x) { return x; }
    inline unsigned char  abs(signed char x)   { return x < 0 ? -x : x; }
    inline unsigned char  abs(char x)          { return (unsigned char)x; }
    inline unsigned short abs(short x)         { return x < 0 ? -x : x; }
    inline unsigned short abs(unsigned short x){ return x; }
    inline unsigned int   abs(int x)           { return x < 0 ? -x : x; }
    inline unsigned int   abs(unsigned int x)  { return x; }
    inline unsigned long  abs(long x)          { return x < 0L ? -x : x; }
    inline unsigned long  abs(unsigned long x) { return x; }
    inline float          abs(float x)         { return x < 0.0f ? -x : x; }
    inline double         abs(double x)        { return x < 0.0 ? -x : x; }
    inline long double    abs(long double x)   { return x < 0.0 ? -x : x; }

    // max
    inline int           max(int x, int y)                     { return (x > y) ? x : y; }
    inline unsigned int  max(unsigned int x, unsigned int y)   { return (x > y) ? x : y; }
    inline long          max(long x, long y)                   { return (x > y) ? x : y; }
    inline unsigned long max(unsigned long x, unsigned long y) { return (x > y) ? x : y;}
    inline float         max(float x, float y)                 { return (x < y) ? y : x; }
    inline double        max(double x, double y)               { return (x < y) ? y : x; }

    // min
    inline int           min(int x, int y)                     { return (x < y) ? x : y; }
    inline unsigned int  min(unsigned int x, unsigned int y)   { return (x < y) ? x : y; }
    inline long          min(long x, long y)                   { return (x < y) ? x : y; }
    inline unsigned long min(unsigned long x, unsigned long y) { return (x < y) ? x : y;}
    inline float         min(float x, float y)                 { return (x > y) ? y : x; }
    inline double        min(double x, double y)               { return (x > y) ? y : x; }

    // sqr (square)
    inline bool          sqr(bool x)          { return x; }
    inline int           sqr(int x)           { return x*x; }
    inline unsigned int  sqr(unsigned int x)  { return x*x; }
    inline long          sqr(long x)          { return x*x; }
    inline unsigned long sqr(unsigned long x) { return x*x; }
    inline float         sqr(float x)         { return x*x; }
    inline double        sqr(double x)        { return x*x; }

    // cube
    inline bool          cube(bool x)          { return x; }
    inline int           cube(int x)           { return x*x*x; }
    inline unsigned int  cube(unsigned int x)  { return x*x*x; }
    inline long          cube(long x)          { return x*x*x; }
    inline unsigned long cube(unsigned long x) { return x*x*x; }
    inline float         cube(float x)         { return x*x*x; }
    inline double        cube(double x)        { return x*x*x; }
  } // end of math

} // end of ntk

#endif	    /* !NTK_UTILS_MATH_H_ */
