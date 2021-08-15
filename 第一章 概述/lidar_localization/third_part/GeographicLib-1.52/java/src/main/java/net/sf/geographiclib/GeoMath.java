/**
 * Implementation of the net.sf.geographiclib.GeoMath class
 *
 * Copyright (c) Charles Karney (2013-2020) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
package net.sf.geographiclib;

/**
 * Mathematical functions needed by GeographicLib.
 * <p>
 * Define mathematical functions and constants so that any version of Java
 * can be used.
 **********************************************************************/
public class GeoMath {
  /**
   * The number of binary digits in the fraction of a double precision
   * number (equivalent to C++'s {@code numeric_limits<double>::digits}).
   **********************************************************************/
  public static final int digits = 53;

  /**
   * Square a number.
   * <p>
   * @param x the argument.
   * @return <i>x</i><sup>2</sup>.
   **********************************************************************/
  public static double sq(double x) { return x * x; }

  /**
   * The inverse hyperbolic tangent function.  This is defined in terms of
   * Math.log1p(<i>x</i>) in order to maintain accuracy near <i>x</i> = 0.
   * In addition, the odd parity of the function is enforced.
   * <p>
   * @param x the argument.
   * @return atanh(<i>x</i>).
   **********************************************************************/
  public static double atanh(double x)  {
    double y = Math.abs(x);     // Enforce odd parity
    y = Math.log1p(2 * y/(1 - y))/2;
    return x > 0 ? y : (x < 0 ? -y : x);
  }

  /**
   * Normalize a sine cosine pair.
   * <p>
   * @param p return parameter for normalized quantities with sinx<sup>2</sup>
   *   + cosx<sup>2</sup> = 1.
   * @param sinx the sine.
   * @param cosx the cosine.
   **********************************************************************/
  public static void norm(Pair p, double sinx, double cosx) {
    double r = Math.hypot(sinx, cosx);
    p.first = sinx/r; p.second = cosx/r;
  }

  /**
   * The error-free sum of two numbers.
   * <p>
   * @param u the first number in the sum.
   * @param v the second number in the sum.
   * @param p output Pair(<i>s</i>, <i>t</i>) with <i>s</i> = round(<i>u</i> +
   *   <i>v</i>) and <i>t</i> = <i>u</i> + <i>v</i> - <i>s</i>.
   * <p>
   * See D. E. Knuth, TAOCP, Vol 2, 4.2.2, Theorem B.
   **********************************************************************/
  public static void sum(Pair p, double u, double v) {
    double s = u + v;
    double up = s - v;
    double vpp = s - up;
    up -= u;
    vpp -= v;
    double t = -(up + vpp);
    // u + v =       s      + t
    //       = round(u + v) + t
    p.first = s; p.second = t;
  }

  /**
   * Evaluate a polynomial.
   * <p>
   * @param N the order of the polynomial.
   * @param p the coefficient array (of size <i>N</i> + <i>s</i> + 1 or more).
   * @param s starting index for the array.
   * @param x the variable.
   * @return the value of the polynomial.
   * <p>
   * Evaluate <i>y</i> = &sum;<sub><i>n</i>=0..<i>N</i></sub>
   * <i>p</i><sub><i>s</i>+<i>n</i></sub>
   * <i>x</i><sup><i>N</i>&minus;<i>n</i></sup>.  Return 0 if <i>N</i> &lt; 0.
   * Return <i>p</i><sub><i>s</i></sub>, if <i>N</i> = 0 (even if <i>x</i> is
   * infinite or a nan).  The evaluation uses Horner's method.
   **********************************************************************/
  public static double polyval(int N, double p[], int s, double x) {
    double y = N < 0 ? 0 : p[s++];
    while (--N >= 0) y = y * x + p[s++];
    return y;
  }

  /**
   * Coarsen a value close to zero.
   * <p>
   * @param x the argument
   * @return the coarsened value.
   * <p>
   * This makes the smallest gap in <i>x</i> = 1/16 &minus; nextafter(1/16, 0)
   * = 1/2<sup>57</sup> for reals = 0.7 pm on the earth if <i>x</i> is an angle
   * in degrees.  (This is about 1000 times more resolution than we get with
   * angles around 90 degrees.)  We use this to avoid having to deal with near
   * singular cases when <i>x</i> is non-zero but tiny (e.g.,
   * 10<sup>&minus;200</sup>).  This converts &minus;0 to +0; however tiny
   * negative numbers get converted to &minus;0.
   **********************************************************************/
  public static double AngRound(double x) {
    final double z = 1/16.0;
    if (x == 0) return 0;
    double y = Math.abs(x);
    // The compiler mustn't "simplify" z - (z - y) to y
    y = y < z ? z - (z - y) : y;
    return x < 0 ? -y : y;
  }

  /**
   * The remainder function.
   * <p>
   * @param x the numerator of the division
   * @param y the denominator of the division
   * @return the remainder in the range [&minus;<i>y</i>/2, <i>y</i>/2].
   * <p>
   * The range of <i>x</i> is unrestricted; <i>y</i> must be positive.
   **********************************************************************/
  public static double remainder(double x, double y) {
    x = x % y;
    return x < -y/2 ? x + y : (x < y/2 ? x : x - y);
  }

  /**
   * Normalize an angle.
   * <p>
   * @param x the angle in degrees.
   * @return the angle reduced to the range [&minus;180&deg;, 180&deg;).
   * <p>
   * The range of <i>x</i> is unrestricted.
   **********************************************************************/
  public static double AngNormalize(double x) {
    x = remainder(x, 360.0);
    return x == -180 ? 180 : x;
  }

  /**
   * Normalize a latitude.
   * <p>
   * @param x the angle in degrees.
   * @return x if it is in the range [&minus;90&deg;, 90&deg;], otherwise
   *   return NaN.
   **********************************************************************/
  public static double LatFix(double x) {
    return Math.abs(x) > 90 ? Double.NaN : x;
  }

  /**
   * The exact difference of two angles reduced to (&minus;180&deg;, 180&deg;].
   * <p>
   * @param x the first angle in degrees.
   * @param y the second angle in degrees.
   * @param p output Pair(<i>d</i>, <i>e</i>) with <i>d</i> being the rounded
   *   difference and <i>e</i> being the error.
   * <p>
   * The computes <i>z</i> = <i>y</i> &minus; <i>x</i> exactly, reduced to
   * (&minus;180&deg;, 180&deg;]; and then sets <i>z</i> = <i>d</i> + <i>e</i>
   * where <i>d</i> is the nearest representable number to <i>z</i> and
   * <i>e</i> is the truncation error.  If <i>d</i> = &minus;180, then <i>e</i>
   * &gt; 0; If <i>d</i> = 180, then <i>e</i> &le; 0.
   **********************************************************************/
  public static void AngDiff(Pair p, double x, double y) {
    sum(p, AngNormalize(-x), AngNormalize(y));
    double d = AngNormalize(p.first), t = p.second;
    sum(p, d == 180 && t > 0 ? -180 : d, t);
  }

  /**
   * Evaluate the sine and cosine function with the argument in degrees
   *
   * @param p return Pair(<i>s</i>, <i>t</i>) with <i>s</i> = sin(<i>x</i>) and
   *   <i>c</i> = cos(<i>x</i>).
   * @param x in degrees.
   * <p>
   * The results obey exactly the elementary properties of the trigonometric
   * functions, e.g., sin 9&deg; = cos 81&deg; = &minus; sin 123456789&deg;.
   **********************************************************************/
  public static void sincosd(Pair p, double x) {
    // In order to minimize round-off errors, this function exactly reduces
    // the argument to the range [-45, 45] before converting it to radians.
    double r; int q;
    r = x % 360.0;
    q = (int)Math.round(r / 90); // If r is NaN this returns 0
    r -= 90 * q;
    // now abs(r) <= 45
    r = Math.toRadians(r);
    // Possibly could call the gnu extension sincos
    double s = Math.sin(r), c = Math.cos(r);
    double sinx, cosx;
    switch (q & 3) {
    case  0: sinx =  s; cosx =  c; break;
    case  1: sinx =  c; cosx = -s; break;
    case  2: sinx = -s; cosx = -c; break;
    default: sinx = -c; cosx =  s; break; // case 3
    }
    if (x != 0) { sinx += 0.0; cosx += 0.0; }
    p.first = sinx; p.second = cosx;
  }

  /**
   * Evaluate the atan2 function with the result in degrees
   *
   * @param y the sine of the angle
   * @param x the cosine of the angle
   * @return atan2(<i>y</i>, <i>x</i>) in degrees.
   * <p>
   * The result is in the range (&minus;180&deg; 180&deg;].  N.B.,
   * atan2d(&plusmn;0, &minus;1) = +180&deg;; atan2d(&minus;&epsilon;,
   * &minus;1) = &minus;180&deg;, for &epsilon; positive and tiny;
   * atan2d(&plusmn;0, 1) = &plusmn;0&deg;.
   **********************************************************************/
  public static double atan2d(double y, double x) {
    // In order to minimize round-off errors, this function rearranges the
    // arguments so that result of atan2 is in the range [-pi/4, pi/4] before
    // converting it to degrees and mapping the result to the correct
    // quadrant.
    int q = 0;
    if (Math.abs(y) > Math.abs(x)) { double t; t = x; x = y; y = t; q = 2; }
    if (x < 0) { x = -x; ++q; }
    // here x >= 0 and x >= abs(y), so angle is in [-pi/4, pi/4]
    double ang = Math.toDegrees(Math.atan2(y, x));
    switch (q) {
      // Note that atan2d(-0.0, 1.0) will return -0.  However, we expect that
      // atan2d will not be called with y = -0.  If need be, include
      //
      //   case 0: ang = 0 + ang; break;
      //
      // and handle mpfr as in AngRound.
    case 1: ang = (y >= 0 ? 180 : -180) - ang; break;
    case 2: ang =  90 - ang; break;
    case 3: ang = -90 + ang; break;
    default: break;
    }
    return ang;
  }

  /**
   * Test for finiteness.
   * <p>
   * @param x the argument.
   * @return true if number is finite, false if NaN or infinite.
   **********************************************************************/
  public static boolean isfinite(double x) {
    return Math.abs(x) <= Double.MAX_VALUE;
  }

  /**
   * Normalize a sine cosine pair.
   * <p>
   * @param sinx the sine.
   * @param cosx the cosine.
   * @return a Pair of normalized quantities with sinx<sup>2</sup> +
   *   cosx<sup>2</sup> = 1.
   *
   * @deprecated Use {@link #sincosd(Pair, double)} instead.
   **********************************************************************/
  // @Deprecated
  public static Pair norm(double sinx, double cosx)
  { Pair p = new Pair(); norm(p, sinx, cosx); return p; }

  /**
   * The error-free sum of two numbers.
   * <p>
   * @param u the first number in the sum.
   * @param v the second number in the sum.
   * @return Pair(<i>s</i>, <i>t</i>) with <i>s</i> = round(<i>u</i> +
   *   <i>v</i>) and <i>t</i> = <i>u</i> + <i>v</i> - <i>s</i>.
   * <p>
   * See D. E. Knuth, TAOCP, Vol 2, 4.2.2, Theorem B.
   *
   * @deprecated Use {@link #sincosd(Pair, double)} instead.
   **********************************************************************/
  // @Deprecated
  public static Pair sum(double u, double v)
  { Pair p = new Pair(); sum(p, u, v); return p; }

  /**
   * The exact difference of two angles reduced to (&minus;180&deg;, 180&deg;].
   * <p>
   * @param x the first angle in degrees.
   * @param y the second angle in degrees.
   * @return Pair(<i>d</i>, <i>e</i>) with <i>d</i> being the rounded
   *   difference and <i>e</i> being the error.
   * <p>
   * The computes <i>z</i> = <i>y</i> &minus; <i>x</i> exactly, reduced to
   * (&minus;180&deg;, 180&deg;]; and then sets <i>z</i> = <i>d</i> + <i>e</i>
   * where <i>d</i> is the nearest representable number to <i>z</i> and
   * <i>e</i> is the truncation error.  If <i>d</i> = &minus;180, then <i>e</i>
   * &gt; 0; If <i>d</i> = 180, then <i>e</i> &le; 0.
   *
   * @deprecated Use {@link #sincosd(Pair, double)} instead.
   **********************************************************************/
  // @Deprecated
  public static Pair AngDiff(double x, double y)
  { Pair p = new Pair(); AngDiff(p, x, y); return p; }

  /**
   * Evaluate the sine and cosine function with the argument in degrees
   *
   * @param x in degrees.
   * @return Pair(<i>s</i>, <i>t</i>) with <i>s</i> = sin(<i>x</i>) and
   *   <i>c</i> = cos(<i>x</i>).
   * <p>
   * The results obey exactly the elementary properties of the trigonometric
   * functions, e.g., sin 9&deg; = cos 81&deg; = &minus; sin 123456789&deg;.
   *
   * @deprecated Use {@link #sincosd(Pair, double)} instead.
   **********************************************************************/
  // @Deprecated
  public static Pair sincosd(double x)
  { Pair p = new Pair(); sincosd(p, x); return p; }

  private GeoMath() {}
}
