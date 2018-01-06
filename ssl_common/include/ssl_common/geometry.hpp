/* This file contains the template class Vector2D which  implements
 * all major functions for point type objects with two fields x and y.
 * It also implements overloading of the following operators : -,+,*,/
 * for these objects.
 */

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cmath>

#define PI  (3.14159265358979323f)
#define INF (9999999)

// Vector2D can also be called as Point2D
#define Point2D Vector2D

// Function definitions
template <class T>
class Vector2D
{
public:
  T x;
  T y;

  bool valid(void) const
  {
    return (x != -INF && y != -INF && x != INF && y != INF);
  }

  inline Vector2D() :
    x(-INF),
    y(-INF)
  { }

  inline Vector2D(T x, T y) :
    x(x),
    y(y)
  { }

  // Copy constructor
  inline Vector2D(const Vector2D<T>& v) :
    x(v.x),
    y(v.y)
  { }

  // Get Invalid Vector2D
  static inline Vector2D<T> InvalidVector(void)
  {
    return Vector2D<T>();
  }

  // Sets the vector using polar coordinates
  static inline const Vector2D<T> fromPolar(float r, float theta)
  {
    Vector2D<T> v;
    v.x = r * cos(theta);
    v.y = r * sin(theta);
    return v;
  }

  // Returns the angle made by the vector (head - tail) in the range -pi to pi
  static inline float angle(const Vector2D<T>& head, const Vector2D<T>& tail)
  {
    return atan2((float)head.y - tail.y, (float)head.x - tail.x);
  }

  // Returns the Eucledian distance between the 2 vectors
  static inline float dist(const Vector2D<T>& v1, const Vector2D<T>& v2)
  {
    return sqrt((float)(v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y));
  }

  // Returns the squared Eucledian distane between 2 vectors. Prefer it over dist() for speed improvements
  static inline T distSq(const Vector2D<T>& v1, const Vector2D<T>& v2)
  {
    return ((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y));
  }

  // Returns the absolute value of the vector
  inline float abs(void) const
  {
    return sqrt((float)x * x + y * y);
  }

  // Returns the squared absolute value of the vector. Prefer if over abs() for speed improvements
  inline T absSq(void) const
  {
    return (x * x + y * y);
  }

  inline Vector2D<T>& operator += (const Vector2D<T>& rhs)
  {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }

  inline Vector2D<T>& operator -= (const Vector2D<T>& rhs)
  {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }

  inline T dot(const Vector2D<T>& rhs)
  {
    return x * rhs.x + y * rhs.y;
  }
};

// Binary minus operator overloading
template <class T>
inline Vector2D<T> operator - (const Vector2D<T>& lhs, const Vector2D<T>& rhs)
{
  Vector2D<T> result = lhs;
  result.x = result.x - rhs.x;
  result.y = result.y - rhs.y;
  return result;
}

// Binary plus operator overloading
template <class T>
inline Vector2D<T> operator + (const Vector2D<T>& lhs, const Vector2D<T>& rhs)
{
  Vector2D<T> result = lhs;
  result.x = result.x + rhs.x;
  result.y = result.y + rhs.y;
  return result;
}

// Binary * (for scaler product) operator overloading
template <class T>
inline Vector2D<T> operator * (float scale, const Vector2D<T>& rhs)
{
  Vector2D<T> result;
  result.x = scale * rhs.x;
  result.y = scale * rhs.y;
  return result;
}

// Binary * (for scaler product) operator overloading
template <class T>
inline Vector2D<T> operator * (const Vector2D<T>& lhs, float scale)
{
  Vector2D<T> result;
  result.x = scale * lhs.x;
  result.y = scale * lhs.y;
  return result;
}

// Binary / (for scaler division) operator overloading
template <class T>
inline Vector2D<T> operator / (const Vector2D<T>& lhs, float scaleInv)
{
  Vector2D<T> result;
  result.x = (1.0f / scaleInv) * lhs.x;
  result.y = (1.0f / scaleInv) * lhs.y;
  return result;
}

// Binary == (equality) operator overloading
template <class T>
inline bool operator == (const Vector2D<T>& lhs, const Vector2D<T>& rhs)
{
  return (lhs.x == rhs.x && lhs.y == rhs.y);
}

// Binary != (equality) operator overloading
template <class T>
inline bool operator != (const Vector2D<T>& lhs, const Vector2D<T>& rhs)
{
  return (lhs.x != rhs.x || lhs.y != rhs.y);
}

// Normalizes the angle (in radians) to be in the range (-pi, pi]
inline float normalizeAngle(float angle)
{
  if (angle > PI)
    return angle - 2 * PI;
  else if (angle <= -PI)
    return angle + 2 * PI;
  return angle;
}

// Detects if a point is within a circle or not
template <class T>
inline bool intersects(const Point2D<int>& point,
                       const Vector2D<T>& center,
                       T radius)
{
  return (Vector2D<T>::distSq(point, center) < radius * radius);
}

// Detects if a line segment intersects a circle or not
template <class T>
inline bool intersects(const Point2D<T>& point1,
                       const Point2D<T>& point2,
                       const Point2D<T>& center,
                       T radius)
{
  /* Source of algorithm used: http://stackoverflow.com/questions/1073336/circle-line-collision-detection */

  Vector2D<int> d = point2 - point1;
  Vector2D<int> f = point1 - center;

  float a = d.dot(d);
  float b = 2 * f.dot(d);
  float c = f.dot(f) - radius * radius;
  float dis = b * b - 4 * a * c;

  if (dis >= 0)
  {
    dis = sqrt(dis);
    float t1 = (-b + dis) / (2 * a);
    float t2 = (-b - dis) / (2 * a);
    if (t1 >= 0 && t1 <= 1)
      return true;
    if (t2 >= 0 && t2 <= 1)
      return true;
  }
  return false;
}

#endif // GEOMETRY_H
