/*
 * vector.hpp
 *
 *  Created on: Sep 16, 2015
 *      Author: anhnt
 */

#ifndef WANDRIAN_INCLUDE_COMMON_VECTOR_HPP_
#define WANDRIAN_INCLUDE_COMMON_VECTOR_HPP_

#include "point.hpp"

namespace wandrian {
namespace common {

enum Orientation {
  //                      IN_FRONT (1)
  //                           |
  //                        ___|___
  //                       |   |   |
  // AT_LEFT_SIDE (2) _____|___|___|_____ AT_RIGHT_SIDE (0)
  //                       |   |   |
  //                       |___|___|
  //                           |
  //                           |
  //                      IN_BACK (3)

  AT_RIGHT_SIDE,
  IN_FRONT,
  AT_LEFT_SIDE,
  IN_BACK
};

inline Orientation operator+(Orientation o) {
  switch (o) {
  case AT_RIGHT_SIDE:
    return IN_FRONT;
  case IN_FRONT:
    return AT_LEFT_SIDE;
  case AT_LEFT_SIDE:
    return IN_BACK;
  case IN_BACK:
    return AT_RIGHT_SIDE;
  }
  return o;
}

inline Orientation& operator++(Orientation &o) {
  o = +o;
  return o;
}

struct Vector {

  double x, y;

  Vector();
  Vector(double);
  Vector(double, double);
  Vector(const Vector&);
  Vector(const boost::shared_ptr<Vector>);

  void rotate_counterclockwise();
  void rotate_clockwise();

  double get_magnitude();
  double get_angle();
};

typedef boost::shared_ptr<Vector> VectorPtr;

inline VectorPtr operator*(VectorPtr v, double k) {
  return VectorPtr(new Vector(v->x * k, v->y * k));
}

inline VectorPtr operator/(VectorPtr v, double k) {
  return VectorPtr(new Vector(v->x / k, v->y / k));
}

inline PointPtr operator+(PointPtr p, VectorPtr v) {
  return PointPtr(new Point(p->x + v->x, p->y + v->y));
}

inline VectorPtr operator-(PointPtr p1, PointPtr p2) {
  return VectorPtr(new Vector(p1->x - p2->x, p1->y - p2->y));
}

inline double operator^(VectorPtr v1, VectorPtr v2) {
  double a1 = std::atan2(v1->y, v1->x) - std::atan2(v2->y, v2->x);
  double a2 = (a1 > 0) ? a1 - 2 * M_PI : a1 + 2 * M_PI;
  return (std::abs(a1) < std::abs(a2)) ? a1 : a2;
}

inline Orientation operator%(VectorPtr v1, VectorPtr v2) {
  double angle = v1 ^ v2;
  if (std::abs(angle) >= 3 * M_PI_4)
    return IN_BACK;
  else if (std::abs(angle) <= M_PI_4)
    return IN_FRONT;
  else if (angle > 0)
    return AT_LEFT_SIDE;
  else
    return AT_RIGHT_SIDE;
}

inline VectorPtr operator+(VectorPtr v) {
  return VectorPtr(new Vector(-v->y, v->x));
}

inline VectorPtr operator++(VectorPtr v) {
  v->rotate_counterclockwise();
  return v;
}

inline VectorPtr operator++(VectorPtr v, int) {
  VectorPtr vector = VectorPtr(new Vector(*v));
  v->rotate_counterclockwise();
  return vector;
}

inline VectorPtr operator-(VectorPtr v) {
  return VectorPtr(new Vector(v->y, -v->x));
}

inline VectorPtr operator--(VectorPtr v) {
  v->rotate_clockwise();
  return v;
}

inline VectorPtr operator--(VectorPtr v, int) {
  VectorPtr vector = VectorPtr(new Vector(*v));
  v->rotate_clockwise();
  return vector;
}

inline Orientation operator~(VectorPtr v) {
  return v % VectorPtr(new Vector());
}

inline VectorPtr operator~(Orientation o) {
  switch (o) {
  case AT_RIGHT_SIDE:
    return VectorPtr(new Vector(1, 0));
  case IN_FRONT:
    return VectorPtr(new Vector(0, 1));
  case AT_LEFT_SIDE:
    return VectorPtr(new Vector(-1, 0));
  case IN_BACK:
    return VectorPtr(new Vector(0, -1));
  default:
    return VectorPtr(new Vector());
  }
}

}
}

#endif /* WANDRIAN_INCLUDE_COMMON_VECTOR_HPP_ */
