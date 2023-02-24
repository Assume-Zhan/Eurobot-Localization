#pragma once

#include <cmath>

namespace lidar_localization
{
template <class T, class S>
double length(T& pose1, S& pose2)
{
  return sqrt(pow(pose1.x - pose2.x, 2.) + pow(pose1.y - pose2.y, 2.));
}

template <class T>
double length(T& pose1)
{
  return sqrt(pow(pose1.x, 2.) + pow(pose1.y, 2.));
}

double length(double x, double y)
{
  return sqrt(pow(x, 2.) + pow(y, 2.));
}

inline bool is_whthin_tolerance(double measure, double expect, double tolerance)
{
  return abs(measure - expect) < tolerance;
}

inline double radian_to_degree(double r)
{
  return r / M_PI * 180;
}

}  // namespace lidar_localization
