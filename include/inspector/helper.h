
#ifndef INSPECTOR_HELPER_H_
#define INSPECTOR_HELPER_H_

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_listener.h>

#include <cmath>

namespace helper {

double deg2rad(double deg);

double rad2deg(double rad);

geometry_msgs::Point rosvec2rospoint(geometry_msgs::Vector3 vec);

geometry_msgs::Point rostfvec2rospoint(tf::Vector3 vec);

}  // namespace helper

#endif  // INSPECTOR_HELPER_H_