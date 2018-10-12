
#ifndef INSPECTOR_HELPER_H_
#define INSPECTOR_HELPER_H_

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_listener.h>
#include <Eigen/Dense>

#include <cmath>

namespace helper {

double deg2rad(double deg);

double rad2deg(double rad);

geometry_msgs::Point rosvec2rospoint(geometry_msgs::Vector3 vec);

geometry_msgs::Point rostfvec2rospoint(tf::Vector3 vec);

geometry_msgs::Point eigenvec2rospoint(Eigen::Vector3d vec);

Eigen::Vector3d rostfvec2eigenvec(tf::Vector3 vec);

geometry_msgs::Quaternion set_quat(double roll, double pitch, double yaw);

geometry_msgs::Quaternion set_quat(double w, double x, double y, double z);

}  // namespace helper

#endif  // INSPECTOR_HELPER_H_