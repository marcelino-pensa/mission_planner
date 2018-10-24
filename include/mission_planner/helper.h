
#ifndef MISSION_HELPER_H_
#define MISSION_HELPER_H_

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include "mg_msgs/PVAJ_request.h"

#include <cmath>

namespace helper {

double deg2rad(const double &deg);

double rad2deg(const double &rad);

geometry_msgs::Point rosvec2rospoint(const geometry_msgs::Vector3 &vec);

geometry_msgs::Point rostfvec2rospoint(const tf::Vector3 &vec);

geometry_msgs::Point eigenvec2rospoint(const Eigen::Vector3d &vec);

geometry_msgs::Vector3 eigenvec2rosvec(const Eigen::Vector3d &vec);

Eigen::Vector3d rostfvec2eigenvec(const tf::Vector3 &vec);

geometry_msgs::Quaternion set_quat(const double &roll, const double &pitch, const double &yaw);

geometry_msgs::Quaternion set_quat(const double &w, const double &x, const double &y, const double &z);

mg_msgs::PVAJ_request get_empty_PVAJ();

std::vector<std::pair<uint, uint>> split_waypoints(const uint &n_waypoints,
	                                               const uint &wp_per_segment);

}  // namespace helper

#endif  // MISSION_HELPER_H_