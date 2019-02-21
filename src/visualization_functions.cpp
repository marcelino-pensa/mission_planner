/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <string>
#include <vector>
#include <set>
#include "mission_planner/visualization_functions.h"

namespace visualization_functions {

void DrawArrowPoints(const geometry_msgs::Point& p1,
                     const geometry_msgs::Point& p2,
                     const std_msgs::ColorRGBA &color,
                     const std::string &frame_id,
                     const std::string &ns,
                     const uint &sequence_number,
                     const double &diameter,
                     visualization_msgs::Marker* marker) {
  marker->type = visualization_msgs::Marker::ARROW;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = color;

  marker->points.resize(2);
  marker->points[0] = p1;
  marker->points[1] = p2;
  // EigenPoint2RosPoint(p1, &marker->points[0]);
  // EigenPoint2RosPoint(p2, &marker->points[1]);

  marker->scale.x = diameter;
  marker->scale.y = diameter * 2;
  marker->scale.z = 0;
  marker->pose.orientation.w = 1.0;

  marker->ns = ns;
  marker->header.frame_id = frame_id;
  marker->header.stamp = ros::Time::now();
  marker->header.seq = 0;
  marker->id = sequence_number;
}

void DrawArrowPoints(const Eigen::Vector3d& p1,
                     const Eigen::Vector3d& p2,
                     const std_msgs::ColorRGBA &color,
                     const std::string &frame_id,
                     const std::string &ns,
                     const uint &sequence_number,
                     const double &diameter,
                     visualization_msgs::Marker* marker) {
  DrawArrowPoints(msg_conversions::eigen_to_ros_point(p1), msg_conversions::eigen_to_ros_point(p2),
                  color, frame_id, ns, sequence_number, diameter, marker);
}

void DeleteMarkersTemplate(const std::string &frame_id,
                           visualization_msgs::MarkerArray *marker_array) {
  visualization_msgs::Marker deleteMarker;
  deleteMarker.action = deleteMarker.DELETEALL;
  deleteMarker.scale.x = 0.1;
  deleteMarker.scale.y = 0.1;
  deleteMarker.scale.z = 0.1;
  deleteMarker.header.frame_id = frame_id;
  deleteMarker.ns = "";
  marker_array->markers.push_back(deleteMarker);
}

}  // namespace visualization_functions
