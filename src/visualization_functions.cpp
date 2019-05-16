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

void SelectColor(const uint &i, std_msgs::ColorRGBA *color) {
  const uint n_colors = 9;
  const uint index = i % n_colors;  // Returns a number between 0 and 8
  switch(index) {
    case 0: *color = Color::Cyan(); break;
    case 1: *color = Color::Purple(); break;
    case 2: *color = Color::Chartreuse(); break;
    case 3: *color = Color::Teal(); break;
    case 4: *color = Color::Pink(); break;
  }
}

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
  DrawArrowPoints(helper::eigenvec2rospoint(p1), helper::eigenvec2rospoint(p2),
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

void drawTrajectory(const mg_msgs::PVAJS_array &flatStates, 
                    const std::string &frame_id,
                    const std::string &ns,
                    const std_msgs::ColorRGBA &color,
                    visualization_msgs::MarkerArray* marker_array) {
  // CHECK_NOTNULL(marker_array);
  marker_array->markers.clear();

  visualization_msgs::Marker line_strip;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.color = color;
  line_strip.scale.x = 0.01;
  line_strip.ns = ns;

  double min_displacement = 0.025;

  Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_appended_position = Eigen::Vector3d::Zero();
  geometry_msgs::Point last_position_msg;

  // Add first point to line strip
  last_appended_position = helper::rospoint2eigenvec(flatStates.PVAJS_array[0].Pos);
  line_strip.points.push_back(flatStates.PVAJS_array[0].Pos);
  uint n_points = flatStates.PVAJS_array.size();
  for (size_t i = 0; i < n_points; ++i) {
    // const mav_msgs::EigenTrajectoryPoint& flat_state = flat_states[i];
    Eigen::Vector3d cur_position = helper::rospoint2eigenvec(flatStates.PVAJS_array[i].Pos);
    
    if ((cur_position - last_appended_position).norm() > min_displacement) {
      last_appended_position = cur_position;
      line_strip.points.push_back(flatStates.PVAJS_array[i].Pos);
    }

    if(line_strip.points.size() > 16300) {
      marker_array->markers.push_back(line_strip);
      line_strip.points.clear();
      line_strip.id = line_strip.id + 1;
    }
  }
  line_strip.points.push_back(flatStates.PVAJS_array[n_points-1].Pos);
  marker_array->markers.push_back(line_strip);
  
  // Set marker headers
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  for (uint i = 0; i < marker_array->markers.size(); i++) {
    marker_array->markers[i].header = header;
    marker_array->markers[i].action = visualization_msgs::Marker::ADD;
    marker_array->markers[i].id = i;
    marker_array->markers[i].lifetime = ros::Duration(0.0);
  }
}

void drawTrajectory(const std::vector<Eigen::Vector2d> &trajectory,
                    const double &height,
                    const std::string &frame_id,
                    const std::string &ns,
                    const std_msgs::ColorRGBA &color,
                    visualization_msgs::MarkerArray* marker_array) {
  marker_array->markers.clear();

  visualization_msgs::Marker line_strip;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.color = color;
  line_strip.scale.x = 0.01;
  line_strip.ns = ns;

  double min_displacement = 0.025;

  Eigen::Vector2d last_position = Eigen::Vector2d::Zero();
  Eigen::Vector2d last_appended_position = Eigen::Vector2d::Zero();

  // Add first point to line strip
  last_appended_position = trajectory[0];
  line_strip.points.push_back(helper::eigenvec2d2rospoint(trajectory[0], height));
  uint n_points = trajectory.size();
  for (size_t i = 0; i < n_points; ++i) {
    // const mav_msgs::EigenTrajectoryPoint& flat_state = flat_states[i];
    Eigen::Vector2d cur_position = trajectory[i];
    
    if ((cur_position - last_appended_position).norm() > min_displacement) {
      last_appended_position = cur_position;
      line_strip.points.push_back(helper::eigenvec2d2rospoint(trajectory[i], height));
    }

    if(line_strip.points.size() > 16300) {
      marker_array->markers.push_back(line_strip);
      line_strip.points.clear();
      line_strip.id = line_strip.id + 1;
    }
  }
  line_strip.points.push_back(helper::eigenvec2d2rospoint(trajectory[n_points-1], height));
  marker_array->markers.push_back(line_strip);
  
  // Set marker headers
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  for (uint i = 0; i < marker_array->markers.size(); i++) {
    marker_array->markers[i].header = header;
    marker_array->markers[i].action = visualization_msgs::Marker::ADD;
    marker_array->markers[i].id = i;
    marker_array->markers[i].lifetime = ros::Duration(0.0);
  }
}

void drawWaypoints(const nav_msgs::Path &Waypoints, 
                   const std::string& frame_id,
                   visualization_msgs::MarkerArray* marker_array) {
  marker_array->markers.clear();

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color = Color::Red();
  marker.scale.x = 0.025;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;
  marker.ns = "path";
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.pose.orientation.w = 1.0;

  //Get the number of requested waypoints
  int n_w = Waypoints.poses.size();

  for (size_t i = 0; i < n_w; ++i) {
    geometry_msgs::Point last_position_msg = Waypoints.poses[i].pose.position;
    marker.pose.position = last_position_msg;
    marker.header.seq = i;
    marker_array->markers.push_back(marker);
  }
  

  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  for (uint i = 0; i < marker_array->markers.size(); i++) {
    marker_array->markers[i].header = header;
    marker_array->markers[i].action = visualization_msgs::Marker::ADD;
    marker_array->markers[i].id = i;
    marker_array->markers[i].lifetime = ros::Duration(0.0);
  }
}


void drawWaypoints(const std::vector<Eigen::Vector2d> &Waypoints,
                   const double &height,
                   const std::string& frame_id,
                   visualization_msgs::MarkerArray* marker_array) {
  marker_array->markers.clear();

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color = Color::Red();
  marker.scale.x = 0.025;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;
  marker.ns = "path";
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.pose.orientation.w = 1.0;

  //Get the number of requested waypoints
  int n_w = Waypoints.size();

  for (size_t i = 0; i < n_w; ++i) {
    geometry_msgs::Point last_position_msg = 
        helper::eigenvec2d2rospoint(Waypoints[i], height);
    marker.pose.position = last_position_msg;
    marker.header.seq = i;
    marker_array->markers.push_back(marker);
  }
  

  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  for (uint i = 0; i < marker_array->markers.size(); i++) {
    marker_array->markers[i].header = header;
    marker_array->markers[i].action = visualization_msgs::Marker::ADD;
    marker_array->markers[i].id = i;
    marker_array->markers[i].lifetime = ros::Duration(0.0);
  }
}

void deleteMarkersTemplate(const std::string &frame_id,
                           visualization_msgs::MarkerArray* marker_array) {
  visualization_msgs::Marker deleteMarker;
  deleteMarker.action = deleteMarker.DELETEALL;
  deleteMarker.header.frame_id = frame_id;
  marker_array->markers.push_back(deleteMarker);
}

}  // namespace visualization_functions
