#ifndef XYZHEADING_CLASS_H_
#define XYZHEADING_CLASS_H_

#include <mission_planner/helper.h>
#include <Eigen/Dense>

namespace mission_planner {

class xyz_heading {
 public:
	float x_;
	float y_;
	float z_;
	float yaw_;

	xyz_heading() {};

	xyz_heading(double x, double y, double z, double yaw) {
		x_ = x; y_ = y; z_ = z; yaw_ = yaw;
	}

	xyz_heading(Eigen::Vector3d xyz, double yaw) {
		x_ = xyz[0]; y_ = xyz[1]; z_ = xyz[2]; yaw_ = yaw;
	}

	xyz_heading(tf::StampedTransform transform) {
		Eigen::Vector3d pos = helper::rostfvec2eigenvec(transform.getOrigin());
		double roll, pitch, yaw;
		transform.getBasis().getRPY(roll, pitch, yaw);
		x_ = pos[0]; y_ = pos[1]; z_ = pos[2]; yaw_ = yaw;
	}

	geometry_msgs::Point GetXYZ() const {
		geometry_msgs::Point Pt;
		Pt.x = x_; Pt.y = y_; Pt.z = z_;
		return Pt;
	}

	double GetYaw() const {
		return yaw_;
	}

	Eigen::Vector3d GetEigenXYZ() const {
		return Eigen::Vector3d(x_, y_, z_);
	}

	double GetXYZDist(Eigen::Vector3d vector2) const {
		return (this->GetEigenXYZ()  - vector2).norm();
	}

	double GetXYZDist(xyz_heading vector2) const {
		return (this->GetEigenXYZ()  - vector2.GetEigenXYZ()).norm();
	}

	// void transformWaypoints(tf::StampedTransform pose) {
	// 	tf::Vector3 pos(x_, y_, z_);
	// }
};

} // namespace mission_planner

#endif  // XYZHEADING_CLASS_H_