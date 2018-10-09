
#ifndef INSPECTOR_STRUCTS_H_
#define INSPECTOR_STRUCTS_H_

namespace inspector {

class xyz_heading {
 public:
	float x_;
	float y_;
	float z_;
	float yaw_;

	xyz_heading();

	xyz_heading(double x, double y, double z, double yaw) {
		x_ = x; y_ = y; z_ = z; yaw_ = yaw;
	}
};

}  // namespace inspector

#endif  // MAPPER_STRUCTS_H_