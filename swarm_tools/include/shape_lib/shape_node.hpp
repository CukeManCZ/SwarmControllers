#pragma once

#include <Eigen/Dense>

namespace shape_lib
{

struct ShapeNode {
  double size;
  Eigen::Vector3d position;
  int value;
};

} // namespace shape_lib