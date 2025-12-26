#pragma once
#include <Eigen/Eigen>
#include <Perceptral/core/Macros.h>

namespace Perceptral {
namespace Component {

struct PC_API Transform {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
  Eigen::Vector3f scale = Eigen::Vector3f::Ones();
};
} // namespace Component
} // namespace Perceptral
