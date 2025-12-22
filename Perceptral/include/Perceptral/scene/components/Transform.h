#pragma once
#include <Eigen/Eigen>
#include <Perceptral/core/Macros.h>

namespace Perceptral {
namespace Component {

struct PC_API Transform {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
  Eigen::Vector3f scale = Eigen::Vector3f::Ones();

  Transform() = default;
  explicit Transform(const Eigen::Vector3f &pos) : position(pos) {}

  Eigen::Matrix4f getTransform() const;

  Eigen::Vector3f forward() const;
  Eigen::Vector3f right() const;
  Eigen::Vector3f up() const;
};
} // namespace Component
} // namespace Perceptral
