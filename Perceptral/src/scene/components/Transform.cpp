#include <Eigen/Geometry>
#include <Perceptral/scene/components/Transform.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Perceptral {

namespace Component {

inline Eigen::Vector3f Transform::forward() const {
  return rotation * Eigen::Vector3f(0.f, 0.f, -1.f);
}

inline Eigen::Vector3f Transform::right() const {
  return rotation * Eigen::Vector3f(1.f, 0.f, 0.f);
}

inline Eigen::Vector3f Transform::up() const {
  return rotation * Eigen::Vector3f(0.f, 1.f, 0.f);
}

Eigen::Matrix4f Transform::getTransform() const {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  transform.block<3, 3>(0, 0) =
      rotation.toRotationMatrix() * scale.asDiagonal();

  transform.block<3, 1>(0, 3) = position;

  return transform;
}

} // namespace Component
} // namespace Perceptral
