#pragma once
#include <Eigen/Eigen>
#include <Perceptral/core/Macros.h>

namespace Perceptral {
namespace Component {

struct PC_API Camera {
  float fov = 45.0f;
  float aspectRatio = 16.0f / 9.0f;
  float nearPlane = 0.1f;
  float farPlane = 1000.0f;

  bool autoAspect{true};

  Eigen::Matrix4f projectionMatrix = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f viewMatrix = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f viewProjectionMatrix = Eigen::Matrix4f::Identity();

  bool needsProjectionUpdate = true;
  bool needsViewUpdate = true;
};

struct PC_API MainCamera {};

struct PC_API ActiveCamera {};

} // namespace Component
} // namespace Perceptral