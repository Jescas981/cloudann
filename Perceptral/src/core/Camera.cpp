#include <Eigen/Eigen>
#include <Perceptral/core/Log.h>
#include <Perceptral/scene/components/Camera.h>
#include <cmath>

namespace Perceptral {
namespace Component {

Camera::Camera()
    : isPerspective_(true), fov_(60.0f), aspectRatio_(16.0f / 9.0f),
      nearPlane_(0.1f), farPlane_(50.0f), orthoLeft_(-1.0f), orthoRight_(1.0f),
      orthoBottom_(-1.0f), orthoTop_(1.0f) {}

Camera::~Camera() {}

void Camera::setPerspective(float fov, float aspect, float nearPlane,
                            float farPlane) {
  isPerspective_ = true;
  fov_ = fov;
  aspectRatio_ = aspect;
  nearPlane_ = nearPlane;
  farPlane_ = farPlane;
  recalcProjection();
}

void Camera::setOrthographic(float left, float right, float bottom, float top,
                             float nearPlane, float farPlane) {
  isPerspective_ = false;
  orthoLeft_ = left;
  orthoRight_ = right;
  orthoBottom_ = bottom;
  orthoTop_ = top;
  nearPlane_ = nearPlane;
  farPlane_ = farPlane;
  recalcProjection();
}

void Camera::recalcProjection() {
  projection_.setIdentity();

  if (isPerspective_) {
    float tanHalfFov = std::tan(fov_ * 0.5f);
    projection_(0, 0) = 1.0f / (aspectRatio_ * tanHalfFov);
    projection_(1, 1) = 1.0f / tanHalfFov;
    projection_(2, 2) = -(farPlane_ + nearPlane_) / (farPlane_ - nearPlane_);
    projection_(2, 3) =
        -(2.0f * farPlane_ * nearPlane_) / (farPlane_ - nearPlane_);
    projection_(3, 2) = -1.0f;
    projection_(3, 3) = 0.0f;
  } else {
    projection_(0, 0) = 2.0f / (orthoRight_ - orthoLeft_);
    projection_(1, 1) = 2.0f / (orthoTop_ - orthoBottom_);
    projection_(2, 2) = -2.0f / (farPlane_ - nearPlane_);
    projection_(0, 3) =
        -(orthoRight_ + orthoLeft_) / (orthoRight_ - orthoLeft_);
    projection_(1, 3) =
        -(orthoTop_ + orthoBottom_) / (orthoTop_ - orthoBottom_);
    projection_(2, 3) = -(farPlane_ + nearPlane_) / (farPlane_ - nearPlane_);
  }
}

Eigen::Matrix4f Camera::getProjectionMatrix() const { return projection_; }

} // namespace Component
} // namespace Perceptral