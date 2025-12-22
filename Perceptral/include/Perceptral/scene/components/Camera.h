#pragma once

#include <Eigen/Eigen>
#include <Perceptral/core/Macros.h>

namespace Perceptral {
namespace Component {
class PC_API Camera {
public:
  Camera();
  ~Camera();

  void setPerspective(float fov, float aspect, float near, float far);
  void setOrthographic(float left, float right, float bottom, float top,
                       float near, float far);

  Eigen::Matrix4f getProjectionMatrix() const;

  float getFOV() const { return fov_; }
  float getAspectRatio() const { return aspectRatio_; }
  float getNear() const { return nearPlane_; }
  float getFar() const { return farPlane_; }

private:
  void recalcProjection();

private:
  // Projection parameters
  bool isPerspective_;
  float fov_;
  float aspectRatio_;
  float nearPlane_;
  float farPlane_;

  // Orthographic projection parameters
  float orthoLeft_;
  float orthoRight_;
  float orthoBottom_;
  float orthoTop_;

  Eigen::Matrix4f projection_ = Eigen::Matrix4f::Identity();
};

} // namespace Component
} // namespace Perceptral
