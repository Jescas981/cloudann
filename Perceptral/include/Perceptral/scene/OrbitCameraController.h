// #pragma once

// #include <Eigen/Core>
// #include <Perceptral/scene/CameraController.h>

// namespace Perceptral {
// namespace Component {

// class OrbitCameraController {
// public:
//   OrbitCameraController();

//   void onUpdate(float deltaTime);
//   void onMouseMove(float xoffset,
//                    float yoffset);
//   void onMouseScroll(float offset);

//   void setTarget(const Eigen::Vector3f &target);
//   void setDistance(float distance);

// private:
//   Eigen::Vector3f target_{Eigen::Vector3f::Zero()};

//   float distance_ = 10.0f;
//   float minDistance_ = 0.5f;
//   float maxDistance_ = 1000.0f;

//   float yaw_ = 0.0f;   // radians
//   float pitch_ = 0.0f; // radians
// };

// } // namespace Component
// } // namespace Perceptral