#pragma once
#include <Perceptral/core/DeltaTime.h>
#include <Eigen/Core>

namespace Perceptral {
namespace Component {


struct OrbitCameraController {
    Eigen::Vector3f target{Eigen::Vector3f::Zero()};

    float distance = 10.0f;
    float zoomSpeed = 10.0f;
    float rotationSpeed = 0.05f;
    float yaw = 0.0f;   
    float pitch = 0.0f;
    float minDistance = 0.5f;
    float maxDistance = 1000.0f;
    float minPitch = -89.0f;
    float maxPitch = 89.0f;   

    bool firstMovement = false;
    bool isRotating = false;
    float lastMouseX = 0.0f;
    float lastMouseY = 0.0f;
};

} // namespace Component
} // namespace Perceptral