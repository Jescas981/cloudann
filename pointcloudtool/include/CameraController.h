#pragma once

#include "core/Camera.h"
#include "core/Event.h"
#include "core/Input.h"

namespace PointCloudTool {

enum class CameraMode {
    Orbit,
    FPS
};

class CameraController {
public:
    CameraController(CloudCore::Camera* camera);

    bool onEvent(CloudCore::Event& e);
    bool isCapturingInput() const;
    
    // Mode switching
    void setMode(CameraMode mode) { mode_ = mode; }
    CameraMode getMode() const { return mode_; }
    void toggleMode();

    // Keyboard update (call in update loop)
    void onUpdate(float deltaTime);

private:
    bool onMouseButtonPressed(CloudCore::MouseButtonPressedEvent& e);
    bool onMouseButtonReleased(CloudCore::MouseButtonReleasedEvent& e);
    bool onMouseMoved(CloudCore::MouseMovedEvent& e);
    bool onMouseScrolled(CloudCore::MouseScrolledEvent& e);

    CloudCore::Camera* camera_;
    
    // Mouse state
    bool leftMousePressed_ = false;
    bool middleMousePressed_ = false;
    bool rightMousePressed_ = false;
    bool firstMouse_ = true;
    float lastMouseX_ = 0.0f;
    float lastMouseY_ = 0.0f;
    
    // Camera mode
    CameraMode mode_ = CameraMode::Orbit;
};

} // namespace PointCloudTool