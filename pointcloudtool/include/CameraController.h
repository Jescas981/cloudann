#pragma once

#include <Perceptral/core/Event.h>
#include <Perceptral/core/Input.h>

namespace PointCloudTool {

class CameraController {
public:
    CameraController(Perceptral::Camera* camera);

    bool onEvent(Perceptral::Event& e);
    bool isCapturingInput() const;
    
    // Keyboard update (call in update loop)
    void onUpdate(float deltaTime);

private:
    bool onMouseButtonPressed(Perceptral::MouseButtonPressedEvent& e);
    bool onMouseButtonReleased(Perceptral::MouseButtonReleasedEvent& e);
    bool onMouseMoved(Perceptral::MouseMovedEvent& e);
    bool onMouseScrolled(Perceptral::MouseScrolledEvent& e);

    Perceptral::Camera* camera_;
    
    // Mouse state
    bool leftMousePressed_ = false;
    bool middleMousePressed_ = false;
    bool rightMousePressed_ = false;
    bool firstMouse_ = true;
    float lastMouseX_ = 0.0f;
    float lastMouseY_ = 0.0f;
};

} // namespace PointCloudTool