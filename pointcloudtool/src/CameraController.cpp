#include "CameraController.h"
#include "core/Log.h"
#include "core/KeyCodes.h"

namespace PointCloudTool {

CameraController::CameraController(CloudCore::Camera* camera)
    : camera_(camera)
{
}

void CameraController::toggleMode()
{
    if (mode_ == CameraMode::Orbit) {
        mode_ = CameraMode::FPS;
        CC_CORE_INFO("Switched to FPS camera mode");
    } else {
        mode_ = CameraMode::Orbit;
        CC_CORE_INFO("Switched to Orbit camera mode");
    }
}

void CameraController::onUpdate(float deltaTime)
{
    if (mode_ != CameraMode::FPS) {
        return;
    }

    // FPS keyboard movement
    using CloudCore::KeyCode;
    using CloudCore::Input;
    using CloudCore::CameraMovement;

    if (Input::isKeyPressed(KeyCode::W)) {
        camera_->processKeyboard(CameraMovement::Forward, deltaTime);
    }
    if (Input::isKeyPressed(KeyCode::S)) {
        camera_->processKeyboard(CameraMovement::Backward, deltaTime);
    }
    if (Input::isKeyPressed(KeyCode::A)) {
        camera_->processKeyboard(CameraMovement::Left, deltaTime);
    }
    if (Input::isKeyPressed(KeyCode::D)) {
        camera_->processKeyboard(CameraMovement::Right, deltaTime);
    }
    if (Input::isKeyPressed(KeyCode::E) || Input::isKeyPressed(KeyCode::Space)) {
        camera_->processKeyboard(CameraMovement::Up, deltaTime);
    }
    if (Input::isKeyPressed(KeyCode::Q) || Input::isKeyPressed(KeyCode::LeftControl)) {
        camera_->processKeyboard(CameraMovement::Down, deltaTime);
    }
}

bool CameraController::onEvent(CloudCore::Event& e)
{
    CloudCore::EventDispatcher dispatcher(e);

    dispatcher.dispatch<CloudCore::MouseButtonPressedEvent>(
        [this](CloudCore::MouseButtonPressedEvent& event) {
            return onMouseButtonPressed(event);
        });

    dispatcher.dispatch<CloudCore::MouseButtonReleasedEvent>(
        [this](CloudCore::MouseButtonReleasedEvent& event) {
            return onMouseButtonReleased(event);
        });

    dispatcher.dispatch<CloudCore::MouseMovedEvent>(
        [this](CloudCore::MouseMovedEvent& event) {
            return onMouseMoved(event);
        });

    dispatcher.dispatch<CloudCore::MouseScrolledEvent>(
        [this](CloudCore::MouseScrolledEvent& event) {
            return onMouseScrolled(event);
        });

    return false;
}

bool CameraController::isCapturingInput() const
{
    return leftMousePressed_ || middleMousePressed_ || rightMousePressed_;
}

bool CameraController::onMouseButtonPressed(CloudCore::MouseButtonPressedEvent& e)
{
    if (e.getMouseButton() == CloudCore::MouseButton::Left) {
        leftMousePressed_ = true;
        firstMouse_ = true;
        CC_CORE_TRACE("Left mouse pressed");
        return true;
    }
    if (e.getMouseButton() == CloudCore::MouseButton::Middle) {
        middleMousePressed_ = true;
        firstMouse_ = true;
        CC_CORE_TRACE("Middle mouse pressed - pan mode");
        return true;
    }
    if (e.getMouseButton() == CloudCore::MouseButton::Right) {
        rightMousePressed_ = true;
        firstMouse_ = true;
        CC_CORE_TRACE("Right mouse pressed");
        return true;
    }
    return false;
}

bool CameraController::onMouseButtonReleased(CloudCore::MouseButtonReleasedEvent& e)
{
    if (e.getMouseButton() == CloudCore::MouseButton::Left) {
        leftMousePressed_ = false;
        CC_CORE_TRACE("Left mouse released");
        return true;
    }
    if (e.getMouseButton() == CloudCore::MouseButton::Middle) {
        middleMousePressed_ = false;
        CC_CORE_TRACE("Middle mouse released");
        return true;
    }
    if (e.getMouseButton() == CloudCore::MouseButton::Right) {
        rightMousePressed_ = false;
        CC_CORE_TRACE("Right mouse released");
        return true;
    }
    return false;
}

bool CameraController::onMouseMoved(CloudCore::MouseMovedEvent& e)
{
    float xpos = e.getX();
    float ypos = e.getY();

    if (firstMouse_) {
        lastMouseX_ = xpos;
        lastMouseY_ = ypos;
        firstMouse_ = false;
        return false;
    }

    float xoffset = xpos - lastMouseX_;
    float yoffset = lastMouseY_ - ypos; // Reversed for natural rotation

    bool shiftPressed = CloudCore::Input::isKeyPressed(CloudCore::KeyCode::LeftShift) ||
                       CloudCore::Input::isKeyPressed(CloudCore::KeyCode::RightShift);
    bool ctrlPressed = CloudCore::Input::isKeyPressed(CloudCore::KeyCode::LeftControl) ||
                      CloudCore::Input::isKeyPressed(CloudCore::KeyCode::RightControl);

    if (mode_ == CameraMode::Orbit) {
        // Orbit mode controls
        // Zoom mode: right mouse or ctrl+left mouse
        if (rightMousePressed_ || (leftMousePressed_ && ctrlPressed)) {
            camera_->orbitZoom(-yoffset * 0.1f);
            CC_CORE_TRACE("Zooming: offset({:.2f})", yoffset);
        }
        // Pan mode: middle mouse or shift+left mouse
        else if (middleMousePressed_ || (leftMousePressed_ && shiftPressed)) {
            camera_->orbitPan(xoffset, yoffset);
            CC_CORE_TRACE("Panning: offset({:.2f}, {:.2f})", xoffset, yoffset);
        }
        // Rotate mode: left mouse only (no modifiers)
        else if (leftMousePressed_) {
            camera_->orbitRotate(xoffset, yoffset);
            CC_CORE_TRACE("Rotating: offset({:.2f}, {:.2f})", xoffset, yoffset);
        }
    } else {
        // FPS mode controls
        // Right mouse button enables look-around in FPS mode
        if (rightMousePressed_) {
            camera_->processMouseMovement(xoffset, yoffset);
        }
    }

    lastMouseX_ = xpos;
    lastMouseY_ = ypos;
    return leftMousePressed_ || middleMousePressed_ || rightMousePressed_;
}

bool CameraController::onMouseScrolled(CloudCore::MouseScrolledEvent& e)
{
    if (mode_ == CameraMode::Orbit) {
        camera_->orbitZoom(e.getYOffset());
        CC_CORE_TRACE("Orbit Zoom: {:.2f}, distance: {:.2f}",
                     e.getYOffset(), camera_->getOrbitDistance());
    } else {
        camera_->processMouseScroll(e.getYOffset());
        CC_CORE_TRACE("FPS Zoom (FOV): {:.2f}", e.getYOffset());
    }
    return true;
}

} // namespace PointCloudTool