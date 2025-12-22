#pragma once
#include <entt/entt.hpp>
#include <Perceptral/scene/components/Transform.h>
#include <Perceptral/scene/components/Camera.h>
#include <Perceptral/scene/components/CameraController.h>

namespace Perceptral {

class CameraControllerSystem {
public:
    void update(entt::registry& registry, float deltaTime) {
        // Find all entities that have Transform + CameraController
        auto view = registry.view<Component::Transform, Component::CameraController>();

        for (auto entity : view) {
            auto& transform = view.get<Component::Transform>(entity);
            auto& controllerComp = view.get<Component::CameraController>(entity);

            if (controllerComp.controller) {
                controllerComp.controller->onUpdate(transform, deltaTime);
            }
        }
    }
};

} // namespace Perceptral
