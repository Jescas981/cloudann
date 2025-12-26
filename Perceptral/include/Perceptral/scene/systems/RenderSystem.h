#pragma once
#include <Perceptral/core/DeltaTime.h>
#include <Perceptral/scene/components/Camera.h>
#include <Perceptral/scene/components/OrbitCameraController.h>
#include <Perceptral/scene/components/Transform.h>
#include <Perceptral/scene/systems/System.h>
#include <entt/entt.hpp>

namespace Perceptral {

class RenderSystem : public System {
public:
  void onCreate(entt::registry &registry) override;
  void onUpdate(entt::registry &registry, DeltaTime deltaTime) override;
  void onRender(entt::registry &registry) override;
  void onEvent(entt::registry &registry, Event &event) override;

private:
  void renderMeshes(entt::registry &registry, const Component::Camera &camera);
  void renderPointClouds(entt::registry &registry,
                         const Component::Camera &camera);
};

} // namespace Perceptral
