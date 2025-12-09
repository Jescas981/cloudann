#pragma once

#include "Components.h"
#include "Scene.h"
#include "core/Timestep.h"
#include <entt/entt.hpp>

namespace CloudCore {

class Camera;

// System for updating script components
class ScriptSystem {
public:
  static void update(Scene &scene, Timestep deltaTime);
};

// System for rendering point clouds
class PointCloudRenderSystem {
public:
  static void render(Scene &scene, Camera &camera);
  static void
  shutdown(); // Clean up OpenGL resources before context destruction
};

// System for rendering background
class BackgroundRenderSystem {
public:
  static void render(Scene &scene, Camera &camera);
};

// System for transform updates (future use - hierarchical transforms)
class TransformSystem {
public:
  static void update(Scene &scene, Timestep deltaTime);
};

} // namespace CloudCore
