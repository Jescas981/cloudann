#include "scene/Systems.h"
#include "core/Log.h"
#include "rendering/BackgroundRenderer.h"
#include "scene/Components.h"
#include "core/Camera.h"
#include "renderer/Renderer.h"
#include "rendering/PointCloudRenderer.h"
#include <memory>

namespace CloudCore {

// Static renderer for point clouds (needs to be cleaned up before OpenGL context destruction)
static std::unique_ptr<PointCloudRenderer> s_pointCloudRenderer;
static std::unique_ptr<BackgroundRenderer> s_backgroundRenderer;

void ScriptSystem::update(Scene& scene, Timestep deltaTime) {
    auto& registry = scene.getRegistry();
    auto view = registry.view<ScriptComponent>();

    for (auto entity : view) {
        auto& script = view.get<ScriptComponent>(entity);
        if (script.onUpdate) {
            script.onUpdate(deltaTime);
        }
    }
}

void BackgroundRenderSystem::render(Scene& scene, Camera& camera) {
    auto& registry = scene.getRegistry();
    
    // Get the global background settings
    auto& bgSettings = registry.ctx().get<BackgroundSettings>();
    
    if (!bgSettings.enabled) {
        return;
    }
    
    if(!s_backgroundRenderer){
        s_backgroundRenderer = std::make_unique<BackgroundRenderer>();
        s_backgroundRenderer->initialize(bgSettings);
    }
    
    s_backgroundRenderer->render();
}

void PointCloudRenderSystem::render(Scene& scene, Camera& camera) {
    auto& registry = scene.getRegistry();
    auto view = registry.view<PointCloudComponent, TransformComponent, RenderableComponent>();

    for (auto entity : view) {
        auto& pcComp = view.get<PointCloudComponent>(entity);
        // auto& transform = view.get<TransformComponent>(entity);
        // auto& renderable = view.get<RenderableComponent>(entity);

        if (pcComp.visible && pcComp.pointCloud && !pcComp.pointCloud->empty()) {
            // For now, use the old PointCloudRenderer
            // TODO: Refactor to use new RenderCommand system
            if (!s_pointCloudRenderer) {
                s_pointCloudRenderer = std::make_unique<PointCloudRenderer>();
                s_pointCloudRenderer->initialize();
            }

            Renderer::setPointSize(pcComp.pointSize);
            s_pointCloudRenderer->render(*pcComp.pointCloud, pcComp, camera);
        }
    }
}

void PointCloudRenderSystem::shutdown() {
    // Clean up static renderer before OpenGL context is destroyed
    s_pointCloudRenderer.reset();
}

void TransformSystem::update(Scene& scene, Timestep deltaTime) {
    // Future: Handle hierarchical transforms here
}

} // namespace CloudCore
