// ============================================
// layers/RenderLayer.h
// ============================================
#pragma once
#include "core/Camera.h"
#include "core/Layer.h"
#include "renderer/RenderPipeline.h"
#include "scene/Scene.h"

namespace CloudCore {

/**
 * @brief Layer responsible for rendering the scene
 * 
 * Owns and manages the RenderPipeline, executing all render passes
 * to visualize the scene with the given camera.
 */
class RenderLayer : public Layer {
public:
    RenderLayer(Scene& scene, Camera& camera);
    virtual ~RenderLayer() = default;

    // Layer lifecycle
    void onAttach() override;
    void onDetach() override;
    void onRender() override;
    
    void onImGuiRender() override;

    // Access to render pipeline for configuration
    RenderPipeline& getRenderPipeline() { return m_renderPipeline; }

private:
    Scene& m_scene;
    Camera& m_camera;
    RenderPipeline m_renderPipeline;
};

} // namespace CloudCore