#include "core/layers/RenderLayer.h"
#include "core/Log.h"
#include "renderer/passes/BackgroundPass.h"
#include "renderer/passes/GridPass.h"
#include "renderer/passes/PointCloudPass.h"
#include "renderer/passes/AxisLinePass.h"

namespace CloudCore {

RenderLayer::RenderLayer(Scene& scene, Camera& camera)
    : Layer("RenderLayer")
    , m_scene(scene)
    , m_camera(camera) {
}

void RenderLayer::onAttach() {
    CC_CORE_INFO("RenderLayer attached");
    m_renderPipeline.addPass<BackgroundPass>();
    m_renderPipeline.addPass<PointCloudPass>();
    // m_renderPipeline.addPass<GridPass>();
    m_renderPipeline.addPass<AxisLinePass>();
    m_renderPipeline.initialize();
}

void RenderLayer::onDetach() {
    CC_CORE_INFO("RenderLayer detached");
    m_renderPipeline.shutdown();
}

void RenderLayer::onRender() {
    // Execute render pipeline
    m_renderPipeline.render(m_scene, m_camera);
}

void RenderLayer::onImGuiRender() {
   
}

} // namespace CloudCore