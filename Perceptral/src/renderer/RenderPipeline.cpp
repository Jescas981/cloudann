#include <Perceptral/renderer/RenderPipeline.h>

namespace Perceptral {

void RenderPipeline::initialize() {
    for (auto &pass : m_passes) {
        pass->initialize();
    }
    m_initialized = true;
}

void RenderPipeline::render(Scene &scene, Camera &camera) {
    if (!m_initialized) {
        PC_CORE_ERROR("RenderPipeline not initialized!");
        return;
    }

    for (auto &pass : m_passes) {
        pass->execute(scene, camera);
    }
}

void RenderPipeline::shutdown() {
    for (auto &pass : m_passes) {
        pass->shutdown();
    }
    m_passes.clear();
}

void RenderPipeline::sortPasses() {
    std::sort(m_passes.begin(), m_passes.end(),
              [](const auto &a, const auto &b) {
                  return a->getPriority() < b->getPriority();
              });
}

} // namespace Perceptral
