// renderer/passes/BackgroundPass.h
#include "renderer/RenderPass.h"
#include "rendering/AxisLineRenderer.h"

namespace CloudCore {

class Scene;

class AxisLinePass : public RenderPass {
    std::unique_ptr<AxisLineRenderer> m_renderer;

public:
    void initialize() override {
        m_renderer = std::make_unique<AxisLineRenderer>();
        m_renderer->initialize();
    }

    void execute(Scene& scene, Camera& camera) override {
        // auto& registry = scene.getRegistry();
        
        // Check if background settings exist
        // if (!registry.ctx().contains<BackgroundSettings>()) {
        //     return;
        // }
        
        // auto& settings = registry.ctx().get<BackgroundSettings>();
        // if (!settings.enabled) {
        //     return;
        // }

        if (!m_renderer) {
            m_renderer = std::make_unique<AxisLineRenderer>();
            m_renderer->initialize();
        }

        m_renderer->render(camera);
    }

    void shutdown() override {
        m_renderer.reset();
    }

    int getPriority() const override { return 50; } // Render first
};

} // namespace CloudCore