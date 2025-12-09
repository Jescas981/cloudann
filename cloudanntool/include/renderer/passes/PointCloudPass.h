// renderer/passes/PointCloudPass.h
#include "renderer/RenderPass.h"
#include "rendering/PointCloudRenderer.h"
#include "renderer/Renderer.h"

namespace CloudCore {

class PointCloudPass : public RenderPass {
    std::unique_ptr<PointCloudRenderer> m_renderer;

public:
    void initialize() override {
        m_renderer = std::make_unique<PointCloudRenderer>();
        m_renderer->initialize();
    }

    void execute(Scene& scene, Camera& camera) override {
        auto& registry = scene.getRegistry();
        auto view = registry.view<PointCloudComponent, TransformComponent, RenderableComponent>();

        for (auto entity : view) {
            auto& pcComp = view.get<PointCloudComponent>(entity);
            
            if (pcComp.visible && pcComp.pointCloud && !pcComp.pointCloud->empty()) {
                Renderer::setPointSize(pcComp.pointSize);
                m_renderer->render(*pcComp.pointCloud, pcComp, camera);
            }
        }
    }

    void shutdown() override {
        m_renderer.reset();
    }

    int getPriority() const override { return 100; } // Render after background
};

} // namespace CloudCore