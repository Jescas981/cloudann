#pragma once

#include <Perceptral/core/Log.h>
#include <Perceptral/renderer/RenderPass.h>
#include <memory>
#include <vector>
#include <algorithm>

namespace Perceptral {

class PC_API RenderPipeline {
public:
    RenderPipeline() = default;
    ~RenderPipeline() = default;

    template <typename T, typename... Args>
    void addPass(Args&&... args);

    void initialize();
    void render(Scene &scene, Camera &camera);
    void shutdown();

private:
    void sortPasses();

    std::vector<std::unique_ptr<RenderPass>> m_passes;
    bool m_initialized = false;
};

// Template method implementation must remain in header
template <typename T, typename... Args>
void RenderPipeline::addPass(Args&&... args) {
    auto pass = std::make_unique<T>(std::forward<Args>(args)...);
    m_passes.push_back(std::move(pass));
    sortPasses();
}

} // namespace Perceptral
