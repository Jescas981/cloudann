#pragma once
#include <Perceptral/scene/Scene.h>

namespace Perceptral {
class PC_API Camera;

class PC_API RenderPass {
public:
    virtual ~RenderPass() = default;
    virtual void initialize() {}
    virtual void execute(Scene& scene, Camera& camera) = 0;
    virtual void shutdown() {}
    virtual int getPriority() const = 0;
};

} // namespace Perceptral