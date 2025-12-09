#pragma once
#include "scene/Scene.h"

namespace CloudCore {
class Camera;

class RenderPass {
public:
    virtual ~RenderPass() = default;
    virtual void initialize() {}
    virtual void execute(Scene& scene, Camera& camera) = 0;
    virtual void shutdown() {}
    virtual int getPriority() const = 0;
};

} // namespace CloudCore