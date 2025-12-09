#pragma once

#include "RenderAPI.h"
#include <Eigen/Core>
#include <memory>

namespace CloudCore {

// Static interface for submitting render commands
class Renderer {
public:
    static void init() {
        s_RenderAPI->init();
    }

    static void beginFrame() {
        s_RenderAPI->beginFrame();
    }

    static void endFrame() {
        s_RenderAPI->endFrame();
    }

    static void shutdown() {
        s_RenderAPI->shutdown();
    }

    static void setViewport(uint32_t x, uint32_t y, uint32_t width, uint32_t height) {
        s_RenderAPI->setViewport(x, y, width, height);
    }

    static void setClearColor(const Eigen::Vector4f& color) {
        s_RenderAPI->setClearColor(color);
    }

    static void clear() {
        s_RenderAPI->clear();
    }

    static void drawIndexed(const std::shared_ptr<VertexArray>& vertexArray, uint32_t indexCount = 0) {
        s_RenderAPI->drawIndexed(vertexArray, indexCount);
    }

    static void drawArrays(const std::shared_ptr<VertexArray>& vertexArray, uint32_t vertexCount) {
        s_RenderAPI->drawArrays(vertexArray, vertexCount);
    }

    static void setDepthTest(bool enabled) {
        s_RenderAPI->setDepthTest(enabled);
    }

    static void setBlending(bool enabled) {
        s_RenderAPI->setBlending(enabled);
    }

    static void setWireframe(bool enabled) {
        s_RenderAPI->setWireframe(enabled);
    }

    static void setPointSize(float size) {
        s_RenderAPI->setPointSize(size);
    }

    static void setLineWidth(float size) {
        s_RenderAPI->setLineWidth(size);
    }

    static void setRenderAPI(std::unique_ptr<RenderAPI> api) {
        s_RenderAPI = std::move(api);
    }

private:
    static std::unique_ptr<RenderAPI> s_RenderAPI;
};

} // namespace CloudCore
