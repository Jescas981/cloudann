#pragma once

#include <memory>
#include <Eigen/Core>
#include <Perceptral/renderer/VertexArray.h>
#include <Perceptral/scene/Components.h>
#include <Perceptral/renderer/VertexArray.h>
#include <Perceptral/renderer/Shader.h>

namespace Perceptral {

class PC_API PointCloud;
class PC_API Camera;
class PC_API Shader;
class PC_API VertexBuffer;
struct PointCloudComponent;

class PC_API PointCloudRenderer {
public:
    PointCloudRenderer();
    ~PointCloudRenderer();

    // Initialize renderer
    bool initialize();

    // Render point cloud with component settings
    void render(const PointCloud& pointCloud, const PointCloudComponent& component, const Camera& camera);

    // Settings
    void setPointSize(float size) { pointSize_ = size; }
    float getPointSize() const { return pointSize_; }

private:
    void setupShaders();
    void updateBuffers(const PointCloud& pointCloud, const PointCloudComponent& component);

    std::shared_ptr<Shader> shader_;
    std::shared_ptr<VertexArray> vertexArray_;

    float pointSize_;
    size_t lastPointCount_;
    size_t pointCount_;
    size_t lastSelectionHash_;
};

} // namespace Perceptral
