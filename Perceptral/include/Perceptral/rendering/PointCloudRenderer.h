#pragma once

#include <Eigen/Core>
#include <Perceptral/renderer/Shader.h>
#include <Perceptral/renderer/VertexArray.h>
#include <memory>

namespace Perceptral {

namespace Component {
class PointCloud;
};

namespace Resource {
class PointCloud;
};

class Camera;
class Shader;
class VertexBuffer;

class PC_API PointCloudRenderer {
public:
  PointCloudRenderer();
  ~PointCloudRenderer();

  // Initialize renderer
  bool initialize();

  // Render point cloud with component settings
  void render(const Resource::PointCloud &pointCloud,
              const Component::PointCloud &component, const Camera &camera);

  // Settings
  void setPointSize(float size) { pointSize_ = size; }
  float getPointSize() const { return pointSize_; }

private:
  void setupShaders();
  void updateBuffers(const Resource::PointCloud &pointCloud,
                     const Component::PointCloud &component);

  std::shared_ptr<Shader> shader_;
  std::shared_ptr<VertexArray> vertexArray_;

  float pointSize_;
  size_t lastPointCount_;
  size_t lastSelectionHash_;
};

} // namespace Perceptral
