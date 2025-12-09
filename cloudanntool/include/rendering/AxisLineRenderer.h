#pragma once

#include "renderer/Shader.h"
#include "renderer/VertexArray.h"
#include <Eigen/Core>
#include <memory>

namespace CloudCore {

class Shader;
class Camera;

class AxisLineRenderer {
public:
  AxisLineRenderer();
  ~AxisLineRenderer();

  void initialize();

    void render(const Camera &camera);

private:
  std::shared_ptr<Shader> shader_;
  std::shared_ptr<VertexArray> vertexArray_;
};

} // namespace CloudCore