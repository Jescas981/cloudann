#include "Perceptral/renderer/Shader.h"
#include "Perceptral/renderer/VertexArray.h"
#include <Perceptral/core/DeltaTime.h>
#include <entt/entt.hpp>

namespace Perceptral {
namespace Component {

struct PC_API Material {
  std::shared_ptr<Shader> shader;
  std::unordered_map<std::string, float> floatUniforms;
  std::unordered_map<std::string, int> intUniforms;
  std::unordered_map<std::string, Eigen::Vector3f> vec3Uniforms;
  std::unordered_map<std::string, bool> boolUniforms;
};

struct PC_API MeshData {
  std::vector<Eigen::Vector3f> vertices;
  std::vector<Eigen::Vector3f> normals;
  std::vector<unsigned int> indices;
  bool isDirty{true};
};

struct PC_API MeshRenderer {
  Material material;
  bool visible{true};

  struct GPU {
    std::shared_ptr<VertexArray> vao{nullptr};
    std::size_t indexCount = 0;
  } gpu;
};

} // namespace Component
} // namespace Perceptral