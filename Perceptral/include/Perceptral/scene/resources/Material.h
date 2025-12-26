#pragma once

#include "Perceptral/renderer/Shader.h"
#include <Perceptral/core/Macros.h>
#include <Perceptral/scene/resources/Resource.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace Perceptral {
namespace Resource {

class PC_API Material : public Resource {
  std::shared_ptr<Shader> shader;
  std::map<std::string, float> floatUniforms;
  std::map<std::string, Eigen::Vector3f> vec3Uniforms;
};

} // namespace Resource
} // namespace Perceptral