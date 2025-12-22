#pragma once
#include <string>

namespace Perceptral {
namespace Component {

struct Tag {
  std::string tag;

  Tag() = default;
  Tag(const std::string &tag) : tag(tag) {}
};

} // namespace Component
} // namespace Perceptral