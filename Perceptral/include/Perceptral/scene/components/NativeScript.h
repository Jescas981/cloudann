#pragma once
#include <Eigen/Eigen>
#include <Perceptral/core/Macros.h>

namespace Perceptral {
class Scriptable;

namespace Component {

struct NativeScript {
  bool enabled{true};
  Scriptable *instance = nullptr;

  Scriptable *(*instantiate)();
  void (*destroy)(NativeScript *);
};

} // namespace Component
} // namespace Perceptral