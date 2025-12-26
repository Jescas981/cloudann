#pragma once
#include <Perceptral/core/DeltaTime.h>
#include <Perceptral/core/Event.h>
#include <Perceptral/scene/components/NativeScript.h>
#include <entt/entt.hpp>

namespace Perceptral {
class Scriptable {
public:
  virtual ~Scriptable() = default;

  virtual void onCreate() {}
  virtual void onDestroy() {}
  virtual void onUpdate(DeltaTime dt) { UNUSED(dt); }
  virtual void onEvent(Event &e) { UNUSED(e); }

  void setEntity(entt::entity entity, entt::registry *registry) {
    entity_ = entity;
    registry_ = registry;
  }

protected:
  entt::entity entity_{entt::null};
  entt::registry *registry_{nullptr};
};

template <typename T> void BindNativeScript(Component::NativeScript &ns) {
  static_assert(std::is_base_of_v<Scriptable, T>,
                "T must derive from Scriptable");

  ns.instantiate = []() -> Scriptable * { return new T(); };

  ns.destroy = [](Component::NativeScript *ns) {
    delete ns->instance;
    ns->instance = nullptr;
  };
}

} // namespace Perceptral