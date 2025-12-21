#pragma once

#include <Perceptral/core/Timestep.h"
#include <string>
#include <memory>

namespace Perceptral {

class PC_API Scene;

// Base class PC_API for all scene objects with lifecycle
class PC_API SceneObject {
public:
    SceneObject(const std::string& name = "SceneObject")
        : name_(name), active_(true) {}

    virtual ~SceneObject() = default;

    // Lifecycle methods (to be overridden by derived classes)
    virtual void onCreate() {}
    virtual void onUpdate(Timestep deltaTime) {}
    virtual void onRender() {}
    virtual void onDestroy() {}

    // Getters/setters
    const std::string& getName() const { return name_; }
    void setName(const std::string& name) { name_ = name; }

    bool isActive() const { return active_; }
    void setActive(bool active) { active_ = active; }

    Scene* getScene() const { return scene_; }

protected:
    friend class PC_API Scene;
    void setScene(Scene* scene) { scene_ = scene; }

    std::string name_;
    bool active_;
    Scene* scene_ = nullptr;
};

} // namespace Perceptral
