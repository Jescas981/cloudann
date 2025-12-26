// scene/SystemManager.cpp
#include <Perceptral/scene/SystemManager.h>

namespace Perceptral {

SystemManager::~SystemManager() {
    clear();
}

void SystemManager::update(entt::registry& registry, float deltaTime) {
    // Update all systems in order
    for (auto* system : systems_) {
        system->update(registry, deltaTime);
    }
}

void SystemManager::render(entt::registry& registry) {
    // Render all systems
    for (auto* system : systems_) {
        system->render(registry);
    }
}

void SystemManager::onEvent(entt::registry& registry, Event& e) {
    // Forward event to all systems
    for (auto* system : systems_) {
        system->onEvent(registry, e);
        if (e.handled) break;  // Stop if handled
    }
}

void SystemManager::clear() {
    for (auto* system : systems_) {
        delete system;
    }
    systems_.clear();
}

} // namespace Perceptral