#include "scene/Scene.h"
#include "scene/Components.h"
#include "scene/Systems.h"
#include "rendering/BackgroundRenderer.h"



namespace CloudCore {

Scene::Scene(const std::string& name)
    : name_(name) {
    registry_.ctx().emplace<BackgroundSettings>();
}

Scene::~Scene() {
    clear();
}

void Scene::onCreate(){
}

void Scene::onUpdate(Timestep deltaTime) {
    // Update all systems
ScriptSystem::update(*this, deltaTime);
    TransformSystem::update(*this, deltaTime);
}

void Scene::onRender() {
    if (camera_) {
        BackgroundRenderSystem::render(*this,*camera_);
        PointCloudRenderSystem::render(*this, *camera_);
    }
}

Entity Scene::createEntity(const std::string& name) {
    Entity entity = { registry_.create(), this };
    entity.addComponent<TagComponent>(name);
    entity.addComponent<TransformComponent>();
    return entity;
}

void Scene::destroyEntity(Entity entity) {
    registry_.destroy(entity);
}

void Scene::clear() {
    registry_.clear();
}

Entity Scene::findEntityByName(const std::string& name) {
    auto view = registry_.view<TagComponent>();
    for (auto entity : view) {
        const TagComponent& tag = view.get<TagComponent>(entity);
        if (tag.tag == name) {
            return { entity, this };
        }
    }
    return {};
}

} // namespace CloudCore
