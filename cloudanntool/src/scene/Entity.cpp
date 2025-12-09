#include "scene/Entity.h"
#include "scene/Scene.h"

namespace CloudCore {

Entity::Entity(entt::entity handle, Scene* scene)
    : entityHandle_(handle), scene_(scene) {
}

} // namespace CloudCore
