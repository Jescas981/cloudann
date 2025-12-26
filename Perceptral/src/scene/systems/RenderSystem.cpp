#include "Perceptral/core/math/TransformUtils.h"
#include "Perceptral/renderer/Renderer.h"
#include "Perceptral/scene/components/Renderable.h"
#include <Perceptral/scene/components/Camera.h>
#include <Perceptral/scene/systems/RenderSystem.h>

namespace Perceptral {

void RenderSystem::onRender(entt::registry &registry) {
  auto camView = registry.view<Component::Camera, Component::MainCamera>();
  if (camView.begin() == camView.end()) {
    return;
  }

  auto cameraEntity = *camView.begin();
  auto &camera = camView.get<Component::Camera>(cameraEntity);

  renderMeshes(registry, camera);
  renderPointClouds(registry, camera);
}

void RenderSystem::renderMeshes(entt::registry &registry,
                                const Component::Camera &camera) {
  auto view = registry.view<Component::Transform, Component::MeshRenderer,
                            Component::MeshData>();

  for (auto entity : view) {
    auto &transform = view.get<Component::Transform>(entity);
    auto &renderer = view.get<Component::MeshRenderer>(entity);
    auto &meshData = view.get<Component::MeshData>(entity);

    // Skip if not visible or invalid
    if (!renderer.visible) {
      continue;
    }

    // Bind shader
    auto &material = renderer.material;
    material.shader->bind();
    // Load uniforms
    Eigen::Matrix4f modelMatrix = Math::toMatrix(transform);
    material.shader->setMat4("u_Projection", camera.projectionMatrix);
    material.shader->setMat4("u_View", camera.viewMatrix);
    material.shader->setMat4("u_Model", modelMatrix);

    // Upload Material-specific uniforms
    for (auto &[name, value] : material.floatUniforms) {
      material.shader->setFloat(name, value);
    }
    for (auto &[name, value] : material.intUniforms) {
      material.shader->setInt(name, value);
    }
    for (auto &[name, value] : material.vec3Uniforms) {
      material.shader->setFloat3(name, value);
    }
    for (auto &[name, value] : material.boolUniforms) {
      material.shader->setBool(name, value);
    }

    if (meshData.isDirty) {
      // Create VAO if it doesn't exist yet
      if (!renderer.gpu.vao) {
        renderer.gpu.vao = VertexArray::create();
      }

      // Upload to GPU
      auto vertexBuffer = VertexBuffer::create(meshData.vertices.data(),
                                               meshData.vertices.size() *
                                                   sizeof(Eigen::Vector3f));

      vertexBuffer->setLayout({{ShaderDataType::Float3, "aPosition"},
                               {ShaderDataType::Float3, "aNormal"}});

      // Attach buffer to VAO
      renderer.gpu.vao->clearVertexBuffers();
      renderer.gpu.vao->addVertexBuffer(vertexBuffer);

      auto indexBuffer =
          IndexBuffer::create(meshData.indices.data(), meshData.indices.size());
      renderer.gpu.vao->setIndexBuffer(indexBuffer);

      renderer.gpu.indexCount = meshData.indices.size();
      meshData.isDirty = false;
    }

    Renderer::drawIndexed(renderer.gpu.vao, renderer.gpu.indexCount);
  }
}

void RenderSystem::renderPointClouds(entt::registry &registry,
                                     const Component::Camera &camera) {}

} // namespace Perceptral