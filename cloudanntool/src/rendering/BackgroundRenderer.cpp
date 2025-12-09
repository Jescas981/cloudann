#include "rendering/BackgroundRenderer.h"
#include "core/Camera.h"
#include "core/Log.h"
#include "renderer/Buffer.h"
#include "renderer/VertexArray.h"
#include <glad/glad.h>
#include <rendering/BackgroundRenderer.h>

namespace CloudCore {

BackgroundRenderer::BackgroundRenderer()
    : shader_(nullptr), vertexArray_(nullptr), settings_() {}

BackgroundRenderer::~BackgroundRenderer() {
  // Cleanup if needed
}

void BackgroundRenderer::initialize(BackgroundSettings settings) {
  settings_ = settings;
  shader_ = Shader::create("shaders/background.glsl");
  vertexArray_ = VertexArray::create();
  float vertices[12] = {
      0.0f, 0.0f, 0.0f, // bottom-left
      1.0f, 0.0f, 0.0f, // bottom-right
      0.0f, 1.0f, 0.0f, // top-left
      1.0f, 1.0f, 0.0f  // top-right
  };

  auto vertexBuffer = VertexBuffer::create(vertices, sizeof(vertices));
  BufferLayout layout{{ShaderDataType::Float3, "aPosition"}};
  vertexBuffer->setLayout(layout);
  vertexArray_->addVertexBuffer(vertexBuffer);
  vertexArray_->unbind();
}

void BackgroundRenderer::render() {
  if (!settings_.enabled) {
    return;
  }

  // Save current OpenGL state
  GLboolean depthMaskEnabled;
  glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMaskEnabled);
  GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
  GLboolean blendEnabled = glIsEnabled(GL_BLEND);

  // Disable depth and blend for background
  glDepthMask(GL_FALSE);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);

  // Use shader
  shader_->bind();

  // Set uniforms based on mode
  shader_->setInt("uMode", static_cast<int>(settings_.mode));

  switch (settings_.mode) {
  case BackgroundSettings::Mode::SOLID_COLOR:
    shader_->setFloat4("uSolidColor", settings_.clearColor);
    break;

  case BackgroundSettings::Mode::GRADIENT:
    shader_->setFloat4("uGradientTop", settings_.gradientTop);
    shader_->setFloat4("uGradientBottom", settings_.gradientBottom);
    break;
  }

  vertexArray_->bind();
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  vertexArray_->unbind();

  // Restore OpenGL state
  if (depthTestEnabled) {
    glEnable(GL_DEPTH_TEST);
  }
  if (depthMaskEnabled) {
    glDepthMask(GL_TRUE);
  }
  if (blendEnabled) {
    glEnable(GL_BLEND);
  }
}

} // namespace CloudCore