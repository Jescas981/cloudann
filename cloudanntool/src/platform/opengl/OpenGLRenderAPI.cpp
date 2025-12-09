#include "platform/opengl/OpenGLRenderAPI.h"
#include "renderer/VertexArray.h"
#include <glad/glad.h>
#include <iostream>

namespace CloudCore {

void OpenGLRenderAPI::init() {
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  std::cout << "OpenGL Renderer initialized" << std::endl;
  std::cout << "  Vendor: " << glGetString(GL_VENDOR) << std::endl;
  std::cout << "  Renderer: " << glGetString(GL_RENDERER) << std::endl;
  std::cout << "  Version: " << glGetString(GL_VERSION) << std::endl;
}

void OpenGLRenderAPI::shutdown() {
  // Cleanup if needed
}

void OpenGLRenderAPI::beginFrame() {
  // Cleanup if needed
}

void OpenGLRenderAPI::endFrame() {
  // Cleanup if needed
}

void OpenGLRenderAPI::setViewport(uint32_t x, uint32_t y, uint32_t width,
                                  uint32_t height) {
  glViewport(x, y, width, height);
}

void OpenGLRenderAPI::setClearColor(const Eigen::Vector4f &color) {
  glClearColor(color.x(), color.y(), color.z(), color.w());
}

void OpenGLRenderAPI::clear() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void OpenGLRenderAPI::drawIndexed(
    const std::shared_ptr<VertexArray> &vertexArray, uint32_t indexCount) {
  vertexArray->bind();
  uint32_t count =
      indexCount ? indexCount : vertexArray->getIndexBuffer()->getCount();
  glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, nullptr);
}

void OpenGLRenderAPI::drawArrays(
    const std::shared_ptr<VertexArray> &vertexArray, uint32_t vertexCount) {
  vertexArray->bind();
  glDrawArrays(GL_POINTS, 0, vertexCount);
}

void OpenGLRenderAPI::setDepthTest(bool enabled) {
  if (enabled)
    glEnable(GL_DEPTH_TEST);
  else
    glDisable(GL_DEPTH_TEST);
}

void OpenGLRenderAPI::setBlending(bool enabled) {
  if (enabled)
    glEnable(GL_BLEND);
  else
    glDisable(GL_BLEND);
}

void OpenGLRenderAPI::setWireframe(bool enabled) {
  glPolygonMode(GL_FRONT_AND_BACK, enabled ? GL_LINE : GL_FILL);
}

void OpenGLRenderAPI::setPointSize(float size) { glPointSize(size); }

void OpenGLRenderAPI::setLineWidth(float size) { glLineWidth(size); }

} // namespace CloudCore
