#include "renderer/RenderAPI.h"
#include "platform/opengl/OpenGLRenderAPI.h"
#include <iostream>
#include "core/Platform.h"

namespace CloudCore {

RenderAPI::API RenderAPI::s_API = RenderAPI::API::OpenGL;

std::unique_ptr<RenderAPI> RenderAPI::create() {
    #ifdef CC_RENDER_API_OPENGL
    return std::make_unique<OpenGLRenderAPI>();
    #else
    #error "Platform not supported"
    #endif
}

} // namespace CloudCore
