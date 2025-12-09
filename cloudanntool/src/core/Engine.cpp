#include "core/Engine.h"
#include "core/Log.h"
#include "core/Window.h"
// #include "core/Renderer.h"
#include "renderer/Renderer.h"
#include "core/EventManager.h"
#include "core/Camera.h"
#include <GLFW/glfw3.h>

namespace CloudCore {

Engine::Engine()
    : running_(false)
    , deltaTime_(0.0)
    , lastFrameTime_(0.0)
    , fps_(0.0)
    , frameCount_(0)
    , fpsUpdateTime_(0.0)
{
}

Engine::~Engine()
{
    shutdown();
}

bool Engine::initialize(int width, int height, const std::string& title)
{
    // Initialize logger first
    Log::init();
    CC_CORE_INFO("Initializing CloudCore Engine");

    // Initialize GLFW
    if (!glfwInit()) {
        CC_CORE_ERROR("Failed to initialize GLFW");
        return false;
    }
    CC_CORE_DEBUG("GLFW initialized");

    // Create window
    window_ = std::make_unique<Window>();
    if (!window_->create(width, height, title)) {
        CC_CORE_ERROR("Failed to create window");
        glfwTerminate();
        return false;
    }
    CC_CORE_INFO("Window created: {}x{}", width, height);

    // Initialize renderer
    Renderer::init();
    CC_CORE_INFO("Renderer initialized");

    // Create event manager
    eventManager_ = std::make_unique<EventManager>();
    eventManager_->initialize(window_->getHandle());
    CC_CORE_DEBUG("Event manager initialized");

    // Create camera
    // camera_ = std::make_unique<Camera>();
    // camera_->setPosition(Eigen::Vector3f(0.0f, 0.0f, 5.0f));
    // camera_->setTarget(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    // camera_->setPerspective(45.0f, window_->getAspectRatio(), 0.1f, 1000.0f);
    // CC_CORE_DEBUG("Camera initialized");

    // Set up window resize callback
    // window_->setResizeCallback([this](int w, int h) {
    //     renderer_->setViewport(0, 0, w, h);
    //     camera_->setPerspective(camera_->getFOV(),
    //                            static_cast<float>(w) / h,
    //                            camera_->getNear(),
    //                            camera_->getFar());
    //     CC_CORE_DEBUG("Window resized: {}x{}", w, h);
    // });

    running_ = true;
    lastFrameTime_ = glfwGetTime();

    CC_CORE_INFO("Engine initialized successfully");
    return true;
}

void Engine::run()
{
    while (running_ && !window_->shouldClose()) {
        updateFrameTiming();

        window_->pollEvents();
        eventManager_->processEvents();

        Renderer::beginFrame();
        
        update();
        render();
        
        Renderer::endFrame();

        window_->swapBuffers();
    }
}


void Engine::shutdown()
{
    CC_CORE_INFO("Shutting down Engine");

    eventManager_.reset();
    renderer_.reset();
    window_.reset();

    glfwTerminate();
    Log::shutdown();
}

void Engine::updateFrameTiming()
{
    double currentTime = glfwGetTime();
    deltaTime_ = currentTime - lastFrameTime_;
    lastFrameTime_ = currentTime;

    // Update FPS counter
    frameCount_++;
    if (currentTime - fpsUpdateTime_ >= 1.0) {
        fps_ = frameCount_ / (currentTime - fpsUpdateTime_);
        frameCount_ = 0;
        fpsUpdateTime_ = currentTime;
    }
}

} // namespace CloudCore
