#pragma once

#include <memory>
#include <string>
#include "renderer/RenderPipeline.h"

namespace CloudCore {

class Window;
class Renderer;
class EventManager;
class Camera;

class Engine {
public:
    Engine();
    ~Engine();

    // Initialize the engine with window parameters
    bool initialize(int width, int height, const std::string& title);

    // Main loop
    void run();

    // Shutdown and cleanup
    void shutdown();

    // Getters
    Window* getWindow() const { return window_.get(); }
    Renderer* getRenderer() const { return renderer_.get(); }
    EventManager* getEventManager() const { return eventManager_.get(); }
    // Control
    bool isRunning() const { return running_; }
    void stop() { running_ = false; }

    // Frame timing
    double getDeltaTime() const { return deltaTime_; }
    double getFPS() const { return fps_; }

private:
    void updateFrameTiming();
    void update();
    void render();
    
    std::unique_ptr<Window> window_;
    std::unique_ptr<Renderer> renderer_;
    std::unique_ptr<EventManager> eventManager_;

    bool running_;
    double deltaTime_;
    double lastFrameTime_;
    double fps_;
    int frameCount_;
    double fpsUpdateTime_;
};

} // namespace CloudCore
