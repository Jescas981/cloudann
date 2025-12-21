#pragma once

#include <Perceptral/core/Application.h>

extern Perceptral::Application* Perceptral::createApplication();

int main(int argc, char** argv) {
    Perceptral::Log::init();
    
    auto app = Perceptral::createApplication();
    
    if (!app->initialize(1280, 720)) {
        PC_ERROR("Failed to initialize application");
        return -1;
    }
    
    app->run();
    delete app;
    
    return 0;
}