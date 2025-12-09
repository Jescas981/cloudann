#include "core/Application.h"
#include "core/Log.h"
#include "core/Input.h"
#include "core/KeyCodes.h"
#include "core/Event.h"
#include "scene/Scene.h"
#include "scene/Entity.h"
#include "scene/Components.h"
#include "core/Camera.h"
#include "io/PCDLoader.h"
#include "io/PLYLoader.h"
#include "io/ImageExporter.h"
#include <Eigen/src/Core/Matrix.h>
#include <imgui.h>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace CloudCore;

// Point Cloud Viewer Scene
class PointCloudViewerScene : public Scene {
public:
    PointCloudViewerScene() : Scene("PointCloudViewer") {}

    void onCreate() override {
        CC_CORE_INFO("Creating PointCloudViewerScene...");

        // Try to load point cloud from assets
        std::shared_ptr<PointCloud> pointCloud;
        pointCloud = PCDLoader::load("assets/stack.pcd");

        if (!pointCloud) {
            CC_CORE_WARN("PCD file not found, trying PLY...");
            pointCloud = PLYLoader::load("assets/pointcloud.ply");
        }

        if (!pointCloud || pointCloud->empty()) {
            CC_CORE_ERROR("No point cloud found in assets");
        }

        // Compute bounds and center camera with orbit controls
        pointCloud->computeBounds();
        Eigen::Vector3f center = pointCloud->getCenter();
        float radius = pointCloud->getBoundingSphereRadius();

        CC_CORE_INFO("Point cloud loaded: {} points", pointCloud->size());
        CC_CORE_DEBUG("  Bounds: [{}, {}, {}] to [{}, {}, {}]",
                     pointCloud->getMin().x(), pointCloud->getMin().y(), pointCloud->getMin().z(),
                     pointCloud->getMax().x(), pointCloud->getMax().y(), pointCloud->getMax().z());
        CC_CORE_DEBUG("  Center: [{}, {}, {}]", center.x(), center.y(), center.z());
        CC_CORE_DEBUG("  Radius: {}", radius);

        // Setup orbit camera to frame the point cloud
        if (getCamera()) {
            getCamera()->frameTarget(center, radius);
            CC_CORE_INFO("Camera framed at distance: {:.2f}", getCamera()->getOrbitDistance());
        }

        // Create entity with ECS
        auto entity = createEntity("PointCloud");
        entity.addComponent<PointCloudComponent>(pointCloud);
        entity.addComponent<RenderableComponent>();

        CC_CORE_INFO("Scene created successfully!");
    }
};

// Point Cloud Viewer Application (NO GLFW references!)
class PointCloudViewerApp : public Application {
public:
    PointCloudViewerApp() : Application("Point Cloud Viewer") {}

    void onInit() override {
        CC_CORE_INFO("=== Point Cloud Viewer ===");
        CC_CORE_INFO("Initializing viewer application...");

        auto scene = std::make_shared<PointCloudViewerScene>();
        pushScene(scene);

        CC_CORE_INFO("\nPCL-style Camera Controls:");
        CC_CORE_INFO("  Left Mouse Drag:       Rotate around target");
        CC_CORE_INFO("  Middle Mouse Drag:     Pan camera");
        CC_CORE_INFO("  Shift + Left Drag:     Pan camera");
        CC_CORE_INFO("  Right Mouse Drag:      Zoom in/out");
        CC_CORE_INFO("  Ctrl + Left Drag:      Zoom in/out");
        CC_CORE_INFO("  Mouse Wheel:           Zoom in/out");
        CC_CORE_INFO("  R:                     Reset camera view");
        CC_CORE_INFO("  V:                     Toggle verbose logging");
        CC_CORE_INFO("  ESC:                   Exit\n");

        CC_CORE_INFO("Point Cloud Viewer initialized successfully");
    }

    void onUpdate(Timestep deltaTime) override {
        // FPS and statistics logging
        static double accumulatedTime = 0.0;
        static int frameCount = 0;
        accumulatedTime += deltaTime;
        frameCount++;

        if (accumulatedTime >= 5.0) {
            auto scene = getSceneManager().getCurrentScene();
            auto& registry = scene->getRegistry();
            auto view = registry.view<PointCloudComponent>();

            size_t totalPoints = 0;
            for (auto entity : view) {
                auto& pc = view.get<PointCloudComponent>(entity);
                if (pc.pointCloud) {
                    totalPoints += pc.pointCloud->size();
                }
            }

            float avgFps = frameCount / accumulatedTime;
            CC_CORE_INFO("FPS: {:.1f} | Points: {} | Camera Distance: {:.2f}",
                        avgFps, totalPoints, getCamera()->getOrbitDistance());

            accumulatedTime = 0.0;
            frameCount = 0;
        }

        // Capture frame if recording
        if (ImageExporter::isRecording()) {
            ImageExporter::captureFrame();
        }
    }

    // Event-driven input handling with orbit controls
    void onEvent(Event& e) override {
        Application::onEvent(e);

        // If ImGui handled the event, don't process camera controls
        if (e.isHandled()) {
            return;
        }

        EventDispatcher dispatcher(e);

        // Keyboard events
        dispatcher.dispatch<KeyPressedEvent>([this](KeyPressedEvent& event) {
            if (event.getKeyCode() == KeyCode::Escape) {
                CC_CORE_INFO("Exiting application...");
                stop();
                return true;
            }
            if (event.getKeyCode() == KeyCode::R) {
                // Reset camera to frame the point cloud
                auto scene = getSceneManager().getCurrentScene();
                auto& registry = scene->getRegistry();
                auto view = registry.view<PointCloudComponent>();
                for (auto entity : view) {
                    auto& pc = view.get<PointCloudComponent>(entity);
                    if (pc.pointCloud) {
                        pc.pointCloud->computeBounds();
                        getCamera()->frameTarget(pc.pointCloud->getCenter(),
                                               pc.pointCloud->getBoundingSphereRadius());
                        CC_CORE_INFO("Camera reset to frame point cloud");
                        break;
                    }
                }
                return true;
            }
            if (event.getKeyCode() == KeyCode::V) {
                // Toggle verbose logging
                verboseLogging_ = !verboseLogging_;
                if (verboseLogging_) {
                    Log::setDebug();
                    CC_CORE_INFO("Verbose logging enabled");
                } else {
                    Log::setInfo();
                    CC_CORE_INFO("Verbose logging disabled");
                }
                return true;
            }
            return false;
        });

        // Mouse button events (PCL-style)
        dispatcher.dispatch<MouseButtonPressedEvent>([this](MouseButtonPressedEvent& event) {
            if (event.getMouseButton() == MouseButton::Left) {
                leftMousePressed_ = true;
                firstMouse_ = true;
                CC_CORE_TRACE("Left mouse pressed");
                return true;
            }
            if (event.getMouseButton() == MouseButton::Middle) {
                middleMousePressed_ = true;
                firstMouse_ = true;
                CC_CORE_TRACE("Middle mouse pressed - pan mode");
                return true;
            }
            if (event.getMouseButton() == MouseButton::Right) {
                rightMousePressed_ = true;
                firstMouse_ = true;
                CC_CORE_TRACE("Right mouse pressed - zoom mode");
                return true;
            }
            return false;
        });

        dispatcher.dispatch<MouseButtonReleasedEvent>([this](MouseButtonReleasedEvent& event) {
            if (event.getMouseButton() == MouseButton::Left) {
                leftMousePressed_ = false;
                CC_CORE_TRACE("Left mouse released");
                return true;
            }
            if (event.getMouseButton() == MouseButton::Middle) {
                middleMousePressed_ = false;
                CC_CORE_TRACE("Middle mouse released");
                return true;
            }
            if (event.getMouseButton() == MouseButton::Right) {
                rightMousePressed_ = false;
                CC_CORE_TRACE("Right mouse released");
                return true;
            }
            return false;
        });

        // Mouse movement for PCL-style orbit controls
        dispatcher.dispatch<MouseMovedEvent>([this](MouseMovedEvent& event) {
            float xpos = event.getX();
            float ypos = event.getY();

            if (firstMouse_) {
                lastMouseX_ = xpos;
                lastMouseY_ = ypos;
                firstMouse_ = false;
                return false;
            }

            float xoffset = xpos - lastMouseX_;
            float yoffset = lastMouseY_ - ypos; // Reversed for natural rotation

            bool shiftPressed = Input::isKeyPressed(KeyCode::LeftShift) ||
                               Input::isKeyPressed(KeyCode::RightShift);
            bool ctrlPressed = Input::isKeyPressed(KeyCode::LeftControl) ||
                              Input::isKeyPressed(KeyCode::RightControl);

            // Zoom mode: right mouse or ctrl+left mouse
            if (rightMousePressed_ || (leftMousePressed_ && ctrlPressed)) {
                // Use vertical movement for zoom (like PCL)
                getCamera()->orbitZoom(-yoffset * 0.1f);
                CC_CORE_TRACE("Zooming: offset({:.2f})", yoffset);
            }
            // Pan mode: middle mouse or shift+left mouse
            else if (middleMousePressed_ || (leftMousePressed_ && shiftPressed)) {
                getCamera()->orbitPan(xoffset, yoffset);
                CC_CORE_TRACE("Panning: offset({:.2f}, {:.2f})", xoffset, yoffset);
            }
            // Rotate mode: left mouse only (no modifiers)
            else if (leftMousePressed_) {
                getCamera()->orbitRotate(xoffset, yoffset);
                CC_CORE_TRACE("Rotating: offset({:.2f}, {:.2f})", xoffset, yoffset);
            }

            lastMouseX_ = xpos;
            lastMouseY_ = ypos;
            return leftMousePressed_ || middleMousePressed_ || rightMousePressed_;
        });

        // Mouse scroll for zoom
        dispatcher.dispatch<MouseScrolledEvent>([this](MouseScrolledEvent& event) {
            getCamera()->orbitZoom(event.getYOffset());
            CC_CORE_TRACE("Zoom: {:.2f}, distance: {:.2f}",
                         event.getYOffset(), getCamera()->getOrbitDistance());
            return true;
        });
    }

    // ImGui UI rendering
    void onImGuiRender() override {
        // Get point cloud component for settings
        auto scene = getSceneManager().getCurrentScene();
        if (!scene) return;

        auto& registry = scene->getRegistry();
        auto view = registry.view<PointCloudComponent>();

        PointCloudComponent* pcComp = nullptr;
        for (auto entity : view) {
            pcComp = &view.get<PointCloudComponent>(entity);
            break; // Get first point cloud
        }

        if (!pcComp) return;

        // Main menu bar
        if (ImGui::BeginMainMenuBar()) {
            // File menu
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Load Point Cloud...", "Ctrl+O")) {
                    // TODO: File browser dialog
                    CC_CORE_INFO("Load point cloud - Not implemented yet");
                }

                ImGui::Separator();

                if (ImGui::MenuItem("Exit", "ESC")) {
                    CC_CORE_INFO("Exiting application...");
                    stop();
                }

                ImGui::EndMenu();
            }

            // Settings menu
            if (ImGui::BeginMenu("Settings")) {
                ImGui::Text("Camera");
                ImGui::Separator();

                float fov = getCamera()->getFOV();
                if (ImGui::SliderFloat("FOV", &fov, 10.0f, 120.0f, "%.1f°")) {
                    getCamera()->setPerspective(fov,
                                               getCamera()->getAspectRatio(),
                                               getCamera()->getNear(),
                                               getCamera()->getFar());
                    CC_CORE_TRACE("FOV changed to: {:.1f}°", fov);
                }

                if (ImGui::MenuItem("Reset Camera", "R")) {
                    if (pcComp->pointCloud) {
                        pcComp->pointCloud->computeBounds();
                        getCamera()->frameTarget(pcComp->pointCloud->getCenter(),
                                               pcComp->pointCloud->getBoundingSphereRadius());
                        CC_CORE_INFO("Camera reset to frame point cloud");
                    }
                }

                ImGui::Spacing();
                ImGui::Text("Rendering");
                ImGui::Separator();

                ImGui::SliderFloat("Point Size", &pcComp->pointSize, 1.0f, 20.0f, "%.1f");

                ImGui::Spacing();
                ImGui::Text("Logging");
                ImGui::Separator();

                if (ImGui::MenuItem("Toggle Verbose Logging", "V", verboseLogging_)) {
                    verboseLogging_ = !verboseLogging_;
                    if (verboseLogging_) {
                        Log::setDebug();
                        CC_CORE_INFO("Verbose logging enabled");
                    } else {
                        Log::setInfo();
                        CC_CORE_INFO("Verbose logging disabled");
                    }
                }

                ImGui::EndMenu();
            }

            // Rendering menu
            if (ImGui::BeginMenu("Rendering")) {
                ImGui::Text("Capture");
                ImGui::Separator();

                if (ImGui::MenuItem("Screenshot...", "F11")) {
                    showScreenshotDialog_ = true;
                }

                ImGui::Spacing();

                bool isRecording = ImageExporter::isRecording();
                if (ImGui::MenuItem(isRecording ? "Stop Recording" : "Start Recording...", "F12")) {
                    if (isRecording) {
                        // Stop recording
                        ImageExporter::stopRecording(compileToVideo_);
                        CC_CORE_INFO("Recording stopped. {} frames captured", ImageExporter::getFrameCount());
                    } else {
                        // Show recording dialog
                        showRecordingDialog_ = true;
                    }
                }

                if (isRecording) {
                    ImGui::Text("Recording: %d frames", ImageExporter::getFrameCount());
                }

                ImGui::Spacing();
                ImGui::Text("Color Mode");
                ImGui::Separator();

                const char* colorModeNames[] = { "RGB", "Flat Color", "Axis X", "Axis Y", "Axis Z", "Gradient" };
                int currentColorMode = static_cast<int>(pcComp->colorMode);

                if (ImGui::Combo("Mode", &currentColorMode, colorModeNames, IM_ARRAYSIZE(colorModeNames))) {
                    pcComp->colorMode = static_cast<PointCloudColorMode>(currentColorMode);
                    CC_CORE_INFO("Color mode changed to: {}", colorModeNames[currentColorMode]);
                }

                // Flat color picker (only show when in FlatColor mode)
                if (pcComp->colorMode == PointCloudColorMode::FlatColor) {
                    float color[3] = { pcComp->flatColor.x(), pcComp->flatColor.y(), pcComp->flatColor.z() };
                    if (ImGui::ColorEdit3("Color", color)) {
                        pcComp->flatColor = Eigen::Vector3f(color[0], color[1], color[2]);
                    }
                }

                ImGui::Spacing();
                ImGui::Text("Selection");
                ImGui::Separator();

                // Show selection color picker
                float selColor[3] = { pcComp->selectionColor.x(), pcComp->selectionColor.y(), pcComp->selectionColor.z() };
                if (ImGui::ColorEdit3("Selection Color", selColor)) {
                    pcComp->selectionColor = Eigen::Vector3f(selColor[0], selColor[1], selColor[2]);
                }

                // Show number of selected points
                if (!pcComp->selectionMask.empty()) {
                    size_t selectedCount = 0;
                    for (auto val : pcComp->selectionMask) {
                        if (val != 0) selectedCount++;
                    }
                    ImGui::Text("Selected Points: %zu", selectedCount);
                }

                ImGui::EndMenu();
            }

            // Help menu
            if (ImGui::BeginMenu("Help")) {
                if (ImGui::MenuItem("About")) {
                    showAbout_ = true;
                }

                ImGui::Separator();

                if (ImGui::MenuItem("Show Info Panel", nullptr, showInfoPanel_)) {
                    showInfoPanel_ = !showInfoPanel_;
                }

                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }

        // Info panel (toggleable)
        if (showInfoPanel_) {
            ImGui::Begin("Info", &showInfoPanel_, ImGuiWindowFlags_AlwaysAutoResize);

            // Camera info
            ImGui::Text("Camera");
            ImGui::Separator();
            ImGui::Text("Distance: %.2f", getCamera()->getOrbitDistance());
            auto pos = getCamera()->getPosition();
            ImGui::Text("Position: [%.2f, %.2f, %.2f]", pos.x(), pos.y(), pos.z());
            ImGui::Text("FOV: %.1f°", getCamera()->getFOV());

            ImGui::Spacing();

            // Point cloud info
            ImGui::Text("Point Cloud");
            ImGui::Separator();
            if (pcComp->pointCloud) {
                ImGui::Text("Points: %zu", pcComp->pointCloud->size());
                ImGui::Text("Name: %s", pcComp->pointCloud->getName().c_str());

                auto min = pcComp->pointCloud->getMin();
                auto max = pcComp->pointCloud->getMax();
                ImGui::Text("Bounds:");
                ImGui::Text("  Min: [%.2f, %.2f, %.2f]", min.x(), min.y(), min.z());
                ImGui::Text("  Max: [%.2f, %.2f, %.2f]", max.x(), max.y(), max.z());
            }

            ImGui::End();
        }

        // About dialog
        if (showAbout_) {
            ImGui::Begin("About Point Cloud Viewer", &showAbout_, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::Text("CloudCore Point Cloud Viewer");
            ImGui::Text("Version 1.0");
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::Text("PCL-style Camera Controls:");
            ImGui::BulletText("Left Mouse Drag: Rotate");
            ImGui::BulletText("Middle Mouse / Shift+Left: Pan");
            ImGui::BulletText("Right Mouse / Ctrl+Left: Zoom");
            ImGui::BulletText("Mouse Wheel: Zoom");
            ImGui::BulletText("R: Reset camera");
            ImGui::BulletText("ESC: Exit");
            ImGui::Spacing();
            if (ImGui::Button("Close")) {
                showAbout_ = false;
            }
            ImGui::End();
        }

        // Screenshot dialog
        if (showScreenshotDialog_) {
            ImGui::Begin("Save Screenshot", &showScreenshotDialog_, ImGuiWindowFlags_AlwaysAutoResize);

            ImGui::Text("Save screenshot of the current view");
            ImGui::Spacing();

            ImGui::InputText("Filename", screenshotFilename_, sizeof(screenshotFilename_));

            ImGui::Spacing();
            ImGui::Text("Format:");
            ImGui::RadioButton("PNG", &screenshotFormat_, 0); ImGui::SameLine();
            ImGui::RadioButton("JPG", &screenshotFormat_, 1); ImGui::SameLine();
            ImGui::RadioButton("BMP", &screenshotFormat_, 2);

            ImGui::Spacing();

            if (ImGui::Button("Save", ImVec2(120, 0))) {
                // Generate filename with timestamp if empty
                std::string filename(screenshotFilename_);
                if (filename.empty()) {
                    auto now = std::chrono::system_clock::now();
                    auto time = std::chrono::system_clock::to_time_t(now);
                    std::stringstream ss;
                    ss << "screenshot_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
                    filename = ss.str();
                }

                // Add extension
                const char* extensions[] = { ".png", ".jpg", ".bmp" };
                filename += extensions[screenshotFormat_];

                // Save screenshot
                int width = getWindow()->getWidth();
                int height = getWindow()->getHeight();
                if (ImageExporter::saveScreenshot(filename, width, height)) {
                    CC_CORE_INFO("Screenshot saved: {}", filename);
                    showScreenshotDialog_ = false;
                } else {
                    CC_CORE_ERROR("Failed to save screenshot");
                }
            }

            ImGui::SameLine();

            if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                showScreenshotDialog_ = false;
            }

            ImGui::End();
        }

        // Recording dialog
        if (showRecordingDialog_) {
            ImGui::Begin("Start Recording", &showRecordingDialog_, ImGuiWindowFlags_AlwaysAutoResize);

            ImGui::Text("Record frames to create a video");
            ImGui::Spacing();

            ImGui::InputText("Output Directory", recordingPath_, sizeof(recordingPath_));
            ImGui::SliderInt("FPS", &recordingFPS_, 15, 60);
            ImGui::Checkbox("Compile to video with ffmpeg", &compileToVideo_);

            ImGui::Spacing();

            if (ImGui::Button("Start Recording", ImVec2(150, 0))) {
                // Generate path with timestamp if empty
                std::string path(recordingPath_);
                if (path.empty()) {
                    auto now = std::chrono::system_clock::now();
                    auto time = std::chrono::system_clock::to_time_t(now);
                    std::stringstream ss;
                    ss << "recording_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
                    path = ss.str();
                }

                // Start recording
                int width = getWindow()->getWidth();
                int height = getWindow()->getHeight();
                ImageExporter::startRecording(path, width, height, recordingFPS_);
                CC_CORE_INFO("Recording started to: {}", path);
                showRecordingDialog_ = false;
            }

            ImGui::SameLine();

            if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                showRecordingDialog_ = false;
            }

            ImGui::End();
        }
    }

private:
    bool leftMousePressed_ = false;
    bool middleMousePressed_ = false;
    bool rightMousePressed_ = false;
    bool verboseLogging_ = false;
    bool showAbout_ = false;
    float lastMouseX_ = 0.0f;
    float lastMouseY_ = 0.0f;
    bool firstMouse_ = true;

    // Screenshot and recording UI state
    bool showScreenshotDialog_ = false;
    bool showRecordingDialog_ = false;
    bool showInfoPanel_ = true;
    char screenshotFilename_[256] = "";
    int screenshotFormat_ = 0; // 0=PNG, 1=JPG, 2=BMP
    char recordingPath_[256] = "";
    int recordingFPS_ = 30;
    bool compileToVideo_ = false;
};

int main(int argc, char** argv) {
    PointCloudViewerApp app;

    if (app.initialize(1280, 720)) {
        app.run();
    }

    return 0;
}
