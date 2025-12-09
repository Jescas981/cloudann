// #include "core/Camera.h"
// #include "scene/PointCloud.h"
// #include "rendering/PointCloudRenderer.h"
// #include <glad/glad.h>
// #include <iostream>

// namespace CloudCore {

// Renderer::Renderer()
//     : clearColor_(0.1f, 0.1f, 0.1f, 1.0f)
//     , pointSize_(2.0f)
//     , depthTestEnabled_(true)
//     , blendingEnabled_(false)
//     , wireframeEnabled_(false)
// {
// }

// Renderer::~Renderer()
// {
// }

// bool Renderer::initialize()
// {
//     // Enable OpenGL features
//     glEnable(GL_DEPTH_TEST);
//     glEnable(GL_MULTISAMPLE);
//     glEnable(GL_PROGRAM_POINT_SIZE);

//     // Create point cloud renderer
//     pointCloudRenderer_ = std::make_unique<PointCloudRenderer>();
//     if (!pointCloudRenderer_->initialize()) {
//         std::cerr << "Failed to initialize point cloud renderer" << std::endl;
//         return false;
//     }

//     std::cout << "Renderer initialized successfully" << std::endl;
//     return true;
// }

// void Renderer::beginFrame()
// {
//     glClearColor(clearColor_.x(), clearColor_.y(), clearColor_.z(), clearColor_.w());
//     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
// }

// void Renderer::endFrame()
// {
//     // Nothing to do here for now
// }

// // void Renderer::renderPointCloud(const PointCloud& pointCloud, const Camera& camera)
// // {
// //     // Legacy method - create temporary component with default settings
// //     PointCloudComponent tempComponent;
// //     tempComponent.pointCloud = std::make_shared<PointCloud>(pointCloud);
// //     tempComponent.pointSize = pointSize_;
// //     pointCloudRenderer_->render(pointCloud, tempComponent, camera);
// // }

// void Renderer::setClearColor(float r, float g, float b, float a)
// {
//     clearColor_ = Eigen::Vector4f(r, g, b, a);
// }

// void Renderer::setViewport(int x, int y, int width, int height)
// {
//     glViewport(x, y, width, height);
// }

// void Renderer::setPointSize(float size)
// {
//     pointSize_ = size;
// }

// void Renderer::enableDepthTest(bool enable)
// {
//     depthTestEnabled_ = enable;
//     if (enable) {
//         glEnable(GL_DEPTH_TEST);
//     } else {
//         glDisable(GL_DEPTH_TEST);
//     }
// }

// void Renderer::enableBlending(bool enable)
// {
//     blendingEnabled_ = enable;
//     if (enable) {
//         glEnable(GL_BLEND);
//         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//     } else {
//         glDisable(GL_BLEND);
//     }
// }

// void Renderer::enableWireframe(bool enable)
// {
//     wireframeEnabled_ = enable;
//     glPolygonMode(GL_FRONT_AND_BACK, enable ? GL_LINE : GL_FILL);
// }

// } // namespace CloudCore
