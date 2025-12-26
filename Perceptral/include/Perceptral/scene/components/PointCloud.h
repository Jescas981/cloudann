#pragma once
#include <Eigen/Eigen>
#include <Perceptral/resources/PointCloud.h>
#include <memory>

namespace Perceptral {
namespace Component {

enum class PointCloudColorMode {
  RGB = 0,       // Use RGB colors from point cloud data
  FlatColor = 1, // Use a single flat color
  AxisX = 2,     // Color based on X coordinate (gradient)
  AxisY = 3,     // Color based on Y coordinate (gradient)
  AxisZ = 4,     // Color based on Z coordinate (gradient)
};

// Point cloud component
struct PC_API PointCloud {
  std::shared_ptr<Resource::PointCloud> pointCloud;
  bool visible = true;
  float pointSize = 3.0f;

  // Simple color mode - applies to all points
  PointCloudColorMode colorMode = PointCloudColorMode::FlatColor;
  Eigen::Vector3f flatColor{0.8f, 0.8f, 0.8f};
};

} // namespace Component
} // namespace Perceptral