#pragma once
#include "Perceptral/resources/LabelDefinition.h"
#include <Perceptral/resources/PointCloud.h>
#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <set>

namespace Perceptral {
namespace Component {
enum class PointCloudColorMode {
  RGB = 0,       // Use RGB colors from point cloud data
  FlatColor = 1, // Use a single flat color
  AxisX = 2,     // Color based on X coordinate (gradient)
  AxisY = 3,     // Color based on Y coordinate (gradient)
  AxisZ = 4,     // Color based on Z coordinate (gradient)
  Gradient = 5   // Full gradient based on all axes
};

// Point cloud component
struct PointCloud {
  std::shared_ptr<Resource::PointCloud> pointCloud;
  bool visible = true;
  float pointSize = 3.0f;

  // Simple color mode - applies to all points
  PointCloudColorMode colorMode = PointCloudColorMode::FlatColor;
  Eigen::Vector3f flatColor{0.8f, 0.8f,
                            0.8f}; // Used when colorMode = FlatColor

  // Labeling system - each point has a label ID
  std::vector<uint8_t> labels; // Per-point label IDs (0-255)
  std::shared_ptr<Resource::LabelDefinition>
      labelDefinition;     // Label definitions (shared across point clouds)
  bool showLabels = false; // Whether to color points by label

  // Selection system with per-point mask (for editing/highlighting)
  std::vector<uint8_t> selectionMask; // 0 = not selected, non-zero = selected
  Eigen::Vector3f selectionColor{
      1.0f, 0.5f, 0.0f}; // Orange - overrides everything for selected points

  // Visibility system - per-point visibility mask
  std::vector<uint8_t>
      visibilityMask; // 0 = hidden, non-zero = visible (default: all visible)
  std::set<uint8_t> hiddenLabels; // Set of label IDs that are currently hidden

  PointCloud() = default;
  PointCloud(std::shared_ptr<Resource::PointCloud> pc) : pointCloud(pc) {}
};

} // namespace Component
} // namespace Perceptral