#pragma once

#include <string>
#include <vector>
#include <memory>

namespace CloudCore {

class PointCloud;

class PointCloudExporter {
public:
    PointCloudExporter() = default;
    ~PointCloudExporter() = default;

    // Save point cloud with labels in PLY format
    // Labels are saved as a custom "label" property (uint8)
    static bool savePLYWithLabels(const std::string& filepath,
                                  const PointCloud& pointCloud,
                                  const std::vector<uint8_t>& labels,
                                  bool binary = true);

    // Save point cloud in XYZL text format (X Y Z Label per line)
    static bool saveXYZL(const std::string& filepath,
                        const PointCloud& pointCloud,
                        const std::vector<uint8_t>& labels);

private:
    // Helper to write PLY ASCII format with labels
    static bool savePLYASCII(const std::string& filepath,
                            const PointCloud& pointCloud,
                            const std::vector<uint8_t>& labels);

    // Helper to write PLY binary format with labels
    static bool savePLYBinary(const std::string& filepath,
                             const PointCloud& pointCloud,
                             const std::vector<uint8_t>& labels);
};

} // namespace CloudCore
