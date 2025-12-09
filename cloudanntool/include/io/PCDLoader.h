#pragma once

#include <string>
#include <memory>

namespace CloudCore {

class PointCloud;

class PCDLoader {
public:
    PCDLoader();
    ~PCDLoader();

    // Load PCD file
    static std::unique_ptr<PointCloud> load(const std::string& filepath);

    // Save PCD file
    static bool save(const std::string& filepath, const PointCloud& pointCloud, bool binary = true);

private:
    static std::unique_ptr<PointCloud> loadASCII(const std::string& filepath);
    static std::unique_ptr<PointCloud> loadBinary(const std::string& filepath);
};

} // namespace CloudCore
