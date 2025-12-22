#pragma once

#include <string>
#include <memory>
#include <Perceptral/core/Macros.h>

namespace Perceptral {

namespace Resource {
class PointCloud;
};


class PC_API PLYLoader {
public:
    PLYLoader();
    ~PLYLoader();

    // Load PLY file
    static std::unique_ptr<Resource::PointCloud> load(const std::string& filepath);

    // Save PLY file
    static bool save(const std::string& filepath, const Resource::PointCloud& pointcloud, bool binary = true);

private:
    static std::unique_ptr<Resource::PointCloud> loadASCII(const std::string& filepath, size_t vertexCount);
    static std::unique_ptr<Resource::PointCloud> loadBinary(const std::string& filepath, size_t vertexCount, bool bigEndian);
};

} // namespace Perceptral
