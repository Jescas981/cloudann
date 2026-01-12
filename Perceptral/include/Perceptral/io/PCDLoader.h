#pragma once

#include <string>
#include <memory>
#include <Perceptral/core/Macros.h>

namespace Perceptral {

class PointCloud;

class PC_API PCDLoader {
public:
    PCDLoader();
    ~PCDLoader();

    // Load PLY file
    static std::unique_ptr<PointCloud> load(const std::string& filepath);
};

} // namespace Perceptral
