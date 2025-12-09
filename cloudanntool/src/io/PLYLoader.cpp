#include "io/PLYLoader.h"
#include "scene/PointCloud.h"
#include <pcl/io/ply_io.h>
#include <iostream>

namespace CloudCore {

PLYLoader::PLYLoader()
{
}

PLYLoader::~PLYLoader()
{
}

std::unique_ptr<PointCloud> PLYLoader::load(const std::string& filepath)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPLYFile<pcl::PointXYZ>(filepath, *cloud) == -1) {
        std::cerr << "Failed to load PLY file: " << filepath << std::endl;
        return nullptr;
    }

    std::cout << "Loaded PLY file: " << cloud->size() << " points" << std::endl;

    auto pointCloud = std::make_unique<PointCloud>(cloud);
    pointCloud->setName(filepath);
    pointCloud->computeBounds();

    return pointCloud;
}

bool PLYLoader::save(const std::string& filepath, const PointCloud& pointCloud, bool binary)
{
    auto cloud = pointCloud.getCloud();

    if (pcl::io::savePLYFile(filepath, *cloud, binary) == -1) {
        std::cerr << "Failed to save PLY file: " << filepath << std::endl;
        return false;
    }

    std::cout << "Saved PLY file: " << filepath << " with " << cloud->size() << " points" << std::endl;
    return true;
}

std::unique_ptr<PointCloud> PLYLoader::loadASCII(const std::string& filepath, size_t vertexCount)
{
    // This is handled by PCL's loadPLYFile
    return load(filepath);
}

std::unique_ptr<PointCloud> PLYLoader::loadBinary(const std::string& filepath, size_t vertexCount, bool bigEndian)
{
    // This is handled by PCL's loadPLYFile
    return load(filepath);
}

} // namespace CloudCore
