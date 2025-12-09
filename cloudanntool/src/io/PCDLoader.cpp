#include "io/PCDLoader.h"
#include "core/Log.h"
#include "scene/PointCloud.h"
#include <pcl/io/pcd_io.h>

namespace CloudCore {

PCDLoader::PCDLoader()
{
}

PCDLoader::~PCDLoader()
{
}

std::unique_ptr<PointCloud> PCDLoader::load(const std::string& filepath)
{
    CC_CORE_INFO("Loading PCD file: {}", filepath);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud) == -1) {
        CC_CORE_ERROR("Failed to load PCD file: {}", filepath);
        return nullptr;
    }

    CC_CORE_INFO("Loaded PCD file successfully: {} points", cloud->size());

    auto pointCloud = std::make_unique<PointCloud>(cloud);
    pointCloud->setName(filepath);
    pointCloud->computeBounds();

    CC_CORE_DEBUG("Point cloud bounds computed");
    return pointCloud;
}

bool PCDLoader::save(const std::string& filepath, const PointCloud& pointCloud, bool binary)
{
    CC_CORE_INFO("Saving PCD file: {} ({})", filepath, binary ? "binary" : "ASCII");
    auto cloud = pointCloud.getCloud();

    if (pcl::io::savePCDFile(filepath, *cloud, binary) == -1) {
        CC_CORE_ERROR("Failed to save PCD file: {}", filepath);
        return false;
    }

    CC_CORE_INFO("Saved PCD file successfully: {} points", cloud->size());
    return true;
}

std::unique_ptr<PointCloud> PCDLoader::loadASCII(const std::string& filepath)
{
    // This is handled by PCL's loadPCDFile
    return load(filepath);
}

std::unique_ptr<PointCloud> PCDLoader::loadBinary(const std::string& filepath)
{
    // This is handled by PCL's loadPCDFile
    return load(filepath);
}

} // namespace CloudCore
