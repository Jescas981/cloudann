#include <Perceptral/io/PCDLoader.h>
#include <Perceptral/scene/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Perceptral/core/Log.h>

namespace Perceptral {

std::unique_ptr<PointCloud> PCDLoader::load(const std::string& filepath)
{
    try {
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>());

        if (pcl::io::loadPCDFile<pcl::PointXYZL>(filepath, *cloud) == -1) {
            PC_ERROR("Failed to load PCD file: {}", filepath);
            return nullptr;
        }

        auto pointCloud = std::make_unique<PointCloud>(cloud);
        pointCloud->setName(filepath);
        pointCloud->computeBounds();
        pointCloud->setHasLabels(false);

        PC_INFO("Loaded PCD file: {} with {} points", filepath, cloud->size());
        return pointCloud;

    } catch (const std::exception& e) {
        PC_ERROR("Exception while loading PCD file {}: {}", filepath, e.what());
        return nullptr;
    }
}


} // namespace Perceptral
