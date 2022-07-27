#ifndef ROSCORE_PC2_CONVERSION_H
#define ROSCORE_PC2_CONVERSION_H

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <SlamCore/pointcloud.h>

namespace slam {

    // Converts PCL to Slam Data types
    slam::PROPERTY_TYPE ROSPointFieldDTypeToSlamDType(int pcl_type);

    // Builds a default Item Schema from a PointCloud2 fields layout
    // Note:    The layout in memory might contain padding (unused bytes) which are added as padding_<i> char properties
    slam::ItemSchema::Builder SchemaBuilderFromCloud2(const sensor_msgs::PointCloud2 &cloud);

    // A PointCloud2 Pointer Wrapper
    // It allows a BufferWrapper to keep a shared pointer to the PointCloud2, ensuring that no data is deallocated prior
    // The destruction of the BufferWrapper
    struct PointCloud2PtrWrapper : slam::BufferWrapper::SmartDataPtrWrapper {

        ~PointCloud2PtrWrapper() override = default;

        explicit PointCloud2PtrWrapper(sensor_msgs::PointCloud2Ptr _ptr) : ptr(_ptr) {}

        explicit PointCloud2PtrWrapper(sensor_msgs::PointCloud2ConstPtr _ptr) : const_ptr(_ptr) {}

        sensor_msgs::PointCloud2Ptr ptr = nullptr;
        sensor_msgs::PointCloud2ConstPtr const_ptr = nullptr;
    };

    // Returns a shallow copy of a point cloud
    // Note:    Once the data of `cloud` is deleted, the copy will
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(const sensor_msgs::PointCloud2 &cloud,
                                                         std::shared_ptr<PointCloud2PtrWrapper> pointer = nullptr);

    // Returns a shallow copy of a point cloud
    // Note:    It will add a reference to the cloud pointer, in order to keep it alive
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(sensor_msgs::PointCloud2Ptr &cloud);

    // Returns a shallow copy of a point cloud
    // Note:    It will add a reference to the cloud pointer, in order to keep it alive
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(const sensor_msgs::PointCloud2ConstPtr &cloud);;


    // Returns a deep copy of a point cloud
    // Note:    The data layout will be the same as the input cloud
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudDeep(const sensor_msgs::PointCloud2 &cloud);

} // namespace slam

#endif //ROSCORE_PC2_CONVERSION_H
