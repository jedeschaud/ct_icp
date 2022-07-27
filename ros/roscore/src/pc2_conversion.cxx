#include "ROSCore/pc2_conversion.h"

namespace slam {

    /* -------------------------------------------------------------------------------------------------------------- */
    PROPERTY_TYPE ROSPointFieldDTypeToSlamDType(int pcl_type) {
        switch (pcl_type) {
            case sensor_msgs::PointField::INT8:
                return slam::INT8;
            case sensor_msgs::PointField::UINT8:
                return slam::UINT8;
            case sensor_msgs::PointField::INT16:
                return slam::INT16;
            case sensor_msgs::PointField::UINT16:
                return slam::UINT16;
            case sensor_msgs::PointField::INT32:
                return slam::INT32;
            case sensor_msgs::PointField::UINT32:
                return slam::UINT32;
            case sensor_msgs::PointField::FLOAT32:
                return slam::FLOAT32;
            case sensor_msgs::PointField::FLOAT64:
                return slam::FLOAT64;
            default:
                throw std::runtime_error("Incompatible type");
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::ItemSchema::Builder SchemaBuilderFromCloud2(const sensor_msgs::PointCloud2 &cloud) {
        slam::ItemSchema::Builder builder(cloud.point_step);
        builder.AddElement("properties", 0);

        std::vector<sensor_msgs::PointField> fields(cloud.fields);
        std::sort(fields.begin(), fields.end(),
                  [](const sensor_msgs::PointField &lhs, const sensor_msgs::PointField &rhs) {
                      return lhs.offset < rhs.offset;
                  });

        int expected_offset = 0;
        int padding_idx = 0;

        bool x_property = false;
        int offset_of_x = 0;
        int dtype_x = -1;

        for (auto &field: fields) {
            if (field.name == "x") {
                x_property = true;
                offset_of_x = field.offset;
                dtype_x = field.datatype;
            }
            auto data_size = slam::PropertySize(slam::ROSPointFieldDTypeToSlamDType(field.datatype));
            if (field.offset != expected_offset) {
                builder.AddScalarProperty<char>("properties", "padding_" + std::to_string(padding_idx++),
                                                expected_offset, field.offset - expected_offset);
            }

            builder.AddProperty("properties", std::string(field.name),
                                slam::ROSPointFieldDTypeToSlamDType(field.datatype),
                                field.offset, 1);
            expected_offset = field.offset + data_size;
        }
        if (expected_offset != cloud.point_step)
            builder.AddScalarProperty<char>("properties", "padding_" + std::to_string(padding_idx++),
                                            expected_offset, cloud.point_step - expected_offset);

        if (x_property) {
            auto dtype = slam::ROSPointFieldDTypeToSlamDType(dtype_x);
            auto dtype_size = slam::PropertySize(dtype);
            builder.AddElement("vertex", offset_of_x)
                    .AddProperty("vertex", "x", dtype, 0, 1)
                    .AddProperty("vertex", "y", dtype, dtype_size, 1)
                    .AddProperty("vertex", "z", dtype, 2 * dtype_size, 1);
        }

        return builder;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(const sensor_msgs::PointCloud2 &cloud,
                                                         std::shared_ptr<PointCloud2PtrWrapper> pointer) {
        auto cloud2_builder = slam::SchemaBuilderFromCloud2(cloud);
        auto collection2 = slam::BufferCollection::Factory(std::make_unique<slam::BufferWrapper>(
                cloud2_builder.Build(),
                (char *) (&cloud.data[0]),
                cloud.width * cloud.height,
                cloud2_builder.GetItemSize(),
                std::dynamic_pointer_cast<BufferWrapper::SmartDataPtrWrapper>(pointer)));
        return std::make_shared<slam::PointCloud>(std::move(collection2), "vertex");
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(sensor_msgs::PointCloud2Ptr &cloud) {
        return ROSCloud2ToSlamPointCloudShallow(*cloud, std::make_shared<PointCloud2PtrWrapper>(cloud));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudDeep(const sensor_msgs::PointCloud2 &cloud) {
        auto cloud2_builder = slam::SchemaBuilderFromCloud2(cloud);
        auto vector_buffer_ptr = std::make_unique<slam::VectorBuffer>(
                cloud2_builder.Build(), cloud2_builder.GetItemSize());
        auto num_items = cloud.width * cloud.height;
        vector_buffer_ptr->Reserve(num_items);
        vector_buffer_ptr->InsertItems(num_items, reinterpret_cast<const char *>(&cloud.data[0]));
        auto collection2 = slam::BufferCollection::Factory(std::move(vector_buffer_ptr));
        return std::make_shared<slam::PointCloud>(std::move(collection2), "vertex");
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(const sensor_msgs::PointCloud2ConstPtr &cloud) {
        return ROSCloud2ToSlamPointCloudShallow(*cloud, std::make_shared<PointCloud2PtrWrapper>(cloud));
    }

    /* -------------------------------------------------------------------------------------------------------------- */

} // namespace slam