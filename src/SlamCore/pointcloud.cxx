#include "SlamCore/pointcloud.h"

namespace slam {

    /* -------------------------------------------------------------------------------------------------------------- */
    bool PointCloud::IsResizable() const {
        return collection_.IsResizable();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloud::resize(size_t new_size) {
        collection_.Resize(new_size);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloud::reserve(size_t new_size) {
        collection_.Reserve(new_size);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    const BufferCollection &PointCloud::GetCollection() const {
        return collection_;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool IsValidXYZElement(const ElementInfo &info) {
        if (info.properties.size() == 3) {
            const auto &front_pty = info.properties.front();
            if (front_pty.type != FLOAT32 && front_pty.type != FLOAT64)
                return false;
            for (int i(1); i < 2; ++i) {
                if (front_pty.type != info.properties[i].type)
                    return false;
                if (front_pty.dimension != 1)
                    return false;
            }
            return true;
        }
        if (info.properties.size() == 1) {
            auto &pty = info.properties.front();
            if ((pty.type == slam::FLOAT64 || pty.type == slam::FLOAT32) &&
                pty.dimension == 1)
                return true;
        }
        return false;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    size_t PointCloud::size() const {
        return collection_.NumItemsPerBuffer();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloud::ChangeXYZElement(const std::string &element_name) {
        CHECK(collection_.HasElement(element_name))
                        << "The buffer collection of the point cloud does not have the element "
                        << element_name << std::endl;
        auto &element = collection_.GetElement(element_name);
        CHECK(element.properties.size() == 3 ||
              (element.properties.size() == 1) && element.properties[0].dimension == 3)
                        << "The element with name " << element_name << " is not a valid element" << std::endl;
        xyz_ = Field{
                int(collection_.GetItemIndex(element_name)),
                {element_name},
                {}
        };
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloud::RemoveElement(const std::string &element_name) {
        CHECK(element_name != *xyz_.element_name)
                        << "Cannot delete the XYZ points element of a point cloud" << std::endl;
        if (!GetCollection().HasElement(element_name))
            return;
        collection_.RemoveElement(element_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PointCloud PointCloud::DeepCopy() const {
        auto result = PointCloud(collection_.DeepCopy(), Field(xyz_));
        result.registered_fields_ = registered_fields_;
        result.timestamps = timestamps;
        result.intensity = intensity;
        result.rgb = rgb;
        result.normals = normals;
        result.world_point = world_point;
        result.raw_point = raw_point;
        return result;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PointCloudPtr PointCloud::DeepCopyPtr() const {
        auto result = std::make_shared<PointCloud>(collection_.DeepCopy(), Field(xyz_));
        result->registered_fields_ = registered_fields_;
        result->timestamps = timestamps;
        result->intensity = intensity;
        result->rgb = rgb;
        result->normals = normals;
        result->world_point = world_point;
        result->raw_point = raw_point;

        return result;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    const PointCloud::Field &PointCloud::GetXYZField() {
        return xyz_;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool PointCloud::IsValidField(const std::optional<Field> &field,
                                  int expected_dimension,
                                  bool check_floating_point) const {
        if (!field || field->IsItem())
            return false;
        if (field->IsElement()) {
            auto &elem_info = collection_.GetElement(*field->element_name);

            if (elem_info.properties.size() == 1) {
                if (check_floating_point) {
                    auto &pty = elem_info.properties.front();
                    if (!(pty.type == FLOAT32 || pty.type == FLOAT64))
                        return false;
                }
                return elem_info.properties.begin()->dimension == expected_dimension;
            } else {
                if (check_floating_point) {
                    for (auto &pty: elem_info.properties) {
                        if (!(pty.type == FLOAT32 || pty.type == FLOAT64))
                            return false;
                    }
                }
                return elem_info.properties.size() == expected_dimension;
            }
        }
        if (field->IsProperty()) {
            auto &pty = collection_.GetElement(*field->element_name)
                    .GetProperty(*field->property_name);
            if (check_floating_point) {
                if (!(pty.type == FLOAT32 || pty.type == FLOAT64))
                    return false;
            }
            return pty.dimension == 1;
        }
        return false;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloud::SetXYZField(PointCloud::Field &&field) {
        SLAM_CHECK_STREAM(IsValidField({field}, 3, true), "The field is an invalid XYZ field");
        xyz_ = std::move(field);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::map<std::string, slam::PointCloud::Field> PointCloud::GetRegisteredFields() const {
        auto copy_fields = registered_fields_;
        copy_fields["xyz"] = xyz_;
        if (HasIntensity())
            copy_fields["intensity"] = *intensity;
        if (HasTimestamps())
            copy_fields["timestamps"] = *timestamps;
        if (HasNormals())
            copy_fields["normals"] = *normals;
        if (HasRGB())
            copy_fields["rgb"] = *rgb;
        return copy_fields;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool PointCloud::HasField(const std::string &field_name) const {
        if (field_name == "vertex")
            return true;
        if (field_name == "intensity" && intensity.has_value())
            return true;
        if (field_name == "timestamps" && timestamps.has_value())
            return true;
        if (field_name == "normals" && normals.has_value())
            return true;
        if (field_name == "rgb" && rgb.has_value())
            return true;
        return registered_fields_.find(field_name) != registered_fields_.end();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PointCloud::Field PointCloud::GetField(const std::string &field_name) const {
        SLAM_CHECK_STREAM(HasField(field_name), "The field `" << field_name << "` is not registered")
        if (field_name == "intensity")
            return *intensity;
        if (field_name == "vertex")
            return xyz_;
        if (field_name == "timestamps")
            return *timestamps;
        if (field_name == "normals")
            return *normals;
        if (field_name == "rgb")
            return *rgb;
        return registered_fields_.at(field_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloud::RegisterFieldsFromSchema() {
        if (collection_.HasElement("properties")) {
            auto &properties_element = collection_.GetElement("properties");
            auto item_idx = int(collection_.GetItemIndex("properties"));
            if (!intensity) {
                if (collection_.HasProperty("properties", "intensity")) {
                    SetIntensityField({item_idx, "properties", "intensity"});
                }

            }
            if (!timestamps) {
                if (collection_.HasProperty("properties", "timestamp"))
                    SetTimestampsField({item_idx, "properties", "timestamp"});
                else if (collection_.HasProperty("properties", "t"))
                    SetTimestampsField({item_idx, "properties", "t"});
            }
        }

        if (!timestamps) {
            for (auto item_idx(0); item_idx < collection_.NumItemsInSchema(); ++item_idx) {
                if (timestamps)
                    break;
                auto &item_info = collection_.GetItemInfo(item_idx);
                for (auto &elem_name: item_info.item_schema.GetElementNames()) {
                    if (timestamps)
                        break;
                    auto &elem_info = item_info.item_schema.GetElementInfo(elem_name);
                    for (auto &property: elem_info.properties) {
                        if (property.property_name == "timestamp" || property.property_name == "t"
                            || property.property_name == "timestamps") {
                            if (property.dimension == 1) {
                                SetTimestampsField(Field{
                                        item_idx,
                                        elem_name,
                                        property.property_name
                                });
                                break;
                            }
                        }
                    }
                }
            }
        }

        if (!normals && collection_.HasElement("normals")) {
            auto item_idx = int(collection_.GetItemIndex("normals"));
            if (collection_.HasElement("normals")) {
                SetNormalsField({item_idx, "normals"});
            }
        }

        if (!rgb && collection_.HasElement("rgb")) {
            auto item_idx = int(collection_.GetItemIndex("rgb"));
            if (collection_.HasElement("rgb")) {
                SetRGBField({item_idx, "rgb"});
            }
        }

        // Try to find the world point from the element named 'world_point' in the schema
        if (!world_point && (collection_.HasElement("world_point"))) {
            auto item_idx = int(collection_.GetItemIndex("world_point"));
            SetWorldPointsField({item_idx, "world_point"});
        }

        if (!raw_point) {
            if (collection_.HasElement("raw_point")) {
                auto item_idx = int(collection_.GetItemIndex("raw_point"));
                SetRawPointsField({item_idx, "raw_point"});
            } else {
                // Set by default the raw point to the `xyz_` field, but only if the world_point field is not set
                // Or different from the xyz_ field (in which case xyz_ already points to the `world_points`)
                if (!HasWorldPoints() || GetWorldPointsField().element_name != xyz_.element_name)
                    SetRawPointsField(Field{xyz_});
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloud::RegisterField(std::string &&field_name, PointCloud::Field &&field, int dimension) {
        SLAM_CHECK_STREAM(IsValidField(field, dimension), "The field is not valid")
        registered_fields_.emplace(std::move(field_name), std::move(field));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PointCloudPtr PointCloud::EmptyCopyPtr() const {
        PointCloudPtr pc = std::make_shared<slam::PointCloud>(collection_.EmptyCopy(), Field(xyz_));
        pc->timestamps = timestamps;
        pc->intensity = intensity;
        pc->rgb = rgb;
        pc->normals = normals;
        pc->registered_fields_ = registered_fields_;
        return pc;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::PointCloudPtr PointCloud::SelectPoints(const std::vector<size_t> &indices) const {
        auto pc = std::make_shared<slam::PointCloud>(collection_.SelectItems(indices), Field(xyz_));
        pc->normals = normals;
        pc->intensity = intensity;
        pc->timestamps = timestamps;
        pc->rgb = rgb;
        pc->registered_fields_ = registered_fields_;
        return pc;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloud::AppendPointCloud(const PointCloud &cloud) {
        SLAM_CHECK_STREAM(IsResizable(), "Cannot append points to a non-resizable point cloud");
        SLAM_CHECK_STREAM(HaveSameSchema(cloud), "The Two point clouds do not have the same schema");
        collection_.Append(cloud.collection_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool PointCloud::HaveSameSchema(const PointCloud &other) const {
        return collection_.HasSameSchema(other.collection_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloud::RawPointsToWorldPoints(const LinearContinuousTrajectory &trajectory) {
        SLAM_CHECK_STREAM(HasRawPoints(), "The RawPoints field is not defined");
        SLAM_CHECK_STREAM(HasTimestamps(), "The Timestamps field is not defined");
        if (!HasWorldPoints())
            AddDefaultWorldPointsField();
        auto _timestamps = TimestampsProxy<double>();
        auto raw_points = RawPointsProxy<Eigen::Vector3d>();
        auto world_points = WorldPointsProxy<Eigen::Vector3d>();
        for (auto idx(0); idx < size(); ++idx) {
            double t = _timestamps[idx];
            Eigen::Vector3d _raw_point = raw_points[idx];
            world_points[idx] = trajectory.InterpolatePose(t).pose * _raw_point;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloud::RawPointsToWorldPoints(const Pose &begin_pose, const Pose &end_pose) {
        SLAM_CHECK_STREAM(HasRawPoints(), "The RawPoints field is not defined");
        SLAM_CHECK_STREAM(HasTimestamps(), "The Timestamps field is not defined");
        if (!HasWorldPoints())
            AddDefaultWorldPointsField();
        auto _timestamps = TimestampsProxy<double>();
        auto raw_points = RawPointsProxy<Eigen::Vector3d>();
        auto world_points = WorldPointsProxy<Eigen::Vector3d>();
        for (auto idx(0); idx < size(); ++idx) {
            double t = _timestamps[idx];
            Eigen::Vector3d _raw_point = raw_points[idx];
            world_points[idx] = begin_pose.InterpolatePose(end_pose, t).pose * _raw_point;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloud::RawPointsToWorldPoints(const SE3 &pose) {
        SLAM_CHECK_STREAM(HasRawPoints(), "The RawPoints field is not defined");
        if (!HasWorldPoints())
            AddDefaultWorldPointsField();
        auto raw_points = RawPointsProxy<Eigen::Vector3d>();
        auto world_points = WorldPointsProxy<Eigen::Vector3d>();
        for (auto idx(0); idx < size(); ++idx) {
            Eigen::Vector3d _raw_point = raw_points[idx];
            world_points[idx] = pose * _raw_point;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    BufferCollection::BufferInfo PointCloud::GetBufferInfoFromField(const PointCloud::Field &field) const {
        SLAM_CHECK_STREAM(field.IsElement() || field.IsProperty(),
                          "Can only query an element or a property field as buffer info from a point cloud");
        if (field.IsElement()) {
            SLAM_CHECK_STREAM(collection_.IsElementAValidBufferInfo(*field.element_name),
                              "The element is not a valid element name");
            return collection_.GetBufferInfoFromElement(*field.element_name);
        }
        return collection_.GetBufferInfoFromProperty(*field.element_name, *field.property_name);
    }

}

