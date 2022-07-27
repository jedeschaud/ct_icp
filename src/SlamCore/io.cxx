#include <fstream>
#include <iomanip>
#include <tinyply/tinyply.h>
#include <cctype>

#include "SlamCore/utils.h"
#include "SlamCore/io.h"
#include "SlamCore/generic_tools.h"
#include "SlamCore/data/buffer.h"

namespace slam {

    namespace {

        // Reads Data from a buffer into a buffer of Points
        template<typename Elem_Type_, bool WithTimestamp = false>
        void ReadPointsFromBuffer(slam::WPoint3D *points_buffer,
                                  const unsigned char *buffer,
                                  size_t count,
                                  const int offset_of_vertex =
                                  offsetof(slam::WPoint3D, raw_point) +
                                  offsetof(slam::Point3D, point)) {

            typedef std::array<Elem_Type_, WithTimestamp ? 4 : 3> element;
            auto *elem_buffer = reinterpret_cast<const element *>(buffer);

            double *vertex_ptr = nullptr;
            double *timestamp_ptr = nullptr;

            element copied_element;
            for (int idx(0); idx < count; idx++) {
                auto &new_point = points_buffer[idx];
                {
                    auto *_ptr = reinterpret_cast<unsigned char *>(&new_point);
                    vertex_ptr = reinterpret_cast<double *>(_ptr + offset_of_vertex);
                    if (WithTimestamp)
                        timestamp_ptr = reinterpret_cast<double *>(_ptr +
                                                                   offsetof(slam::WPoint3D, raw_point.timestamp));
                }
                copied_element = elem_buffer[idx];
                for (int dim(0); dim < 3; dim++)
                    vertex_ptr[dim] = static_cast<double>(copied_element[dim]);
                if (WithTimestamp)
                    *timestamp_ptr = static_cast<double>(copied_element[3]);
            }
        }
    }

    namespace {

        struct memory_buffer : public std::streambuf {
            char *p_start{nullptr};
            char *p_end{nullptr};
            size_t size;

            memory_buffer(char const *first_elem, size_t size)
                    : p_start(const_cast<char *>(first_elem)), p_end(p_start + size), size(size) {
                setg(p_start, p_start, p_end);
            }

            pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which) override {
                if (dir == std::ios_base::cur) gbump(static_cast<int>(off));
                else setg(p_start, (dir == std::ios_base::beg ? p_start : p_end) + off, p_end);
                return gptr() - p_start;
            }

            pos_type seekpos(pos_type pos, std::ios_base::openmode which) override {
                return seekoff(pos, std::ios_base::beg, which);
            }
        };

        struct memory_stream : virtual memory_buffer, public std::istream {
            memory_stream(char const *first_elem, size_t size)
                    : memory_buffer(first_elem, size), std::istream(static_cast<std::streambuf *>(this)) {}
        };

    }

    /* -------------------------------------------------------------------------------------------------------------- */
//    std::vector<WPoint3D> ReadPLYFromStream(std::istream &stream,
//                                            const PointCloudSchema &schema) {
//
//        const auto &bytes = ReadStreamAsByteArray(stream);
//        auto _memory_stream = std::make_unique<memory_stream>((char *) bytes.data(), bytes.size());
//
//        std::vector<WPoint3D> points;
//        tinyply::PlyFile file;
//        try {
//            file.parse_header(*_memory_stream);
//
//            auto &raw_point_elem = schema.raw_point_element;
//            std::vector<std::string> vertex_properties{raw_point_elem.x_property,
//                                                       raw_point_elem.y_property,
//                                                       raw_point_elem.z_property};
//            bool timestamp_in_vertex = false;
//            bool with_timestamp = schema.timestamp_element_and_property.has_value();
//            if (with_timestamp && schema.timestamp_element_and_property->first == raw_point_elem.ply_element) {
//                vertex_properties.push_back(schema.timestamp_element_and_property->second);
//                timestamp_in_vertex = true;
//            }
//            auto main_element_ptr = file.request_properties_from_element(raw_point_elem.ply_element,
//                                                                         vertex_properties);
//
//            std::shared_ptr<tinyply::PlyData> timestamp_ptr = nullptr;
//            std::shared_ptr<tinyply::PlyData> world_point_ptr = nullptr;
//
//            file.read(*_memory_stream);
//
//            const auto kCount = main_element_ptr->count;
//            const auto *buffer = main_element_ptr->buffer.get();
//            points.resize(kCount);
//            if (main_element_ptr->t == tinyply::Type::FLOAT64) {
//                if (timestamp_in_vertex)
//                    ReadPointsFromBuffer<double, true>(&points[0], buffer, kCount);
//                else
//                    ReadPointsFromBuffer<double, false>(&points[0], buffer, kCount);
//            }
//            if (main_element_ptr->t == tinyply::Type::FLOAT32) {
//                if (timestamp_in_vertex)
//                    ReadPointsFromBuffer<float, true>(&points[0], buffer, kCount);
//                else
//                    ReadPointsFromBuffer<float, false>(&points[0], buffer, kCount);
//            }
//
//        } catch (...) {
//            std::cout << "Failed to read a ply file." << std::endl;
//            for (auto &info: file.get_info())
//                std::cout << info << std::endl;
//            throw;
//        }
//
//        return points;
//    }

    /* -------------------------------------------------------------------------------------------------------------- */
//    std::vector<slam::WPoint3D> ReadPLYFromFile(const std::string &file_path, const PointCloudSchema &schema) {
//#if WITH_STD_FILESYSTEM
//        CHECK(fs::exists(file_path) &&
//              fs::is_regular_file(file_path)) << "The file " << file_path
//                                              << " does not exist" << std::endl;
//#endif
//        std::ifstream binary_file(file_path.c_str(), std::ios::binary);
//        CHECK(binary_file.is_open()) << "Could not open file located at " << file_path << std::endl;
//
//        auto points = ReadPLYFromStream(binary_file, schema);
//        binary_file.close();
//        return points;
//    }

    /* -------------------------------------------------------------------------------------------------------------- */
//    tinyply::Type SlamTypeToTinyPLY(PointCloudSchema::TYPE type) {
//        switch (type) {
//            case PointCloudSchema::FLOAT:
//                return tinyply::Type::FLOAT32;
//            default:
//                return tinyply::Type::FLOAT64;
//        }
//    }

    namespace {
        template<typename SourceT, typename DestT, int DIM, int offset>
        void CopyPropertyToBuffer(const std::vector<WPoint3D> &points,
                                  std::vector<uint8_t> &buffer) {
            CHECK(buffer.size() == points.size() * sizeof(DestT) * DIM) << "Mismatch sizes";
            auto elements = transform_vector<std::array<DestT, DIM>>
                    (points, [](auto &point) {
                        std::array<DestT, DIM> data;
                        auto *ptr = reinterpret_cast<const SourceT *>(reinterpret_cast<const uint8_t *>(&point) +
                                                                      offset);
                        for (int i(0); i < DIM; ++i) {
                            data[i] = static_cast<DestT>(ptr[i]);
                        }
                        return data;
                    });
            auto data_ptr = reinterpret_cast<const uint8_t *>(&elements[0]);
            std::copy(data_ptr, data_ptr + buffer.size(), buffer.begin());
        }

    }

//    /* -------------------------------------------------------------------------------------------------------------- */
//    void WritePLY(std::ostream &output_file,
//                  const std::vector<slam::WPoint3D> &points,
//                  const PointCloudSchema &schema) {
//
//        tinyply::PlyFile file;
//
//        const int kDataSize = schema.raw_point_write_type == PointCloudSchema::FLOAT ? sizeof(float) : sizeof(double);
//        std::vector<uint8_t> data(points.size() * kDataSize * 3);
//        std::unique_ptr<std::vector<uint8_t>> data_timestamps = nullptr;
//
//        if (schema.raw_point_write_type == PointCloudSchema::FLOAT) {
//            CopyPropertyToBuffer<double, float, 3, offsetof(slam::WPoint3D, raw_point)>(points, data);
//        } else
//            CopyPropertyToBuffer<double, double, 3, offsetof(slam::WPoint3D, raw_point)>(points, data);
//
//
//        file.add_properties_to_element(schema.raw_point_element.ply_element, {
//                                               schema.raw_point_element.x_property,
//                                               schema.raw_point_element.y_property,
//                                               schema.raw_point_element.z_property
//                                       },
//                                       SlamTypeToTinyPLY(schema.raw_point_write_type),
//                                       points.size(),
//                                       &data[0],
//                                       tinyply::Type::INVALID, 0);
//
//        if (schema.timestamp_element_and_property.has_value()) {
//            auto &pair = schema.timestamp_element_and_property;
//            if (pair->first == schema.raw_point_element.ply_element)
//                CHECK(schema.raw_point_write_type == schema.raw_point_write_type);
//
//            const int kTDataSize = schema.timestamp_write_type == PointCloudSchema::FLOAT ?
//                                   sizeof(float) : sizeof(double);
//            data_timestamps = std::make_unique<std::vector<std::uint8_t>>(points.size() * kTDataSize);
//
//
//            if (schema.timestamp_write_type == PointCloudSchema::FLOAT)
//                CopyPropertyToBuffer<double, float, 1,
//                        offsetof(slam::WPoint3D, raw_point.timestamp)>(points, *data_timestamps);
//            else
//                CopyPropertyToBuffer<double, double, 1,
//                        offsetof(slam::WPoint3D, raw_point.timestamp)>(points, *data_timestamps);
//
//            file.add_properties_to_element(pair->first, {pair->second},
//                                           SlamTypeToTinyPLY(schema.timestamp_write_type), points.size(),
//                                           &(*data_timestamps)[0], tinyply::Type::INVALID, 0);
//        }
//
//        file.write(output_file, true);
//    }

    /* -------------------------------------------------------------------------------------------------------------- */
//    void WritePLY(const std::string &file, const std::vector<slam::WPoint3D> &points, const PointCloudSchema &schema) {
//        std::ofstream output_stream(file, std::ios::out | std::ios::binary);
//        WritePLY(output_stream, points, std::move(schema));
//        output_stream.close();
//    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<uint8_t> ReadStreamAsByteArray(std::istream &stream) {
        std::vector<uint8_t> fileBufferBytes;
        stream.seekg(0, std::ios::end);
        size_t sizeBytes = stream.tellg();
        stream.seekg(0, std::ios::beg);
        fileBufferBytes.resize(sizeBytes);
        CHECK(stream.read((char *) fileBufferBytes.data(), sizeBytes));
        return fileBufferBytes;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<Pose> LoadPosesKITTIFormat(const std::string &file_path) {
        std::vector<Pose> poses;
        std::ifstream pFile(file_path);
        if (pFile.is_open()) {
            size_t iter(0);
            Pose pose;
            while (!pFile.eof()) {
                std::string line;
                std::getline(pFile, line);
                if (line.empty()) continue;
                std::stringstream ss(line);

                pose.dest_frame_id = iter;
                pose.dest_timestamp = static_cast<double>(iter) * 0.1;
                Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
                ss >> P(0, 0) >> P(0, 1) >> P(0, 2) >> P(0, 3) >> P(1, 0) >> P(1, 1) >> P(1, 2) >> P(1, 3) >> P(2, 0)
                   >> P(2, 1) >> P(2, 2) >> P(2, 3);

                pose.pose.quat = Eigen::Quaterniond(P.block<3, 3>(0, 0));
                pose.pose.tr = Eigen::Vector3d(P.block<3, 1>(0, 3));
                poses.push_back(pose);
            }
            pFile.close();
        } else {
            std::cout << "Unable to open file" << std::endl;
        }
        return poses;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool SavePosesKITTIFormat(const std::string &file_path, const std::vector<Pose> &trajectory) {
        auto parent_path = fs::path(file_path).parent_path();
        if (!exists(parent_path))
            fs::create_directories(parent_path);
        std::ofstream pFile(file_path);
        if (pFile.is_open()) {
            pFile << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
            Eigen::Matrix4d pose;
            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            for (auto &_pose: trajectory) {
                pose = _pose.Matrix();
                R = pose.block<3, 3>(0, 0);
                t = pose.block<3, 1>(0, 3);

                pFile << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << t(0)
                      << " " << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " "
                      << t(1) << " " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2)
                      << " " << t(2) << std::endl;
            }
            pFile.close();

            std::cout << "Saved Poses to " << file_path << std::endl;
            return true;
        }
        std::cout << "Could not open file " << file_path << std::endl;
        return false;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    namespace {
        struct Float64PosesData {
            double qs[4], tr[3], ref_t, dest_t;
        };

        typedef std::pair<int, int> pair_fid;
    }

    void SavePosesAsPLY(std::ostream &output_file, const std::vector<Pose> &poses) {
        tinyply::PlyFile file;


        std::vector<Float64PosesData> se3_poses = transform_vector<Float64PosesData>(poses, [](const auto &pose) {
            auto &quat = pose.pose.quat;
            auto &tr = pose.pose.tr;
            return Float64PosesData{{quat.coeffs()[0], quat.coeffs()[1], quat.coeffs()[2], quat.coeffs()[3]},
                                    {tr[0], tr[1], tr[2]}, pose.ref_timestamp, pose.dest_timestamp};
        });

        std::vector<pair_fid> frame_ids = transform_vector<pair_fid>(poses, [](const auto &pose) {
            return std::make_pair(static_cast<int>(pose.ref_frame_id), static_cast<int>(pose.dest_frame_id));
        });


        file.add_properties_to_element("pose", {
                                               "qx", "qy", "qz", "qw",
                                               "x", "y", "z", "ref_t", "dest_t"
                                       },
                                       tinyply::Type::FLOAT64,
                                       poses.size(),
                                       reinterpret_cast<uint8_t *>(&se3_poses[0].qs[0]),
                                       tinyply::Type::INVALID, 0);
        file.add_properties_to_element("pose", {
                                               "ref_fid", "dest_fid"
                                       }, tinyply::Type::INT32, poses.size(),
                                       reinterpret_cast<uint8_t *>(&frame_ids[0].first),
                                       tinyply::Type::INVALID, 0);
        file.write(output_file, true);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void SavePosesAsPLY(const std::string &output_file_path, const std::vector<Pose> &poses) {
        std::ofstream output_stream(output_file_path, std::ios::out | std::ios::binary);
        SavePosesAsPLY(output_stream, poses);
        output_stream.close();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::Pose> ReadPosesFromPLY(std::istream &stream) {
        std::vector<slam::Pose> poses;

        const auto &bytes = ReadStreamAsByteArray(stream);
        auto _memory_stream = std::make_unique<memory_stream>((char *) bytes.data(), bytes.size());

        tinyply::PlyFile file;
        try {
            file.parse_header(*_memory_stream);
            auto quat_tr_timestamps = file.request_properties_from_element("pose",
                                                                           {
                                                                                   "qx", "qy", "qz", "qw", "x", "y",
                                                                                   "z",
                                                                                   "ref_t", "dest_t"
                                                                           });
            auto fids = file.request_properties_from_element("pose",
                                                             {"ref_fid", "dest_fid"});
            file.read(*_memory_stream);

            const auto kCount = quat_tr_timestamps->count;
            const auto *quat_tr_ts_raw_buffer = quat_tr_timestamps->buffer.get();
            const auto *fids_raw_buffer = fids->buffer.get();
            CHECK(quat_tr_timestamps->t == tinyply::Type::FLOAT64)
                            << "Invalid types in poses file (expected Float64)" << std::endl;
            poses.resize(kCount);

            auto *quat_tr_ts_buffer = reinterpret_cast<const Float64PosesData *>(quat_tr_ts_raw_buffer);
            auto *fids_buffer = reinterpret_cast<const pair_fid *>(fids_raw_buffer);

            Pose new_pose;
            for (int idx(0); idx < kCount; idx++) {
                const auto &elem = quat_tr_ts_buffer[idx];
                const auto &_fids = fids_buffer[idx];

                for (int i(0); i < 4; ++i)
                    new_pose.pose.quat.coeffs()[i] = elem.qs[i];
                for (int i(0); i < 3; ++i)
                    new_pose.pose.tr[i] = elem.tr[i];
                new_pose.dest_timestamp = elem.dest_t;
                new_pose.ref_timestamp = elem.ref_t;
                new_pose.ref_frame_id = static_cast<slam::frame_id_t>(_fids.first);
                new_pose.dest_frame_id = static_cast<slam::frame_id_t>(_fids.second);
                poses[idx] = new_pose;
            }
        } catch (...) {
            std::cout << "Failed to read a ply file." << std::endl;
            for (auto &info: file.get_info())
                std::cout << info << std::endl;
            throw;
        }

        return poses;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::Pose> ReadPosesFromPLY(const std::string &file_path) {
        CHECK(fs::exists(file_path) &&
              fs::is_regular_file(file_path)) << "The file " << file_path
                                              << " does not exist" << std::endl;
        std::ifstream input_file(file_path, std::ios::binary);
        CHECK(input_file.is_open()) << "Could not open file on disk at location: " << file_path << std::endl;
        auto poses = ReadPosesFromPLY(input_file);
        input_file.close();
        return poses;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
//    void slam::WritePLY(const std::string &file, const PointCloud &pointcloud) {
//        std::ofstream ofile(file);
//        WritePLY(ofile, pointcloud);
//    }

    namespace {

        struct _PropertyData {
            std::vector<std::uint8_t> data;
            std::vector<std::string> properties; // The properties name for multi dimensionality properties
            tinyply::Type type = tinyply::Type::INVALID;

            _PropertyData() = default;
        };

    }

    /* -------------------------------------------------------------------------------------------------------------- */
//    void slam::WritePLY(std::ostream &output_file, const PointCloud &pointcloud) {
//        tinyply::PlyFile file;
//
//        auto num_items = pointcloud.size();
//        auto &collection = pointcloud.GetCollection();
//        std::map<std::string, std::list<_Property>> properties; // The properties extracted from the point cloud
//
//        // Copies the data of the point cloud into buffer for PLY files
//        for (auto item_id(0); item_id < pointcloud.GetCollection().NumItemsInSchema(); item_id++) {
//            auto &item_info = collection.GetItemInfo(item_id);
//            auto &schema = item_info.item_schema;
//            for (auto &element_name: schema.GetElementNames()) {
//                auto ply_element_name = element_name;
//                // Change the name of element if there is a conflict
//                int i(0);
//                while (properties.find(ply_element_name) != properties.end()) {
//                    ply_element_name = element_name + "_" + std::to_string(i++);
//                }
//                if (ply_element_name != element_name) {
//                    LOG(WARNING) << "The element `" << element_name << "` was changed to `" <<
//                                 ply_element_name << "` due to conflicting element names" << std::endl;
//                }
//
//                auto &elem_info = collection.GetElement("element_name");
//                for (auto &property: elem_info.properties) {
//                    auto pty_size = property.Size();
//                    _Property _property;
//                    _property.type = SlamPropertyToTinyPLYType(property.type);
//                    if (_property.type == tinyply::Type::INVALID) {
//                        LOG(WARNING) << "The element `" << element_name << "` has a property (`" <<
//                                     property.property_name << "`) with type [" << slam::PropertyTypeChar(property.type)
//                                     << "] not supported by tinyply (it will be ignored)";
//                        continue;
//                    }
//                    if (property.dimension == 1)
//                        _property.properties = {property.property_name};
//                    else {
//                        _property.properties.resize(property.dimension);
//                        for (auto k(0); k < property.dimension; k++) {
//                            _property.properties[k] = property.property_name + "_" + std::to_string(k);
//                        }
//                    }
//                    properties[ply_element_name].emplace_back(std::move(_property));
//                    _property.data.resize(pty_size * num_items);
//
//                    // Copy the data inside the buffer
//                    for (auto item_id(0); item_id < num_items; ++item_id) {
//                        auto *buffer_ptr = item_info.parent_buffer->view_data_ptr + item_info.item_size * item_id +
//                                           property.offset_in_elem +
//                                           elem_info.offset_in_item;
//                        std::copy(buffer_ptr,
//                                  buffer_ptr + pty_size,
//                                  &_property.data[pty_size * item_id]);
//                    }
//                }
//            }
//        }
//
//        // Add all properties to the PLY file
//        for (auto &element: properties) {
//            for (auto &ptry: element.second) {
//                file.add_properties_to_element(element.first,
//                                               ptry.properties, ptry.type,
//                                               num_items,
//                                               ptry.data.data(),
//                                               tinyply::Type::INVALID, 0);
//            }
//        }
//        file.write(output_file, true);
//    }

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::PROPERTY_TYPE TinyPLYToSlamPropertyType(tinyply::Type type) {
        switch (type) {
            // Floats
            case tinyply::Type::FLOAT64:
                return slam::FLOAT64;
            case tinyply::Type::FLOAT32:
                return slam::FLOAT32;

                // INTs
            case tinyply::Type::INT8:
                return slam::INT8;
            case tinyply::Type::INT16:
                return slam::INT16;
            case tinyply::Type::INT32:
                return slam::INT32;

                // Unsigned INTs
            case tinyply::Type::UINT8:
                return slam::UINT8;
            case tinyply::Type::UINT16:
                return slam::UINT16;
            case tinyply::Type::UINT32:
                return slam::UINT32;
            default:
                throw std::runtime_error("[tinyply to slam] Bad type conversion");
        }
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    tinyply::Type SlamPropertyToTinyPLYType(slam::PROPERTY_TYPE type) {
        switch (type) {
            // Floats
            case slam::FLOAT64:
                return tinyply::Type::FLOAT64;
            case slam::FLOAT32:
                return tinyply::Type::FLOAT32;

                // INTs
            case slam::INT8:
                return tinyply::Type::INT8;
            case slam::INT16:
                return tinyply::Type::INT16;
            case slam::INT32:
                return tinyply::Type::INT32;

                // Unsigned INTs
            case slam::UINT8:
                return tinyply::Type::UINT8;
            case slam::UINT16:
                return tinyply::Type::UINT16;
            case slam::UINT32:
                return tinyply::Type::UINT32;
            default:
                return tinyply::Type::INVALID;
        }
    }

#define CONVERT_SOURCE_TO_DEST(src_ptr, _src_type, dest_ptr, _dest_type) \
    (*reinterpret_cast<_dest_type *>(dest_ptr)) = _dest_type(*reinterpret_cast<const _src_type *>(src_ptr));

#define TINYPLY_TO_SLAM__CASE_SLAM(dest_slam_pty, _dest_slam_type, _src_tinyply_type) \
    case dest_slam_pty : \
    CONVERT_SOURCE_TO_DEST(source, _src_tinyply_type, dest, _dest_slam_type)\
    break;

#define SLAM_TO_TINYPLY__CASE_SLAM(src_slam_pty, _src_slam_type, _dest_tinyply_type) \
    case src_slam_pty: \
    CONVERT_SOURCE_TO_DEST(source, _src_slam_type, dest, _dest_tinyply_type)\
    break;

#define TINYPLY_TO_SLAM__CASE_TINYPLY(tinyply_type, _type) \
    case tinyply_type: { switch(dest_type) {                  \
        TINYPLY_TO_SLAM__CASE_SLAM(slam::PROPERTY_TYPE::FLOAT64, double, _type); \
        TINYPLY_TO_SLAM__CASE_SLAM(slam::PROPERTY_TYPE::FLOAT32, float, _type);  \
        TINYPLY_TO_SLAM__CASE_SLAM(slam::PROPERTY_TYPE::INT64, std::int64_t , _type);  \
        TINYPLY_TO_SLAM__CASE_SLAM(slam::PROPERTY_TYPE::UINT64, std::uint64_t , _type);  \
        TINYPLY_TO_SLAM__CASE_SLAM(slam::PROPERTY_TYPE::INT32, std::int32_t , _type);  \
        TINYPLY_TO_SLAM__CASE_SLAM(slam::PROPERTY_TYPE::UINT32, std::uint32_t , _type);  \
        TINYPLY_TO_SLAM__CASE_SLAM(slam::PROPERTY_TYPE::INT16, std::int16_t , _type); \
        TINYPLY_TO_SLAM__CASE_SLAM(slam::PROPERTY_TYPE::UINT16, std::uint16_t , _type); \
        TINYPLY_TO_SLAM__CASE_SLAM(slam::PROPERTY_TYPE::INT8, std::int8_t , _type);\
        TINYPLY_TO_SLAM__CASE_SLAM(slam::PROPERTY_TYPE::UINT8, std::uint8_t , _type);\
    } } break;

#define SLAM_TO_TINYPLY__CASE_TINYPLY(tinyply_type, _type) \
    case tinyply_type: { switch(source_type) {                  \
        SLAM_TO_TINYPLY__CASE_SLAM(slam::PROPERTY_TYPE::FLOAT64, double, _type); \
        SLAM_TO_TINYPLY__CASE_SLAM(slam::PROPERTY_TYPE::FLOAT32, float, _type);  \
        SLAM_TO_TINYPLY__CASE_SLAM(slam::PROPERTY_TYPE::INT64, std::int64_t , _type);  \
        SLAM_TO_TINYPLY__CASE_SLAM(slam::PROPERTY_TYPE::UINT64, std::uint64_t , _type);  \
        SLAM_TO_TINYPLY__CASE_SLAM(slam::PROPERTY_TYPE::INT32, std::int32_t , _type);  \
        SLAM_TO_TINYPLY__CASE_SLAM(slam::PROPERTY_TYPE::UINT32, std::uint32_t , _type);  \
        SLAM_TO_TINYPLY__CASE_SLAM(slam::PROPERTY_TYPE::INT16, std::int16_t , _type); \
        SLAM_TO_TINYPLY__CASE_SLAM(slam::PROPERTY_TYPE::UINT16, std::uint16_t , _type); \
        SLAM_TO_TINYPLY__CASE_SLAM(slam::PROPERTY_TYPE::INT8, std::int8_t , _type);\
        SLAM_TO_TINYPLY__CASE_SLAM(slam::PROPERTY_TYPE::UINT8, std::uint8_t , _type);\
    } } break;


    /* -------------------------------------------------------------------------------------------------------------- */
    void TinyPLYScalarToSlamScalar(const char *source, tinyply::Type source_type,
                                   char *dest, slam::PROPERTY_TYPE dest_type) {
        switch (source_type) {
            TINYPLY_TO_SLAM__CASE_TINYPLY (tinyply::Type::FLOAT64, double)
            TINYPLY_TO_SLAM__CASE_TINYPLY (tinyply::Type::FLOAT32, float)
            TINYPLY_TO_SLAM__CASE_TINYPLY (tinyply::Type::INT32, std::int32_t)
            TINYPLY_TO_SLAM__CASE_TINYPLY (tinyply::Type::UINT32, std::uint32_t)
            TINYPLY_TO_SLAM__CASE_TINYPLY (tinyply::Type::INT16, std::int16_t)
            TINYPLY_TO_SLAM__CASE_TINYPLY (tinyply::Type::UINT16, std::uint16_t)
            TINYPLY_TO_SLAM__CASE_TINYPLY (tinyply::Type::INT8, std::int8_t)
            TINYPLY_TO_SLAM__CASE_TINYPLY (tinyply::Type::UINT8, std::uint8_t)
            case tinyply::Type::INVALID:
                throw std::runtime_error("Source Type is the tinyply::INVALID type, cannot copy scalar into buffer.");
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void SlamScalarToTinyPLYScalar(const char *source, slam::PROPERTY_TYPE source_type,
                                   char *dest, tinyply::Type dest_type) {
        switch (dest_type) {
            SLAM_TO_TINYPLY__CASE_TINYPLY (tinyply::Type::FLOAT64, double)
            SLAM_TO_TINYPLY__CASE_TINYPLY (tinyply::Type::FLOAT32, float)
            SLAM_TO_TINYPLY__CASE_TINYPLY (tinyply::Type::INT32, std::int32_t)
            SLAM_TO_TINYPLY__CASE_TINYPLY (tinyply::Type::UINT32, std::uint32_t)
            SLAM_TO_TINYPLY__CASE_TINYPLY (tinyply::Type::INT16, std::int16_t)
            SLAM_TO_TINYPLY__CASE_TINYPLY (tinyply::Type::UINT16, std::uint16_t)
            SLAM_TO_TINYPLY__CASE_TINYPLY (tinyply::Type::INT8, std::int8_t)
            SLAM_TO_TINYPLY__CASE_TINYPLY (tinyply::Type::UINT8, std::uint8_t)
            case tinyply::Type::INVALID:
                throw std::runtime_error("Source Type is the tinyply::INVALID type, cannot copy scalar into buffer.");
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    namespace {
        void WriteCollectionToPLY(std::ostream &output_file,
                                  const BufferCollection &collection,
                                  const PLYSchemaMapper &schema) {
            tinyply::PlyFile file;

            auto check_schema = [](bool predicate) {
                CHECK(predicate) << "[IOError] Bad schema, does not match the point cloud";
            };

            const auto kNumItems = collection.NumItemsPerBuffer();

            std::map<std::string, std::list<_PropertyData>> element_to_properties;
            for (auto &elem: schema.GetPLYElementNames()) {
                for (auto &pty_name: schema.GetPLYProperties(elem)) {
                    auto &pty = schema.GetPLYProperty(elem, pty_name);

                    check_schema(collection.NumItemsInSchema() > pty.item_id);
                    auto &item_info = collection.GetItemInfo(pty.item_id);

                    check_schema(item_info.HasElement(pty.slam_element_name));
                    auto &elem_info = item_info.item_schema.GetElementInfo(pty.slam_element_name);
                    check_schema(elem_info.HasProperty(pty.slam_property_name));
                    auto &slam_pty = elem_info.GetProperty(pty.slam_property_name);
                    check_schema(pty.property_dim < slam_pty.dimension);

                    _PropertyData property_data;
                    property_data.properties.push_back(pty.ply_property_name);
                    property_data.type = pty.pty_type;

                    const auto kSlamPtyType = pty.slam_pty_type;
                    const auto kTinyplyPtyType = pty.pty_type;
                    const auto kItemSize = item_info.item_size;
                    const auto kElemOffset = elem_info.offset_in_item;
                    const auto kPropertyOffset = slam_pty.offset_in_elem +
                                                 slam::PropertySize(slam_pty.type) * pty.property_dim;
                    const auto kTinyPLYPtySize = SizeOfTinyPLYType(pty.pty_type);
                    property_data.data.resize(kNumItems * kTinyPLYPtySize);

                    auto *buffer = item_info.parent_buffer->view_data_ptr + kElemOffset + kPropertyOffset;
                    for (auto idx(0); idx < kNumItems; idx++) {
                        const char *slam_pty_scalar = &buffer[idx * kItemSize];
                        char *tinyply_pty_scalar = reinterpret_cast<char *>(&property_data.data[kTinyPLYPtySize * idx]);
                        // Copies each scalar to the destination TinyPLY buffer
                        SlamScalarToTinyPLYScalar(slam_pty_scalar, kSlamPtyType, tinyply_pty_scalar, kTinyplyPtyType);
                    }
                    element_to_properties[elem].emplace_back(std::move(property_data));
                }
            }

            for (auto &elem_and_properties: element_to_properties) {
                for (auto &pty: elem_and_properties.second) {
                    file.add_properties_to_element(elem_and_properties.first,
                                                   pty.properties,
                                                   pty.type,
                                                   kNumItems, pty.data.data(), tinyply::Type::INVALID, 0);
                }
            }

            file.write(output_file, true);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void WritePLY(std::ostream &output_file,
                  const PointCloud &pointcloud,
                  const PLYSchemaMapper &schema) {
        WriteCollectionToPLY(output_file, pointcloud.GetCollection(), schema);
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    void WritePLY(const std::string &file, const PointCloud &points, const PLYSchemaMapper &schema) {
        std::ofstream output_file(file);
        WritePLY(output_file, points, schema);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void WritePLY(const std::string &file, const std::vector<slam::WPoint3D> &points) {
        std::ofstream output_file(file);
        WritePLY(output_file, points);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void WritePLY(std::ostream &output_file, const std::vector<slam::WPoint3D> &points) {
        std::string xyz_element = "world_point";
        auto pc = slam::PointCloud::WrapConstVector(points, slam::WPoint3D::DefaultSchema(), xyz_element);

        std::vector<slam::ItemSchema> schemas(1);
        schemas[0] = pc.GetCollection().GetItemInfo(0).item_schema;
        WritePLY(output_file, pc, slam::PLYSchemaMapper::BuildDefaultFromSchema(schemas));
    }

    namespace {

        struct _ReadPropertyData {
            std::shared_ptr<tinyply::PlyData> data = nullptr;
            const PLYSchemaMapper::PLYProperty *property = nullptr;
        };

        struct PlyFileWrapper {

            std::vector<std::uint8_t> bytes;
            std::unique_ptr<memory_stream> stream = nullptr;
            tinyply::PlyFile file;

            ~PlyFileWrapper() {
                stream = nullptr;
                bytes.clear();
            }

            void ReadStream(std::istream &file_stream) {
                bytes = ReadStreamAsByteArray(file_stream);
                stream = std::make_unique<memory_stream>((char *) bytes.data(), bytes.size());
                file.parse_header(*stream);
            }

            slam::BufferCollection GetCollection(const PLYSchemaMapper &schema) {
                // Precise the properties that need to be requested
                std::vector<_ReadPropertyData> properties_mapping;
                for (auto &elem_name: schema.GetPLYElementNames()) {
                    for (auto &pty_name: schema.GetPLYProperties(elem_name)) {

                        auto &ply_pty = schema.GetPLYProperty(elem_name, pty_name);
                        properties_mapping.push_back(
                                {file.request_properties_from_element(elem_name, {ply_pty.ply_property_name}),
                                 &ply_pty});
                    }
                }
                file.read(*stream);

                // Verify that all buffers for each property is valid
                size_t num_items = properties_mapping.front().data->count;
                for (auto &pty: properties_mapping) {
                    CHECK(pty.data->t == pty.property->pty_type) << "Inconsistent types between schema and PLYFile";
                    CHECK(pty.data->count == num_items) << "Inconsistent properties count";
                    CHECK(pty.data->buffer.get() != nullptr)
                                    << "An error occurred while reading the PLY file" << std::endl;
                }

                auto collection = schema.AllocateBufferCollection(num_items);
                // For each property copy the content in the buffer
                for (auto &_pty: properties_mapping) {
                    auto &pty = *_pty.property;
                    auto &item_info = collection.GetItemInfo(pty.item_id);
                    const auto &elem_info = item_info.item_schema.GetElementInfo(pty.slam_element_name);
                    const auto &pty_info = elem_info.GetProperty(pty.slam_property_name);

                    // Offsets for the item buffers
                    const int kItemSize = item_info.item_size;
                    const int kElemOffset = elem_info.offset_in_item;
                    const int kPropertyOffset = pty_info.offset_in_elem +
                                                slam::PropertySize(pty_info.type) * pty.property_dim;
                    const int kTinyPlyElemSize = slam::SizeOfTinyPLYType(pty.pty_type);

                    // Copy / Convert the scalar values from the tinyply buffers to the slam buffers
                    std::uint8_t *tinyply_buffer = _pty.data->buffer.get();
                    char *slam_buffer = item_info.parent_buffer->view_data_ptr + kElemOffset + kPropertyOffset;
                    for (auto idx(0); idx < num_items; ++idx) {
                        const char *source_ptr = reinterpret_cast<const char *>(&tinyply_buffer[idx *
                                                                                                kTinyPlyElemSize]);
                        char *dest_ptr = &slam_buffer[idx * kItemSize];
                        TinyPLYScalarToSlamScalar(source_ptr, pty.pty_type, dest_ptr, pty.slam_pty_type);
                    }
                }
                return collection;
            }

            PointCloudPtr ReadPointCloud(const PLYSchemaMapper &schema) {
                auto collection = std::move(GetCollection(schema));
                CHECK(!schema.XYZElementNameConst().empty()) << "The XYZ Element is not specified !" << std::endl;
                CHECK(IsValidXYZElement(collection.GetElement(schema.XYZElementNameConst())));
                return std::make_shared<slam::PointCloud>(std::move(collection),
                                                          std::string(schema.XYZElementNameConst()));
            }
        };
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PointCloudPtr ReadPointCloudFromPLY(std::istream &input_file,
                                        const PLYSchemaMapper &schema) {
        PlyFileWrapper wrapper;
        wrapper.ReadStream(input_file);
        auto pc = wrapper.ReadPointCloud(schema);
        pc->RegisterFieldsFromSchema();
        return pc;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper slam::WPoint3D::FloatSchemaMapper() {
        auto builder = PLYSchemaMapper::Builder::BuilderFromItemSchema(slam::WPoint3D::DefaultSchema(), false);

        builder.AddPLYProperty({"vertex", "x", tinyply::Type::FLOAT32,
                                0, "raw_point", "x", slam::FLOAT64, 0})
                .AddPLYProperty({"vertex", "y", tinyply::Type::FLOAT32,
                                 0, "raw_point", "y", slam::FLOAT64, 0})
                .AddPLYProperty({"vertex", "z", tinyply::Type::FLOAT32,
                                 0, "raw_point", "z", slam::FLOAT64, 0})
                .AddPLYProperty({"vertex", "timestamp", tinyply::Type::FLOAT32,
                                 0, "xyzt", "t", slam::FLOAT64, 0});
        builder.SetXYZElementName("raw_point");
        return builder.Build();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper WPoint3D::DoubleSchemaMapper() {
        auto builder = PLYSchemaMapper::Builder::BuilderFromItemSchema(slam::WPoint3D::DefaultSchema(), false);
        builder.AddPLYProperty({"vertex", "x", tinyply::Type::FLOAT64,
                                0, "raw_point", "x", slam::FLOAT64, 0})
                .AddPLYProperty({"vertex", "y", tinyply::Type::FLOAT64,
                                 0, "raw_point", "y", slam::FLOAT64, 0})
                .AddPLYProperty({"vertex", "z", tinyply::Type::FLOAT64,
                                 0, "raw_point", "z", slam::FLOAT64, 0})
                .AddPLYProperty({"vertex", "timestamp", tinyply::Type::FLOAT64,
                                 0, "xyzt", "t", slam::FLOAT64, 0});
        builder.SetXYZElementName("raw_point");
        return builder.Build();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PointCloudPtr ReadPointCloudFromPLY(const std::string &file, const PLYSchemaMapper &schema) {
        std::ifstream input_file(file);
        return slam::ReadPointCloudFromPLY(input_file, schema);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::WPoint3D> ReadWPoint3DVectorFromPLYStream(std::istream &stream, const PLYSchemaMapper &schema) {
        CHECK(schema.GetItemSchemas().size() == 1 &&
              schema.GetItemSchemas().front().GetItemSize() ==
              sizeof(slam::WPoint3D));
        auto pointcloud = ReadPointCloudFromPLY(stream, schema);
        auto &item_buffer = pointcloud->GetCollection().item<slam::WPoint3D>(0);
        std::vector<slam::WPoint3D> result;
        result.resize(pointcloud->size());
        std::copy(item_buffer.cbegin(), item_buffer.cend(), result.data());
        return result;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::WPoint3D>
    ReadWPoint3DVectorFromPLYFile(const std::string &file_path, const PLYSchemaMapper &schema) {
        std::ifstream ifile(file_path);
        return ReadWPoint3DVectorFromPLYStream(ifile, schema);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PointCloudPtr ReadPointCloudFromPLY(const std::string &file) {
        std::ifstream input_stream(file);
        return ReadPointCloudFromPLY(input_stream);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PointCloudPtr ReadPointCloudFromPLY(std::istream &input_stream) {
        PlyFileWrapper wrapper;
        wrapper.ReadStream(input_stream);
        auto mapper = PLYSchemaMapper::Builder::BuilderFromPLYFile(wrapper.file).Build();
        auto pc = wrapper.ReadPointCloud(mapper);
        pc->RegisterFieldsFromSchema();
        return pc;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void WritePLY(const std::string &output_file, const BufferCollection &collection, const PLYSchemaMapper &schema) {
        std::ofstream of(output_file);
        WritePLY(of, collection, schema);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void WritePLY(std::ostream &output_file, const BufferCollection &collection, const PLYSchemaMapper &schema) {
        WriteCollectionToPLY(output_file, collection, schema);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::ImuData> ReadIMUData(const std::string &file) {
        std::ifstream ifile(file);
        return ReadIMUData(ifile);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::ImuData> ReadIMUData(std::istream &ifile) {
        PlyFileWrapper wrapper;
        wrapper.ReadStream(ifile);
        long long state = ImuData::NONE;
        for (auto &element: wrapper.file.get_elements()) {
            auto &elem_name = element.name;

            if (elem_name == "linear_acceleration")
                state |= ImuData::LINEAR_ACCELERATION;
            if (elem_name == "angular_velocity")
                state |= ImuData::ANGULAR_VELOCITY;
            if (elem_name == "orientation")
                state |= ImuData::ORIENTATION;
            if (elem_name == "linear_acceleration_cov")
                state |= ImuData::LINEAR_ACCELERATION_COV;
            if (elem_name == "angular_velocity_cov")
                state |= ImuData::ANGULAR_VELOCITY_COV;
            if (elem_name == "orientation_cov")
                state |= ImuData::ORIENTATION_COV;
        }
        auto schema_mapper = slam::ImuData::GetSchemaMapper(state);
        slam::BufferCollection collection = wrapper.GetCollection(schema_mapper);

        std::vector<slam::ImuData> imu_data;
        imu_data.resize(collection.NumItemsPerBuffer());
        auto imu_data_view = collection.item<slam::ImuData>(0);
        std::copy(imu_data_view.begin(), imu_data_view.end(), imu_data.begin());
        for (auto &imu: imu_data)
            imu.state = state;
        return imu_data;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void WritePLY(const std::string &output_file, const std::vector<slam::ImuData> &data,
                  long long state) {
        std::ofstream of(output_file);
        WritePLY(of, data, state);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void WritePLY(std::ostream &of, const std::vector<slam::ImuData> &imu_records, long long state) {
        auto buffer_wrapper = BufferWrapper::CreatePtr(const_cast<std::vector<slam::ImuData> &>(imu_records),
                                                       slam::ImuData::GetSchema(state));
        slam::BufferCollection collection(std::move(buffer_wrapper));
        WritePLY(of, collection, slam::ImuData::GetSchemaMapper(state));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    const std::vector<slam::ItemSchema> &PLYSchemaMapper::GetItemSchemas() const {
        return schemas_;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    namespace {
        std::vector<PLYSchemaMapper::PLYProperty>
        DefaultPropertiesFromSchema(const std::vector<slam::ItemSchema> &item_schemas, bool remove_padding = false) {
            std::vector<PLYSchemaMapper::PLYProperty> properties;
            int item_id(0);
            std::set<std::string> element_names;
            for (auto &item: item_schemas) {
                for (auto &ply_element_name: item.GetElementNames()) {
                    if (element_names.find(ply_element_name) != element_names.end()) {
                        SLAM_LOG(WARNING) << "The element `" << ply_element_name << "` appears twice in the schema";
                        continue;
                    }

                    std::string ply_elem_pty_prefix = "";
                    auto &elem_info = item.GetElementInfo(ply_element_name);
                    for (auto &pty: elem_info.properties) {
                        if (remove_padding && pty.property_name.find("padding") != std::string::npos)
                            continue;
                        std::string ply_pty_prefix = ply_elem_pty_prefix + pty.property_name;
                        auto ptype = SlamPropertyToTinyPLYType(pty.type);
                        if (ptype == tinyply::Type::INVALID) {
                            std::stringstream ss_error;
                            ss_error << "Cannot convert slam property type to tinyply type (for property: "
                                     << pty.property_name << ")" << std::endl;
                            ss_error << "in schema:\n" << item << std::endl;
                            throw std::invalid_argument(ss_error.str());
                        }

                        for (int dim_id(0); dim_id < pty.dimension; dim_id++) {
                            auto ply_pty_name = pty.dimension == 1 ?
                                                ply_pty_prefix : ply_pty_prefix + "_" + std::to_string(dim_id);
                            properties.push_back({
                                                         ply_element_name,
                                                         ply_pty_name,
                                                         ptype,

                                                         item_id,
                                                         ply_element_name,
                                                         pty.property_name,
                                                         pty.type,
                                                         dim_id
                                                 });
                        }
                    }
                }

                item_id++;
            }
            return properties;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper PLYSchemaMapper::BuildDefaultFromSchema(const std::vector<slam::ItemSchema> &item_schemas,
                                                            bool ignore_incompatible_types,
                                                            bool remove_padding) {
        std::vector<PLYProperty> properties = DefaultPropertiesFromSchema(item_schemas,
                                                                          remove_padding);
        return PLYSchemaMapper(std::vector<slam::ItemSchema>(item_schemas),
                               std::move(properties));
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<std::string> PLYSchemaMapper::GetPLYElementNames() const {
        std::set<std::string> ply_element_names;
        std::vector<std::string> output;
        for (auto &elem: ply_properties_)
            ply_element_names.insert(elem.ply_element_name);
        output.insert(output.end(), ply_element_names.begin(), ply_element_names.end());
        return output;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    const slam::PropertyInfo &
    PLYSchemaMapper::GetSlamPropertyFromPLYProperty(const std::string &element_name,
                                                    const std::string &property_name,
                                                    int &dim) const {
        auto &pty = GetPLYProperty(element_name, property_name);
        dim = pty.property_dim;
        return schemas_[pty.item_id].GetElementInfo(pty.slam_element_name).GetProperty(pty.slam_property_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<std::string> PLYSchemaMapper::GetPLYProperties(const std::string &element_name) const {
        std::vector<std::string> output;
        for (auto &pty: ply_properties_) {
            if (pty.ply_element_name == element_name)
                output.push_back(pty.ply_property_name);
        }
        return output;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    const PLYSchemaMapper::PLYProperty &
    PLYSchemaMapper::GetPLYProperty(const std::string &element_name, const std::string &property_name) const {
        auto it = std::find_if(ply_properties_.cbegin(),
                               ply_properties_.cend(), [&element_name, &property_name](const auto &pty) {
                    return pty.ply_element_name == element_name && pty.ply_property_name == property_name;
                });
        CHECK(it != ply_properties_.cend()) << "The ply property " << property_name << " for element " << element_name
                                            << " does not exist in the schema" << std::endl;
        return *it;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::BufferCollection PLYSchemaMapper::AllocateBufferCollection(size_t num_items) const {
        std::vector<ItemBufferPtr> buffers;
        for (auto &item_schema: schemas_) {
            auto ptr = std::make_unique<VectorBuffer>(slam::ItemSchema(item_schema),
                                                      item_schema.GetItemSize());
            ptr->Resize(num_items);
            buffers.emplace_back(std::move(ptr));
        }
        return slam::BufferCollection(std::move(buffers));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper PLYSchemaMapper::BuildDefaultFromBufferCollection(const BufferCollection &collection,
                                                                      bool remove_padding) {
        std::vector<slam::ItemSchema> schemas;
        for (auto item_id(0); item_id < collection.NumItemsInSchema(); item_id++) {
            schemas.push_back(collection.GetItemInfo(item_id).item_schema);
        };
        return BuildDefaultFromSchema(schemas, true, remove_padding);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper::Builder PLYSchemaMapper::Builder::BuilderFromItemSchema(ItemSchema &&schema, bool default_ply) {
        PLYSchemaMapper::Builder builder;
        builder.schemas_ = {std::move(schema)};
        if (default_ply)
            builder.ply_properties_ = DefaultPropertiesFromSchema(builder.schemas_);
        return builder;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper::Builder
    PLYSchemaMapper::Builder::BuilderFromBufferCollection(const BufferCollection &collection, bool default_ply) {
        PLYSchemaMapper::Builder builder;
        builder.schemas_.reserve(collection.NumItemsInSchema());
        for (auto i(0); i < collection.NumItemsInSchema(); i++)
            builder.schemas_.emplace_back(collection.GetItemInfo(i).item_schema);
        if (default_ply)
            builder.ply_properties_ = DefaultPropertiesFromSchema(builder.schemas_);
        return builder;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper::Builder &PLYSchemaMapper::Builder::AddItemSchema(ItemSchema &&schema) {
        schemas_.emplace_back(schema);
        return *this;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper::Builder &PLYSchemaMapper::Builder::SetXYZElementName(const std::string &element_name) {
        bool is_valid = false;
        for (auto &schema: schemas_) {
            if (schema.HasElement(element_name)) {
                CHECK(IsValidXYZElement(schema.GetElementInfo(element_name)))
                                << "The XYZElement `" << element_name
                                << "` selected is not a valid point element in the schema: \n" << schema;
                is_valid = true;
            }
        }
        CHECK(is_valid) << "Could not find the element with name `" << element_name << "` in any of the schemas";
        xyz_name_ = element_name;
        return *this;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper::PLYProperty &
    PLYSchemaMapper::Builder::AddPLYProperty(std::string &&ply_element_name, std::string &&ply_property_name,
                                             tinyply::Type type) {
        // Verify that the property does not already exists
        for (auto &pty: ply_properties_) {
            CHECK(pty.ply_element_name != ply_element_name || pty.ply_property_name != ply_property_name)
                            << "There can only be one pair of PLY Element, PLY Property in the mapping";
        }

        ply_properties_.push_back({ply_element_name,
                                   ply_property_name,
                                   type
                                  });
        return ply_properties_.back();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper::PLYProperty &
    PLYSchemaMapper::Builder::GetPLYProperty(const std::string &element_name, const std::string &property_name) {
        for (auto &pty: ply_properties_) {
            if (pty.ply_element_name == element_name && pty.ply_property_name == property_name)
                return pty;
        }
        throw std::runtime_error("Could not find the property `" + property_name +
                                 "` of element `" + element_name + "` in the PLYSchemaMapper");
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PLYSchemaMapper::Builder::CheckIsValid() { CHECK(IsValid()) << "The Builder is in an invalid state"; }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool PLYSchemaMapper::Builder::IsValid() {
        for (auto &pty: ply_properties_) {
            if (pty.ply_element_name.empty() || pty.ply_property_name.empty())
                return false;
            bool has_property = false;
            auto it = std::find_if(schemas_.begin(), schemas_.end(), [&](ItemSchema &schema) {
                return schema.HasElement(pty.slam_element_name) &&
                       schema.HasProperty(pty.slam_element_name, pty.slam_property_name);
            });

            // Check that at least one schema has the given element
            if (it != schemas_.end()) {
                auto &itemschema = *it;
                auto &elem_info = itemschema.GetElementInfo(pty.slam_element_name);

                // Check that the element has the slam property
                if (!elem_info.HasProperty(pty.slam_property_name))
                    return false;
                auto &pty_info = elem_info.GetProperty(pty.slam_property_name);

                // Check that the dimension of the PTY info is larger than the dimension index of the PLYProperty
                if (pty_info.dimension <= pty.property_dim)
                    return false;
                has_property = true;
            }

            if (!has_property)
                return false;
        }
        return true;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper PLYSchemaMapper::Builder::Build() {
        CheckIsValid();
        auto mapper = PLYSchemaMapper(std::vector<ItemSchema>(schemas_), std::vector<PLYProperty>(ply_properties_));
        mapper.xyz_element_name_ = xyz_name_;
        return mapper;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper::Builder &PLYSchemaMapper::Builder::AddPLYProperty(PLYSchemaMapper::PLYProperty &&new_property) {
        ply_properties_.push_back(new_property);
        return *this;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    PLYSchemaMapper::Builder PLYSchemaMapper::Builder::BuilderFromPLYFile(const tinyply::PlyFile &file,
                                                                          bool default_item) {

        bool is_set_xyz_element = false;
        std::string xyz_element_name;

        // Sets the elements
        PLYSchemaMapper::Builder builder;
        auto elements = file.get_elements();
        int item_id = 0;
        for (auto &element: elements) {
            ItemSchema::Builder item_builder;
            item_builder.AddElement(element.name, 0);

            int offset_in_elem = 0;
            int offset_x = -1, offset_y = -1, offset_z = -1;
            PROPERTY_TYPE x_pty;
            for (auto &property: element.properties) {
                auto slam_type = slam::TinyPLYToSlamPropertyType(property.propertyType);
                item_builder.AddProperty(std::string(element.name),
                                         std::string(property.name),
                                         slam::TinyPLYToSlamPropertyType(property.propertyType),
                                         offset_in_elem,
                                         1);
                auto &pty = builder.AddPLYProperty(
                        std::string(element.name),
                        std::string(property.name),
                        property.propertyType);

                pty.item_id = item_id;
                pty.slam_element_name = element.name;
                pty.slam_property_name = property.name;
                pty.slam_pty_type = slam_type;
                pty.property_dim = 0;

                std::string pty_name = property.name;
                std::transform(property.name.begin(), property.name.end(), pty_name.begin(),
                               [](unsigned char c) { return std::tolower(c); });
                if (pty_name == "x") {
                    offset_x = offset_in_elem;
                    x_pty = slam_type;
                }
                if (pty_name == "y")
                    offset_y = offset_in_elem;
                if (pty_name == "z")
                    offset_z = offset_in_elem;
                offset_in_elem += PropertySize(slam_type);
            }

            auto item_size = offset_in_elem;
            item_builder.SetItemSize(item_size);
            item_id++;

            if (!is_set_xyz_element) {
                if (element.name == "vertex") {
                    if (element.properties.size() == 3 || element.properties.size() == 1) {
                        xyz_element_name = element.name;
                        is_set_xyz_element = true;
                    }
                }

                if (!is_set_xyz_element && offset_x >= 0 && offset_y >= 0 && offset_z >= 0) {
                    if (offset_x < offset_y && offset_y < offset_z) {
                        if ((offset_y - offset_x) == (offset_z - offset_y) &&
                            ((offset_z - offset_y) == sizeof(float) || (offset_z - offset_y) == sizeof(double))) {
                            // If the element already has a size of 3, no need to create a new element
                            if (element.properties.size() == 3) {
                                xyz_element_name = element.name;
                                is_set_xyz_element = true;
                            } else {
                                // Add a new element in the item schema
                                item_builder.AddElement("xyz", offset_x);
                                item_builder.AddProperty("xyz", "x", x_pty, 0, 1);
                                item_builder.AddProperty("xyz", "y", x_pty, offset_y - offset_x, 1);
                                item_builder.AddProperty("xyz", "z", x_pty, offset_z - offset_x, 1);
                                xyz_element_name = "xyz";
                                is_set_xyz_element = true;
                            }
                        }
                    }
                }
            }

            builder.AddItemSchema(item_builder.Build());
        }

        if (is_set_xyz_element)
            builder.SetXYZElementName(xyz_element_name);


        return builder;
    }
    /* -------------------------------------------------------------------------------------------------------------- */


} // namespace
