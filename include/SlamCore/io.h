#ifndef SlamCore_IO_H
#define SlamCore_IO_H

#include "SlamCore/types.h"
#include "SlamCore/pointcloud.h"
#include "SlamCore/imu.h"

#include <tinyply/tinyply.h>

namespace slam {

    // Returns the number of bytes for a given tinyply::type
    inline int SizeOfTinyPLYType(tinyply::Type type) {
        switch (type) {
            case tinyply::Type::FLOAT64:
                return sizeof(double);
            case tinyply::Type::FLOAT32:
                return sizeof(float);
            case tinyply::Type::INT8:
            case tinyply::Type::UINT8:
                return sizeof(std::int8_t);
            case tinyply::Type::INT16:
            case tinyply::Type::UINT16:
                return sizeof(std::int16_t);
            case tinyply::Type::INT32:
            case tinyply::Type::UINT32:
                return sizeof(std::int32_t);
            default:
                throw std::runtime_error("Invalid Type");
        }
    }

    // A utility class to convert an array of ItemSchema[] to a PLY Schema
    class PLYSchemaMapper {
    public:

        // A PLY Property mapped to a Property of an Element of an Item Schema
        struct PLYProperty {
            std::string ply_element_name;
            std::string ply_property_name;
            tinyply::Type pty_type;

            int item_id = -1;
            std::string slam_element_name, slam_property_name;
            slam::PROPERTY_TYPE slam_pty_type;
            int property_dim = -1;
        };

        // A Builder to construct PLYSchemaMapper from a set of ItemSchema, or from a PLYFile
        class Builder {
        public:

            /*!
             * @brief  Returns a Builder initialized from an ItemSchema
             *
             * @param default_ply whether the builder should initialize PLY properties
             */
            static Builder BuilderFromItemSchema(slam::ItemSchema &&schema, bool default_ply = false);

            /*!
             * @brief  Returns a Builder initialized from a Buffer Collection
             *
             * @param default_ply whether the builder should initialize PLY properties
             */
            static Builder BuilderFromBufferCollection(const slam::BufferCollection &, bool default_ply = false);

            /*!
             * @brief  Returns a Builder initialized from a PLYFile (after the header has been parsed)
             *
             * @param default_ply whether the builder should initialize PLY properties
             */
            static Builder BuilderFromPLYFile(const tinyply::PlyFile &, bool default_item = false);

            // Adds an Item Schema to the Builder
            Builder &AddItemSchema(slam::ItemSchema &&schema);

            // Sets the XYZ element name (must match a valid element for a given item in the schema)
            Builder &SetXYZElementName(const std::string &element_name);

            // Adds a new PLY property to the schema
            Builder &AddPLYProperty(PLYProperty &&new_property);

            // Adds a PLY property to the schema
            PLYProperty &AddPLYProperty(
                    std::string &&ply_element_name,
                    std::string &&ply_property_name,
                    tinyply::Type type);

            // Returns a PLY property
            PLYProperty &GetPLYProperty(const std::string &element_name,
                                        const std::string &property_name);;

            // Returns true if all PLY properties match a property of the specified item
            bool IsValid();

            // Throws an exception if the Builder is not in a valid state
            void CheckIsValid();

            // Builds the PLYSchemaMapper
            PLYSchemaMapper Build();

        private:
            std::string xyz_name_;
            std::vector<slam::ItemSchema> schemas_;
            std::vector<PLYProperty> ply_properties_;
        };

        // Creates a buffer collection for the items corresponding to the ItemSchema of this Schema Mapper
        slam::BufferCollection AllocateBufferCollection(size_t num_items = 0) const;

        // Returns the slam::ItemSchema's associated with the mapper
        const std::vector<slam::ItemSchema> &GetItemSchemas() const;

        /**
         * Builds a schema mapper from a Buffer Collection
         * @param remove_padding Removes all properties containing the string `padding` in their name
         */
        static PLYSchemaMapper BuildDefaultFromBufferCollection(const BufferCollection &collection,
                                                                bool remove_padding = true);

        // Builds a mapper from a vector of Item Schema
        static PLYSchemaMapper BuildDefaultFromSchema(const std::vector<slam::ItemSchema> &item_schemas,
                                                      bool ignore_incompatible_types = false,
                                                      bool remove_padding = true);

        // Returns the PLY element names
        std::vector<std::string> GetPLYElementNames() const;

        // Returns the properties names for a given element
        std::vector<std::string> GetPLYProperties(const std::string &element_name) const;

        // Returns the PropertyInfo associated with a PLY property with the dimension of the corresponding PLY property
        const slam::PropertyInfo &GetSlamPropertyFromPLYProperty(const std::string &element_name,
                                                                 const std::string &property_name,
                                                                 int &dim) const;

        // Returns a PLY property from a
        const PLYProperty &GetPLYProperty(const std::string &element_name,
                                          const std::string &property_name) const;

        // The XYZ Element name to define
        REF_GETTER(XYZElementName, xyz_element_name_);

    private:
        PLYSchemaMapper(std::vector<slam::ItemSchema> &&schemas,
                        std::vector<PLYProperty> &&properties) :
                schemas_(std::move(schemas)),
                ply_properties_(std::move(properties)) {};

        friend class Builder;

        std::vector<slam::ItemSchema> schemas_;
        std::vector<PLYProperty> ply_properties_;
        std::string xyz_element_name_;
    };

    // Converts a slam::PROPERTY_TYPE to a tinyply::Type (returns INVALID_TYPE if no conversion can be done)
    tinyply::Type SlamPropertyToTinyPLYType(slam::PROPERTY_TYPE type);

    // Converts a tinyply::Type from a slam::PROPERTY_TYPE
    slam::PROPERTY_TYPE TinyPLYToSlamPropertyType(tinyply::Type type);

    std::vector<uint8_t> ReadStreamAsByteArray(std::istream &stream);

    // Reads a PLY file from a Stream
    std::vector<slam::WPoint3D> ReadWPoint3DVectorFromPLYStream(std::istream &stream,
                                                                const PLYSchemaMapper &schema = slam::WPoint3D::FloatSchemaMapper());

    // Reads a PLY file from disk
    std::vector<slam::WPoint3D>
    ReadWPoint3DVectorFromPLYFile(const std::string &file_path, const PLYSchemaMapper &schema);

    // Writes a point cloud to a file
    void WritePLY(std::ostream &output_file, const std::vector<slam::WPoint3D> &points);

    // Writes a point cloud to a file
    void WritePLY(const std::string &file, const std::vector<slam::WPoint3D> &points);

    // Writes a point cloud to a file
    void WritePLY(std::ostream &output_file, const PointCloud &pointcloud, const PLYSchemaMapper &schema);;

    // Writes a point cloud to a file
    void WritePLY(const std::string &file, const PointCloud &points, const PLYSchemaMapper &schema);;

    // Writes a buffer collection to a file
    void WritePLY(const std::string &output_file, const BufferCollection &collection, const PLYSchemaMapper &schema);

    // Writes a vector of ImuData collection to a file
    void WritePLY(const std::string &output_file, const std::vector<slam::ImuData> &data,
                  long long state = slam::ImuData::ALL);

    // Writes a vector of ImuData collection to a stram
    void WritePLY(std::ostream &output_file, const std::vector<slam::ImuData> &data,
                  long long state = slam::ImuData::ALL);

    // Writes a buffer collection to a file
    void WritePLY(std::ostream &output_file, const BufferCollection &collection, const PLYSchemaMapper &schema);

    // Reads a point cloud from a file
    PointCloudPtr ReadPointCloudFromPLY(std::istream &input_file, const PLYSchemaMapper &schema);

    // Reads a point cloud from a file
    PointCloudPtr ReadPointCloudFromPLY(const std::string &file, const PLYSchemaMapper &schema);

    // Reads a point cloud from a file
    PointCloudPtr ReadPointCloudFromPLY(const std::string &file);

    // Reads a ImuData from a file
    std::vector<slam::ImuData> ReadIMUData(const std::string &file);

    // Reads a ImuData from a file
    std::vector<slam::ImuData> ReadIMUData(std::istream &ifile);

    // Reads a point cloud from a stream
    PointCloudPtr ReadPointCloudFromPLY(std::istream &input_stream);

    // Writes the vector of poses as a PLY binary file stream
    void SavePosesAsPLY(std::ostream &output_file,
                        const std::vector<Pose> &poses);

    // Writes the vector of poses as a binary PLY file on disk
    void SavePosesAsPLY(const std::string &file_path,
                        const std::vector<Pose> &poses);

    // Reads a vector of poses from a PLY binary file stream
    std::vector<slam::Pose> ReadPosesFromPLY(std::istream &input_file);

    // Reads a vector of poses from a PLY binary file on disk
    std::vector<slam::Pose> ReadPosesFromPLY(const std::string &input_file_path);

    // Saves Poses to disk, and returns whether the writing was successful
    bool SavePosesKITTIFormat(const std::string &file_path, const std::vector<Pose> &);

    // Loads Poses from disk. Raises a std::runtime_error if it fails to do so
    std::vector<Pose> LoadPosesKITTIFormat(const std::string &file_path);

} // namespace slam

#endif //SlamCore_IO_H
