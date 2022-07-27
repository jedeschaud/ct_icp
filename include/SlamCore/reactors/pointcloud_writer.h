#ifndef SLAMCORE_POINTCLOUD_WRITER_H
#define SLAMCORE_POINTCLOUD_WRITER_H

#include "SlamCore/reactors/reactor.h"
#include "SlamCore/io.h"

namespace slam {

    // An Abstract Message for a PointCloudWriter
    struct APCWriterMessageContent {

        // Types of message expected by the PointCloudWriter actor
        enum MessageType {
            SET_DEFAULT_DIRECTORY,
            SET_SCHEMA_MAPPER,
            WRITE_FRAME
        };

        virtual ~APCWriterMessageContent() = 0;

        // Returns the message type
        virtual MessageType GetMessageType() const = 0;
    };

    typedef std::shared_ptr<APCWriterMessageContent> pcwriter_message_t;

    // specialize message_tag to validate the pcwriter_message_t
    template<>
    struct message_tag<class PointCloudWriter, pcwriter_message_t> {
        static constexpr bool is_valid = true;
    };

    // A Message content containing the new directory path
    struct SetDefaultDirectoryMC : public APCWriterMessageContent {

        ~SetDefaultDirectoryMC() = default;

        MessageType GetMessageType() const { return SET_DEFAULT_DIRECTORY; }

        SetDefaultDirectoryMC(std::string &&directory) : new_directory_path(std::move(directory)) {}

        SetDefaultDirectoryMC(const std::string &directory) : new_directory_path(directory) {}

        const std::string new_directory_path;
    };

    // A Message content containing the new Schema passed to the PointCloudWriter
    struct SetSchemaMapperMC : public APCWriterMessageContent {

        ~SetSchemaMapperMC() = default;

        MessageType GetMessageType() const { return SET_SCHEMA_MAPPER; }

        SetSchemaMapperMC(PLYSchemaMapper &&mapper) : new_schema_mapper(std::move(mapper)) {}

        SetSchemaMapperMC(const PLYSchemaMapper &mapper) : new_schema_mapper(mapper) {}

        std::optional<PLYSchemaMapper> new_schema_mapper;

    };

    // A Message content of a frame to write on disk
    struct WriteFrameMC : public APCWriterMessageContent {

        ~WriteFrameMC() = default;

        MessageType GetMessageType() const { return WRITE_FRAME; }

        WriteFrameMC(slam::PointCloudPtr data_,
                     std::string filename_,
                     std::optional<std::string> directory_path_) :
                data(data_), filename(filename_), directory_path(directory_path_) {}

        WriteFrameMC() = default;

        slam::PointCloudPtr data;
        std::string filename;
        std::optional<std::string> directory_path;

    };

    class PointCloudWriter : public GenericReactor<PointCloudWriter> {
    public:

        PointCloudWriter(slam::blocking_queue<std::string> *log_message_queue = nullptr) :
                log_message_queue_(log_message_queue) {};

        void DoReact(pcwriter_message_t message, message_tag<PointCloudWriter, pcwriter_message_t>);

    private:
        slam::blocking_queue<std::string> *log_message_queue_ = nullptr;
        std::optional<PLYSchemaMapper> mapper = {};
        std::string directory_path = "/tmp/";

        void _SetDirectory(const std::string &directory);

        void _WriteFrame(const WriteFrameMC &frame);

        void _SetSchemaMapper(const SetSchemaMapperMC &mc);
    };


} // namespace slam

#endif //SLAMCORE_POINTCLOUD_WRITER_H
