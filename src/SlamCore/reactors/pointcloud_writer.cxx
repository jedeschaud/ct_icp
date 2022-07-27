#include "SlamCore/reactors/pointcloud_writer.h"

namespace slam {

    /* -------------------------------------------------------------------------------------------------------------- */
    APCWriterMessageContent::~APCWriterMessageContent() {}

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloudWriter::DoReact(pcwriter_message_t message, message_tag<PointCloudWriter, pcwriter_message_t>) {
        if (!message) {
            LOG(WARNING) << "Received `nullptr` message" << std::endl;
            return;
        }
        switch (message->GetMessageType()) {
            case APCWriterMessageContent::SET_DEFAULT_DIRECTORY: {
                auto typed_message = std::reinterpret_pointer_cast<SetDefaultDirectoryMC>(message);
                if (!typed_message)
                    return;
                _SetDirectory(typed_message->new_directory_path);
            }
                break;
            case APCWriterMessageContent::WRITE_FRAME: {
                auto typed_message = std::reinterpret_pointer_cast<WriteFrameMC>(message);
                if (!typed_message)
                    return;
                _WriteFrame(*typed_message);
            }
                break;
            case APCWriterMessageContent::SET_SCHEMA_MAPPER: {
                auto typed_message = std::reinterpret_pointer_cast<SetSchemaMapperMC>(message);
                if (!typed_message)
                    return;
                _SetSchemaMapper(*typed_message);
            }
                break;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloudWriter::_SetDirectory(const std::string &directory) { directory_path = directory; }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloudWriter::_SetSchemaMapper(const SetSchemaMapperMC &mc) { mapper = mc.new_schema_mapper; }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloudWriter::_WriteFrame(const WriteFrameMC &frame) {
        if (!frame.data) {
            if (log_message_queue_) {
                log_message_queue_->push("[ERROR] the pc received is a nullptr");
            }
            return;
        }

        if (frame.filename.empty()) {
            if (log_message_queue_) {
                log_message_queue_->push("[ERROR] The filename is empty");
            }
            return;
        }

        fs::path dir_path =
                frame.directory_path.has_value() && !frame.directory_path->empty() ? fs::path(*frame.directory_path) :
                fs::path(directory_path);

        // Try and create the output directory
        if (!fs::exists(dir_path)) {
            try {
                fs::create_directories(dir_path);
            } catch (...) {
                std::stringstream error_msg;
                error_msg << "[ERROR] Could not create directory " << dir_path.string()
                          << " on disk. Saving aborted.";

                if (log_message_queue_) {
                    log_message_queue_->emplace(error_msg.str());
                } else
                    std::cerr << error_msg.str() << std::endl;
                return;
            }
        }

        auto schema_mapper = mapper.has_value() ? *mapper :
                             slam::PLYSchemaMapper::BuildDefaultFromBufferCollection(frame.data->GetCollection());
        slam::WritePLY((dir_path / frame.filename).string(), *frame.data, schema_mapper);
    }

} // namespace slam