#include "ct_icp/reactors/registration.h"

namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    void RegistrationReactor::DoReact(const ct_icp_message_t &message,
                                      slam::message_tag<RegistrationReactor, ct_icp_message_t>) {
        switch (message.action) {
            case registration::CHANGE_PARAMS:
                if (!message.options_ptr) {
                    logger.Log(Logger::WARNING, "The message contains an empty ct_icp::OdometryOptions pointer");
                    return;
                }
                last_inserted_frame_id = 0;
                icp_node.Options() = *message.options_ptr;
                break;
            case registration::REGISTER_FRAME: {
                TrajectoryFrame initial_estimate;
                initial_estimate.begin_pose.dest_frame_id = 0;
                initial_estimate.begin_pose.dest_timestamp = 0;
                initial_estimate.end_pose.dest_frame_id = 0;
                initial_estimate.end_pose.dest_timestamp = 0;

                if (message.registration_message->initial_estimate)
                    initial_estimate = *(message.registration_message->initial_estimate);
                TrajectoryFrame previous_estimate = initial_estimate;

                if (message.registration_message->pointcloud_ptr->HasTimestamps()) {
                    auto timestamps = message.registration_message->pointcloud_ptr->TimestampsProxy<double>();
                    auto size = timestamps.size();

                    double _min = std::numeric_limits<double>::max(), _max = std::numeric_limits<double>::min();
                    for (auto i(0); i < timestamps.size(); ++i) {
                        double timestamp = timestamps[i];
                        if (timestamp < _min)
                            _min = timestamp;
                        if (timestamp > _max)
                            _max = timestamp;
                    }
                    auto [min, max] = std::minmax_element(timestamps.begin(), timestamps.end());
                    initial_estimate.begin_pose.dest_timestamp = *min;
                    initial_estimate.end_pose.dest_timestamp = *max;
                }

                auto summary = icp_node.Register(*(message.registration_message->map),
                                                 *(message.registration_message->pointcloud_ptr),
                                                 initial_estimate, message.motion_model.get());
                SLAM_LOG(INFO) << "[CT-ICP] Finished registration of frame" << std::endl;
                registration_output_t output;
                output.summary = summary;
                output.frame = initial_estimate;
                summary_notifier.Notify(output);
            }
                break;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ct_icp_message_t ct_icp_message_t::NewFrameMessage(slam::PointCloudPtr pointcloud,
                                                       std::shared_ptr<ct_icp::ISlamMap> map,
                                                       std::optional<ct_icp::TrajectoryFrame> initial_estimate) {
        ct_icp_message_t message{registration::REGISTER_FRAME};

        message.registration_message = registration_message_t{};
        message.registration_message->pointcloud_ptr = pointcloud;
        message.registration_message->initial_estimate = initial_estimate;
        message.registration_message->map = map;

        return message;
    }
}