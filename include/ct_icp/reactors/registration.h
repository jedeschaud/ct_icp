#ifndef CT_ICP_REGISTRATION_H
#define CT_ICP_REGISTRATION_H

#include <SlamCore/reactors/reactor.h>
#include <SlamCore/reactors/notifier.h>

#include "ct_icp/ct_icp.h"
#include "ct_icp/reactors/logger.h"

namespace ct_icp {
    struct ct_icp_message_t;
    class RegistrationReactor;
}

SLAM_REGISTER_MESSAGE(ct_icp::RegistrationReactor, ct_icp::ct_icp_message_t)

namespace ct_icp {

    namespace registration {
        enum REGISTRATION_ACTION {
            CHANGE_PARAMS,
            REGISTER_FRAME,
        };
    }

    struct registration_message_t {
        slam::PointCloudPtr pointcloud_ptr = nullptr;
        std::optional<ct_icp::TrajectoryFrame> initial_estimate = {};
        std::shared_ptr<ct_icp::ISlamMap> map = nullptr;

        registration_message_t() = default;

        explicit registration_message_t(slam::PointCloudPtr pc_ptr) : pointcloud_ptr(pc_ptr) {}
    };

    struct ct_icp_message_t {
        registration::REGISTRATION_ACTION action;
        std::shared_ptr<ct_icp::CTICPOptions> options_ptr = nullptr;
        std::optional<registration_message_t> registration_message = {};
        std::shared_ptr<AMotionModel> motion_model = nullptr;

        explicit ct_icp_message_t(registration::REGISTRATION_ACTION action) : action(action) {}

        static ct_icp_message_t NewFrameMessage(slam::PointCloudPtr pointcloud,
                                                std::shared_ptr<ct_icp::ISlamMap> map,
                                                std::optional<ct_icp::TrajectoryFrame> initial_estimate = {});
    };

    struct registration_output_t {
        ct_icp::ICPSummary summary;
        TrajectoryFrame frame;
    };


    /** @brief RegistrationReactor is a reactor which performs a registration of a point cloud against a map
     */
    class RegistrationReactor : public slam::GenericReactor<RegistrationReactor> {
    public:

        RegistrationReactor() {}

        RegistrationReactor(const RegistrationReactor &other) : RegistrationReactor() {
            icp_node.Options() = other.icp_node.Options();
            summary_notifier = other.summary_notifier;
        }

        void DoReact(const ct_icp_message_t &message,
                     slam::message_tag<RegistrationReactor, ct_icp_message_t>);

        Logger logger;
        slam::Notifier<registration_output_t> summary_notifier;
    private:
        slam::frame_id_t last_inserted_frame_id = 0;
        ct_icp::CT_ICP_Registration icp_node;
        friend class RegistrationWindow;
    };
}
#endif //CT_ICP_REGISTRATION_H
