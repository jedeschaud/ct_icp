#ifndef CT_ICP_DATASET_LOADER_H
#define CT_ICP_DATASET_LOADER_H

#include <SlamCore/reactors/reactor.h>

#include "ct_icp/dataset.h"
#include "ct_icp/reactors/logger.h"

namespace ct_icp {
    struct dataset_message_t;
    struct CTICPDatasetReactor;
} // namespace ct_icp

SLAM_REGISTER_MESSAGE(ct_icp::CTICPDatasetReactor, ct_icp::dataset_message_t)

namespace ct_icp {

    enum COMMAND {
        NEXT,                   // Sends the next frame
        LOAD,                   // (Re)Loads the dataset
//        CURRENT,                // Sends the current frame
//        CHANGE_PARAMETERS       // Change the parameters
    };

    struct dataset_loading_params {
        ct_icp::DatasetOptions dataset_options;
        std::string sequence_name;
        int frame_idx = 0; //< The initial frame index to set (when loading the dataset)

        dataset_loading_params(ct_icp::DatasetOptions &&options, std::string seq_name, int frame_idx = 0) :
                dataset_options(std::move(options)),
                sequence_name(std::move(seq_name)),
                frame_idx(frame_idx) {}
    };

    /** @brief Message expected by the CTICPDatasetReactor */
    struct dataset_message_t {
        COMMAND command = NEXT;
        std::shared_ptr<dataset_loading_params> definition_ptr = nullptr;
    };

    typedef std::shared_ptr<ct_icp::ADatasetSequence::Frame> FramePtr;

    /**
     * @brief  A Reactor to load a dataset and broadcast frames to subscribed observers
     */
    class CTICPDatasetReactor : public slam::GenericReactor<CTICPDatasetReactor> {
    public:

        CTICPDatasetReactor() = default;

        void DoReact(const dataset_message_t &message,
                     slam::message_tag<CTICPDatasetReactor, dataset_message_t>);

        slam::Notifier<std::string> end_of_dataset_notifier;
        slam::Notifier<FramePtr> frame_notifier;
        Logger logger;
    private:
        std::shared_ptr<ct_icp::ADatasetSequence> sequence = nullptr;
    };

} // namespace ct_icp

#endif //CT_ICP_DATASET_LOADER_H
