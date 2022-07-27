#include "ct_icp/reactors/dataset_loader.h"

namespace ct_icp {


    /* -------------------------------------------------------------------------------------------------------------- */
    void CTICPDatasetReactor::DoReact(const dataset_message_t &message,
                                      slam::message_tag<CTICPDatasetReactor, dataset_message_t>) {

        switch (message.command) {
            case NEXT: {
                if (!sequence) {
                    logger.Log(Logger::ERROR, "The sequence is not defined");
                    if (end_of_dataset_notifier.HasObservers())
                        end_of_dataset_notifier.Notify("The sequence is not defined");
                    return;
                }
                if (!frame_notifier.HasObservers()) {
                    logger.Log(Logger::WARNING, "No observers to the frame notifier");
                    return;
                }
                if (!sequence->HasNext()) {
                    logger.Log(Logger::WARNING, "The dataset has no more frames !");
                    if (end_of_dataset_notifier.HasObservers())
                        end_of_dataset_notifier.Notify("The dataset has no more frames !");
                    return;
                }
                auto next_frame = std::make_shared<ct_icp::ADatasetSequence::Frame>();
                (*next_frame) = std::move(sequence->NextFrame());
                frame_notifier.Notify(next_frame);
            }
                break;
            case LOAD:
                if (!message.definition_ptr) {
                    logger.Log(Logger::ERROR,
                               "The message does not contain a valid message. Cannot load the dataset");
                    return;
                }
                if (!fs::exists(message.definition_ptr->dataset_options.root_path)) {
                    logger.Log(Logger::ERROR,
                               "The root path does not exist on disk: " +
                               message.definition_ptr->dataset_options.root_path);
                    return;
                }

                sequence = nullptr;
                {
                    auto dataset = ct_icp::Dataset::LoadDataset(message.definition_ptr->dataset_options);
                    logger.Log(Logger::INFO, "Loaded dataset from root path: " +
                                             message.definition_ptr->dataset_options.root_path);
                    if (message.definition_ptr->dataset_options.dataset != PLY_DIRECTORY) {
                        if (!dataset.HasSequence(message.definition_ptr->sequence_name)) {
                            logger.Log(Logger::ERROR, "The dataset does not contain sequence: " +
                                                      message.definition_ptr->sequence_name);
                            return;
                        }
                        sequence = dataset.GetSequence(message.definition_ptr->sequence_name);
                    } else {
                        sequence = PLYDirectory::PtrFromDirectoryPath(
                                message.definition_ptr->dataset_options.root_path);
                    }
                    sequence->SetInitFrame(message.definition_ptr->frame_idx);
                }
                break;
        }
    }
} // namespace ct_icp