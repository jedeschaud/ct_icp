#include <ct_icp-viz3d/dataset_window.h>

namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    DatasetVTKWindow::DatasetVTKWindow(std::string &&winname)
            : MultiPolyDataWindow() {
        window_ = std::make_shared<DatasetVTKWindow::_DatasetWindow>(std::move(winname));
        auto window = std::dynamic_pointer_cast<_DatasetWindow>(window_);
        SLAM_CHECK_STREAM(window, "The Window is not in a valid state");
        // Not a good design : the reactor will be called from the notifier
        // They should be encapsulated together ...
        window->reactor.frame_notifier.AddObserverLambda([&window, this](FramePtr frame) {
            if (!frame)
                return;
            auto poly_data = slam::polydata_from_pointcloud(*frame->pointcloud);
            window->last_file_path = frame->file_path;
            this->AddPolyData("Frame", 0, poly_data);
        });
        window->reactor.end_of_dataset_notifier.AddObserverLambda([&window](const std::string &message) {
            window->scheduler.Abort();
        });
        window->scheduler.AddObserverLambda([&window](double) {
            window->reactor.React(dataset_message_t{NEXT});
        });
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void DatasetVTKWindow::_DatasetWindow::DrawImGUIContent() {
        form.Draw();

        // Add Actions
        if (viz3d::ImGui_HorizontalButton("Load")) {
            ct_icp::DATASET dataset = ct_icp::DATASETFromString(form.dataset_name.value);
            if (dataset == INVALID) {
                SLAM_LOG(ERROR) << "Invalid Dataset passed as arguments: " << form.dataset_name.value << std::endl;
            } else {
                reactor.React(dataset_message_t{
                        LOAD,
                        std::make_shared<dataset_loading_params>(
                                ct_icp::DatasetOptions{dataset, form.root_path.value},
                                form.sequence_name.value,
                                form.frame_idx.value)
                });
                scheduler.Stop();
                scheduler.SetFrequency(form.frequency.value);
            }
        }

        if (!scheduler.IsRunning()) {
            if (viz3d::ImGui_HorizontalButton("Next")) {
                reactor.React(dataset_message_t{NEXT});
            }
            if (viz3d::ImGui_HorizontalButton("Start"))
                scheduler.Start();

        } else {
            if (viz3d::ImGui_HorizontalButton("Stop"))
                scheduler.Stop();
        }

        if (!last_file_path.empty())
            ImGui::Text("%s", last_file_path.c_str());

        viz3d::VTKWindow::DrawImGUIContent();
    }
} // namespace ct_icp