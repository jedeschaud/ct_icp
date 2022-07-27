#ifndef CT_ICP_DATASET_WINDOW_H
#define CT_ICP_DATASET_WINDOW_H

#include <SlamCore-viz3d/viz3d_windows.h>
#include <SlamCore/reactors/scheduler.h>

#include "ct_icp/reactors/dataset_loader.h"

namespace ct_icp {

    /** @brief A DatasetVTKWindow is a window to iterate over a dataset (either manually) or using a Scheduler */
    struct DatasetVTKWindow : public slam::MultiPolyDataWindow {

        explicit DatasetVTKWindow(std::string &&winname);

        ~DatasetVTKWindow() {}

        slam::Notifier<FramePtr> &GetFrameNotifier() {
            auto window = std::dynamic_pointer_cast<_DatasetWindow>(window_);
            SLAM_CHECK_STREAM(window, "The Window is not in a valid state");
            return window->reactor.frame_notifier;
        }

        slam::Notifier<std::string> &GetEndOfDatasetNotifier() {
            auto window = std::dynamic_pointer_cast<_DatasetWindow>(window_);
            SLAM_CHECK_STREAM(window, "The Window is not in a valid state");
            return window->reactor.end_of_dataset_notifier;
        }

    protected:

        struct _DatasetWindow : slam::MultiPolyDataWindow::ChildVTKWindow {

            _DatasetWindow(std::string &&winname) : slam::MultiPolyDataWindow::ChildVTKWindow(std::move(winname)),
                                                    form(window_name_ + "_form_id", window_name_ + " Form") {}

            void DrawImGUIContent() override;

            vtkSmartPointer<vtkActor> previous_actor = nullptr;
            CTICPDatasetReactor reactor;
            slam::Scheduler scheduler;
            std::string last_file_path;
            struct DatasetForm : viz3d::ParamGroup {
                using viz3d::ParamGroup::ParamGroup;
                VIZ3D_PARAM_WITH_DEFAULT_VALUE(TextParam, root_path,
                                               "Root Path", "Path to the root of the dataset", "");
                VIZ3D_PARAM_WITH_DEFAULT_VALUE(TextParam, dataset_name,
                                               "Dataset Name",
                                               "Name of the Dataset (kitti_raw, kitti_360, kitti_carla, nclt)",
                                               "kitti_raw");
                VIZ3D_PARAM_WITH_DEFAULT_VALUE(TextParam, sequence_name,
                                               "sequence name",
                                               "Sequence name for the dataset (e.g. 00 for kitti_raw)", "00");
                VIZ3D_PARAM_WITH_DEFAULT_VALUE(FloatParam, frequency,
                                               "frequency",
                                               "Frequency (in Hz) of Frame Generation", 10.f);
                VIZ3D_PARAM_WITH_DEFAULT_VALUE(IntParam, frame_idx,
                                               "Initial Frame",
                                               "The initial frame index", 0);

                ~DatasetForm() {}
            } form;
        };
    };
}


#endif //CT_ICP_DATASET_WINDOW_H
