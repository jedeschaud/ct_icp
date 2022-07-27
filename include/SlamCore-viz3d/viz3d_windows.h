#ifndef SLAMCORE_VIZ3D_WINDOWS_H
#define SLAMCORE_VIZ3D_WINDOWS_H

#ifdef SLAM_WITH_VIZ3D

#include <map>
#include <list>
#include <thread>
#include <memory>
#include <queue>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkTransform.h>

#include <viz3d/vtk_window.h>

#include "SlamCore/types.h"
#include "SlamCore/io.h"
#include "SlamCore/reactors/reactor.h"
#include "SlamCore/reactors/handler.h"

#include <SlamCore/pointcloud.h>
#include <SlamCore/reactors/pointcloud_writer.h>

#include <SlamCore-viz3d/viz3d_utils.h>


namespace slam {

    class VTKActorColoringScheme {
    public:
        struct MinMaxBuilder {
            float min_max[2] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::min()};
            std::string scalar_field;

            void AddActor(vtkSmartPointer<vtkActor> actor);
        };

        inline MinMaxBuilder GetMinMaxBuilder() {
            return MinMaxBuilder{
                    {std::numeric_limits<float>::max(),
                     std::numeric_limits<float>::min()}, scalar_field};
        };

        inline float *GetMinMax() { return min_max_; }

        inline std::string &GetSelectedField() { return scalar_field; }

        inline int &GetSelectedIndex() { return selected_index; }

        inline std::vector<std::string> &GetScalarFields() { return scalar_fields; }

        void ApplyToActor(vtkSmartPointer<vtkActor> actor);

        void SetDefaultField(std::string &&field);

        void ClearFields();

        void FindScalarFields(vtkSmartPointer<vtkActor> actor);

        inline void SetMinMax(MinMaxBuilder builder) {
            if (builder.min_max[0] == std::numeric_limits<float>::max() ||
                builder.min_max[1] == std::numeric_limits<float>::min())
                return;
            min_max_[0] = builder.min_max[0];
            min_max_[1] = builder.min_max[1];
        };

        inline void SetColorMap(vtkSmartPointer<vtkLookupTable> cmap) { lookup_table = cmap; }

        viz3d::VTKColorMapType GetColorMapType() const { return type; };

    private:
        int selected_index = 0;
        float min_max_[2] = {0.f, 1.f};
        std::string scalar_field;
        std::vector<std::string> scalar_fields;
        viz3d::VTKColorMapType type = viz3d::VTKColorMapType::INFERNO;
        vtkSmartPointer<vtkLookupTable> lookup_table = ColorMap(type);
    };

    struct standard_message_t {

        standard_message_t(int msg) : message_type(msg) {}

        const int message_type = 0;

        struct message_data_t {
            virtual ~message_data_t() = 0;
        };

        std::shared_ptr<message_data_t> data = nullptr;
    };

    // A wrapper of a VTKWindow for rendering and controlling multiple PolyDatas by groups.
    class MultiPolyDataWindow {
    public:
        MultiPolyDataWindow(std::string &&winname, int queue_size = 20);

        // Sets the selected field for the given group
        void SetSelectedField(const std::string &group_name, const std::string &field);

        // Registers a PolyData with a given group name and a group id
        void AddPolyData(std::string &&group_name, int id, vtkSmartPointer<vtkPolyData> poly_data);

        // Removes the group from the window
        void EraseGroup(const std::string &group_name);

        // Removes a poly data
        void RemovePolyData(const std::string &group_name, int id);

        // Applies a transform to all the actors in the group
        void ApplyTransform(const std::string &group_name, vtkSmartPointer<vtkTransform> transform);

        // Resets the group transform (applies identity to all the actors in the group)
        void ResetTransform(const std::string &group_name);

        void InitializeWindow() {
            auto &instance = viz3d::GUI::Instance();
            window_->InitializeVTKContext();
            instance.AddWindow(window_);
        }

        ~MultiPolyDataWindow() {
            window_ = nullptr;
        }

    protected:
        explicit MultiPolyDataWindow(int queue_size = 10) {
            //!\\ The Child class must populate the window pointer
            window_ = nullptr;
        };

        struct ChildVTKWindow : public viz3d::VTKWindow {

            using viz3d::VTKWindow::VTKWindow;

            // Sets the selected field for the given group
            void SetSelectedField(const std::string &group_name, const std::string &field);

            // Registers a PolyData with a given group name and a group id
            void AddPolyData(std::string &&group_name, int id, vtkSmartPointer<vtkPolyData> poly_data);

            // Removes the group from the window
            void EraseGroup(const std::string &group_name);

            // Removes a poly data
            void RemovePolyData(const std::string &group_name, int id);

            // Applies a transform to all the actors in the group
            void ApplyTransform(const std::string &group_name, vtkSmartPointer<vtkTransform> transform);

            // Resets the group transform (applies identity to all the actors in the group)
            void ResetTransform(const std::string &group_name);

            // Returns all the groups names
            std::vector<std::string> GetAllGroups();

            void DrawSubordinatedImGuiContent() override;

            // Draws the ImGui Content for the window
            void DrawImGuiWindowConfigurations() override;

            // Updates Group Information from the Poly Data
            void UpdateGroupInformation();

            slam::blocking_queue<standard_message_t> update_messages_queue;

            struct GroupImGuiVars {
                std::set<std::string> field_names;
                std::string selected_field;
                float scalar_range[2] = {0., 1.f};
                int point_size = 1;
                int line_size = 1;
                int selected = 0;
                bool update_minmax = true;
                bool apply_to_new_actors = true;
                bool apply_transform_to_all_ = true;
                bool hide_group = false;
                vtkSmartPointer<vtkTransform> transform = nullptr;

                viz3d::VTKColorMapType colormap_type = viz3d::VTKColorMapType::PLASMA;
                viz3d::ImGui_ColorMapCombo color_combo;

                GroupImGuiVars() : color_combo("") {}

                explicit GroupImGuiVars(const std::string &group_name) : color_combo(std::string(group_name)) {}
            };

            // Colors the actor's group
            virtual void GroupOptionsPopup(const std::string &group, bool open = false);

            void GetMinMaxScalar(const std::string &group_name, float &min, float &max);

            std::map<std::string, std::map<int, vtkSmartPointer<vtkActor>>> actors_by_group_;

            struct MultiPolyDataWindowImGuiVars_ {
                bool open_window_options = false;
            } imgui_vars2_;

            bool do_update_group_information_ = true;
            std::map<std::string, GroupImGuiVars> group_imgui_vars_;
        };
        std::shared_ptr<ChildVTKWindow> window_ = nullptr;
    };


    // A VTKWindow tracking a Queue of PointClouds which can be saved to disk from a dialog
    class PointCloudQueueVTKWindow : public viz3d::VTKWindow {
    public:
        using viz3d::VTKWindow::VTKWindow;

        // Pushes a new frame, which will be added to the window if the window is recording.
        void PushNewFrame(slam::PointCloudPtr new_frame);

        // Clears the window
        void ClearWindow();

        // Draws the ImGUI configurations
        void DrawImGuiWindowConfigurations() override;

        // The Output directory for the PLY pointcloud
        REF_GETTER(OutputDirectory, destination_directory_);

        // The Size of the sliding window
        REF_GETTER(SlidingWindowSize, sliding_window_size);

        void SetSchemaMapper(const PLYSchemaMapper &mapper);

        PointCloudQueueVTKWindow(std::string &&name) : viz3d::VTKWindow(std::move(name)) {
            pcwriter_handler_.Start();
        }

        ~PointCloudQueueVTKWindow() {
            pcwriter_handler_.Abort();
        }

    protected:
        void DrawSubordinatedImGuiContent() override;

        void ModelColorPopup(bool open = false);

    private:
        bool show_window_content = false;
        bool is_recording = false;
        int sliding_window_size = 100;
        size_t frame_idx = 0;
        std::string destination_directory_ = "/tmp/";

        bool update_with_new_actors = true;
        bool apply_to_new_actors = true;
        VTKActorColoringScheme color_scheme_;

        struct Frame {
            size_t frame_idx = 0;
            std::string file_name;
            slam::PointCloudPtr pc_ptr;
        };

        std::list<Frame> sliding_window_frames_;
        vtkSmartPointer<vtkActor> vtk_last_actor = nullptr;
        Handler<PointCloudWriter, pcwriter_message_t> pcwriter_handler_;
    };

}

#endif // SLAM_WITH_VIZ3D

#endif //SLAMCORE_VIZ3D_WINDOWS_H
