#include "SlamCore-viz3d/viz3d_windows.h"
#include "SlamCore-viz3d/viz3d_utils.h"
#include <misc/cpp/imgui_stdlib.h>
#include <viz3d/imgui_utils.h>

#include <algorithm>
#include <thread>

#include <vtkPolyDataMapper.h>
#include <vtkDataSet.h>
#include <vtkPointData.h>
#include <vtkLinearTransform.h>
#include <vtkTransform.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkCellData.h>

namespace slam {

    /* -------------------------------------------------------------------------------------------------------------- */
    void VTKActorColoringScheme::ApplyToActor(vtkSmartPointer<vtkActor> actor) {
        auto *input = actor->GetMapper()->GetInput();
        if (!input)
            return;
        actor->GetMapper()->SetLookupTable(lookup_table);
        auto *point_data = input->GetPointData()->GetScalars(scalar_field.c_str());
        if (point_data) {
            input->GetPointData()->SetActiveScalars(scalar_field.c_str());
            actor->GetMapper()->SetScalarRange(min_max_[0], min_max_[1]);
            return;
        }
        auto *cell_data = input->GetCellData()->GetScalars(scalar_field.c_str());
        if (cell_data) {
            input->GetCellData()->SetActiveScalars(scalar_field.c_str());
            actor->GetMapper()->SetScalarRange(min_max_[0], min_max_[1]);
            return;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void VTKActorColoringScheme::FindScalarFields(vtkSmartPointer<vtkActor> actor) {
        ClearFields();
        auto *mapper = actor->GetMapper();
        if (!mapper)
            return;

        std::set<std::string> fields_;
        auto insert_array_names = [&](auto *data) {
            if (!data)
                return;
            auto num_arrays = data->GetNumberOfArrays();
            for (auto array_id(0); array_id < num_arrays; array_id++) {
                auto array_name = std::string(data->GetArrayName(array_id));
                if (fields_.find(array_name) == fields_.end())
                    fields_.insert(array_name);
            }
        };
        insert_array_names(mapper->GetInput()->GetPointData());
        insert_array_names(mapper->GetInput()->GetCellData());
        scalar_fields.insert(scalar_fields.end(), fields_.begin(), fields_.end());
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void VTKActorColoringScheme::ClearFields() {
        scalar_fields.clear();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void VTKActorColoringScheme::SetDefaultField(std::string &&field) {
        scalar_field = std::move(field);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void VTKActorColoringScheme::MinMaxBuilder::AddActor(vtkSmartPointer<vtkActor> actor) {
        auto scalars = actor->GetMapper()->GetInput()->GetPointData()->GetScalars(scalar_field.c_str());
        if (!scalars) {
            scalars = actor->GetMapper()->GetInput()->GetCellData()->GetScalars(scalar_field.c_str());
            if (!scalars)
                return;
        }
        double range[2];
        scalars->GetRange(range);
        if (std::numeric_limits<double>::max() == range[0] || std::numeric_limits<double>::min() == range[1])
            return;
        if (min_max[0] > range[0])
            min_max[0] = range[0];
        if (min_max[1] < range[1])
            min_max[1] = range[1];
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ChildVTKWindow::AddPolyData(std::string &&group_name,
                                                          int id,
                                                          vtkSmartPointer<vtkPolyData> polydata) {
        RemovePolyData(group_name, id);
        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polydata);
        vtkNew<vtkActor> actor;
        actor->SetMapper(mapper);
        actors_by_group_[group_name][id] = actor;
        AddActor(actor);
        do_update_group_information_ = true;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ChildVTKWindow::RemovePolyData(const std::string &group_name, int id) {
        if (actors_by_group_.find(group_name) != actors_by_group_.end()) {
            auto &group = actors_by_group_[group_name];
            if (group.find(id) != group.end()) {
                auto actor = group[id];
                RemoveActor(actor);
                group.erase(id);
                do_update_group_information_ = true;
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ChildVTKWindow::EraseGroup(const std::string &group_name) {
        if (actors_by_group_.find(group_name) != actors_by_group_.end()) {
            for (auto &actor: actors_by_group_[group_name])
                RemoveActor(actor.second);
            actors_by_group_.erase(group_name);
            group_imgui_vars_.erase(group_name);
            do_update_group_information_ = true;
        }
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ChildVTKWindow::GroupOptionsPopup(const std::string &group, bool open) {
        ImGui::PushID(group.c_str());
        if (open)
            ImGui::OpenPopup("color_group");

        if (ImGui::BeginPopup("color_group")) {
            ImGui::Text("Color Scale Options:");

            auto &group_options = group_imgui_vars_[group];
            const char *combo_preview_value = nullptr;
            if (!group_options.field_names.empty()
                && group_options.selected < group_options.field_names.size()
                && group_options.selected >= 0) {
                auto it = group_options.field_names.begin();
                std::advance(it, group_options.selected);
                combo_preview_value = (*it).c_str();
            } else
                combo_preview_value = "";

            ImGui::Checkbox("Hide Group", &group_options.hide_group);
            ImGui::Checkbox("Apply to new Actors", &group_options.apply_to_new_actors);
            ImGui::Checkbox("Update MinMax", &group_options.update_minmax);
            ImGui::DragInt("Point Size", &group_options.point_size, 1, 10);
            ImGui::DragInt("Line Size", &group_options.line_size, 1, 10);

            ImGui::Separator();
            if (ImGui::BeginCombo("Field Selection", combo_preview_value)) {
                auto it = group_options.field_names.begin();
                for (int i(0); i < group_options.field_names.size(); i++) {
                    const bool is_selected = (group_options.selected == i);
                    if (ImGui::Selectable((*it).c_str(), is_selected)) {
                        group_options.selected = i;
                        group_options.selected_field = *it;
                    }
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                    it++;
                }
                ImGui::EndCombo();
            }
            ImGui::Separator();
            group_options.color_combo.Draw();
            if (group_options.color_combo.GetSelectedColorMapType() !=
                group_options.colormap_type) {
                group_options.colormap_type = group_options.color_combo.GetSelectedColorMapType();
            }
            ImGui::Separator();


            ImVec2 button_size = ImVec2(
                    (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().FramePadding.x) * 0.5f,
                    2 * ImGui::GetFontSize());
            ImGui::InputFloat2("Scalar Range", group_options.scalar_range);
            if (ImGui::Button("Set to Min-Max", ImVec2(button_size.x * 2.f, button_size.y))) {
                float min_max[2] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::min()};
                GetMinMaxScalar(group, min_max[0], min_max[1]);
                double actor_min_max[2];

                for (auto &actor: actors_by_group_[group]) {
                    auto scalars = actor.second->GetMapper()->GetInput()->GetPointData()->GetScalars();
                    if (scalars) {
                        scalars->GetRange(actor_min_max);
                        if (actor_min_max[0] < min_max[0])
                            min_max[0] = float(actor_min_max[0]);
                        if (actor_min_max[1] > min_max[0])
                            min_max[1] = float(actor_min_max[1]);
                    }
                }
                if (min_max[0] != std::numeric_limits<float>::max() &&
                    min_max[1] != std::numeric_limits<float>::min()) {
                    group_options.scalar_range[0] = min_max[0];
                    group_options.scalar_range[1] = min_max[1];
                }

                for (auto &actor: actors_by_group_[group]) {
                    actor.second->GetMapper()->SetScalarRange(group_options.scalar_range[0],
                                                              group_options.scalar_range[1]);
                }
            }

            static float scale = 1.f;
            static float xyz[3] = {0.f, 0.f, 0.f};
            if (ImGui::TreeNode("Transform Options:")) {
                ImGui::InputFloat("Model Scale", &scale, 0.1f, 1.f);
                ImGui::InputFloat3("Model Translation (XYZ)", xyz);
                ImGui::TreePop();
            }

            ImGui::Separator();
            if (ImGui::Button("Apply", button_size)) {
                auto new_transform = vtkSmartPointer<vtkTransform>::New();
                new_transform->Scale(scale, scale, scale);
                new_transform->Translate(xyz);
                ApplyTransform(group, new_transform);
                for (auto &actor: actors_by_group_[group]) {
                    auto *input = actor.second->GetMapper()->GetInput();
                    input->GetPointData()->SetActiveScalars(group_options.selected_field.c_str());
                    input->GetCellData()->SetActiveScalars(group_options.selected_field.c_str());
                    actor.second->GetProperty()->SetPointSize(std::max(group_options.point_size, 1));
                    actor.second->GetProperty()->SetLineWidth(std::max(group_options.line_size, 1));
                }
            }

            ImGui::SameLine();
            if (ImGui::Button("Close", button_size))
                ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
        ImGui::PopID();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ChildVTKWindow::GetMinMaxScalar(const std::string &group_name, float &min, float &max) {
        float min_max[2] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::min()};
        double actor_min_max[2];
        for (auto &actor: actors_by_group_[group_name]) {
            auto scalars = actor.second->GetMapper()->GetInput()->GetPointData()->GetScalars();
            if (scalars) {
                scalars->GetRange(actor_min_max);
                if (actor_min_max[0] < min_max[0])
                    min_max[0] = float(actor_min_max[0]);
                if (actor_min_max[1] > min_max[0])
                    min_max[1] = float(actor_min_max[1]);
            }
        }
        min = min_max[0];
        max = min_max[1];
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ChildVTKWindow::UpdateGroupInformation() {
        std::lock_guard lock{actors_management_mutex_};
        for (auto &group: actors_by_group_) {
            auto &group_name = group.first;
            auto &actors = group.second;
            if (group_imgui_vars_.find(group_name) == group_imgui_vars_.end())
                group_imgui_vars_[group_name] = GroupImGuiVars(group_name);
            auto &group_params = group_imgui_vars_[group_name];

            auto lookup_table = viz3d::ColorMap(group_params.colormap_type);
            if (group_params.apply_transform_to_all_) {
                if (group_params.transform) {
                    for (auto &actor: actors) {
                        actor.second->SetUserTransform(group_params.transform);
                    }
                }
                group_params.apply_transform_to_all_ = false;
            }

            std::set<std::string> scalar_fields{""};
            for (auto &actor: actors) {

                // Find Point Data Fields
                auto *mapper = actor.second->GetMapper();
                mapper->SetLookupTable(lookup_table);
                auto *point_data = actor.second->GetMapper()->GetInput()->GetPointData();
                auto *cell_data = actor.second->GetMapper()->GetInput()->GetCellData();

                auto insert_array_names = [&](auto &data) {
                    auto num_arrays = data->GetNumberOfArrays();
                    for (auto array_id(0); array_id < num_arrays; array_id++) {
                        auto array_name = std::string(data->GetArrayName(array_id));
                        if (scalar_fields.find(array_name) == scalar_fields.end())
                            scalar_fields.insert(array_name);
                    }
                };
                insert_array_names(point_data);
                insert_array_names(cell_data);

                std::swap(group_params.field_names, scalar_fields);
                if (group_params.apply_to_new_actors) {
                    cell_data->SetActiveScalars(group_params.selected_field.c_str());
                    point_data->SetActiveScalars(group_params.selected_field.c_str());
                    mapper->SetScalarRange(group_params.scalar_range[0], group_params.scalar_range[1]);
                    if (!group_params.transform)
                        group_params.transform = vtkSmartPointer<vtkTransform>::New();
                    actor.second->SetUserTransform(group_params.transform);
                    actor.second->GetProperty()->SetPointSize(std::max(group_params.point_size, 1));
                    actor.second->GetProperty()->SetLineWidth(std::max(group_params.line_size, 1));
                }

                if (group_params.hide_group)
                    actor.second->VisibilityOff();
                else
                    actor.second->VisibilityOn();
            }

            if (group_params.update_minmax) {
                float minmax[2];
                GetMinMaxScalar(group_name, minmax[0], minmax[1]);

                if (minmax[0] != std::numeric_limits<float>::max() && minmax[1] != std::numeric_limits<float>::min()) {
                    group_params.scalar_range[0] = minmax[0];
                    group_params.scalar_range[1] = minmax[1];
                }
            }
        }
        do_update_group_information_ = false;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ChildVTKWindow::ApplyTransform(const std::string &group_name,
                                                             vtkSmartPointer<vtkTransform> transform) {
        if (group_imgui_vars_.find(group_name) != group_imgui_vars_.end()) {
            std::lock_guard lock{actors_management_mutex_};
            group_imgui_vars_[group_name].transform = transform;
            group_imgui_vars_[group_name].apply_transform_to_all_ = true;
            do_update_group_information_ = true;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ChildVTKWindow::ResetTransform(const std::string &group_name) {
        ApplyTransform(group_name, vtkSmartPointer<vtkTransform>::New());
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<std::string> MultiPolyDataWindow::ChildVTKWindow::GetAllGroups() {
        std::vector<std::string> group_names;
        for (auto &group: actors_by_group_)
            group_names.push_back(group.first);
        return group_names;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ChildVTKWindow::SetSelectedField(const std::string &group_name,
                                                               const std::string &field) {
        group_imgui_vars_[group_name].selected_field = field;
    }

    standard_message_t::message_data_t::~message_data_t() {}

    namespace {

        enum MESSAGE_TYPE {
            ADD_POLYDATA = 1,
            REMOVE_POLYDATA = 2,
            REMOVE_GROUP = 3,
            SET_TRANSFORM = 4,
            SET_FIELD = 5
        };

        struct add_polydata_payload : standard_message_t::message_data_t {
            std::string group_name;
            int idx;
            vtkSmartPointer<vtkPolyData> polydata;

            add_polydata_payload(std::string &&group_name, int idx, vtkSmartPointer<vtkPolyData> polydata) :
                    group_name(std::move(group_name)), idx(idx), polydata(polydata) {}
        };

        struct remove_item_payload : standard_message_t::message_data_t {
            std::string group_name;
            int idx = -1;

            remove_item_payload(std::string &&group_name, int idx) : group_name(std::move(group_name)), idx(idx) {}
        };

        standard_message_t MakeAddPolyDataMessage(std::string &&winname, int idx,
                                                  vtkSmartPointer<vtkPolyData> polydata) {
            standard_message_t message((int) ADD_POLYDATA);
            message.data = std::dynamic_pointer_cast<standard_message_t::message_data_t>(
                    std::make_shared<add_polydata_payload>(std::move(winname), idx, polydata));
            return message;
        }

        struct set_transform_payload : standard_message_t::message_data_t {
            vtkSmartPointer<vtkTransform> transform = nullptr;
            std::string group_name;
            std::optional<int> id;

            set_transform_payload(std::string &&group_name,
                                  vtkSmartPointer<vtkTransform> transform,
                                  std::optional<int> id = {}) : group_name(std::move(group_name)),
                                                                transform(transform), id(id) {}
        };

        struct set_field_payload : standard_message_t::message_data_t {
            std::string group_name;
            std::string field_name;

            set_field_payload(std::string &&group_name, std::string &&field_name) :
                    group_name(std::move(group_name)),
                    field_name(std::move(field_name)) {}
        };
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ChildVTKWindow::DrawImGuiWindowConfigurations() {
        if (do_update_group_information_) {
            UpdateGroupInformation();
            do_update_group_information_ = false;
        }

        if (!update_messages_queue.empty()) {
            const int kNumUpdatesPerFrame = 1;
            for (int i(0); i < kNumUpdatesPerFrame; ++i) {
                if (update_messages_queue.empty())
                    break;
                auto item = update_messages_queue.pop();
                if (item) {
                    switch (item->message_type) {
                        // HANDLE Messages (only one per frame at maximum)
                        case MESSAGE_TYPE::ADD_POLYDATA: {
                            auto data = std::dynamic_pointer_cast<add_polydata_payload>(item->data);
                            if (data)
                                AddPolyData(std::string(data->group_name), data->idx, data->polydata);
                        }
                            break;
                        case MESSAGE_TYPE::REMOVE_GROUP:
                        case MESSAGE_TYPE::REMOVE_POLYDATA: {
                            auto data = std::dynamic_pointer_cast<remove_item_payload>(item->data);
                            if (data) {
                                if (item->message_type == MESSAGE_TYPE::REMOVE_GROUP)
                                    EraseGroup(data->group_name);
                                else
                                    RemovePolyData(data->group_name, data->idx);
                            }
                        }
                            break;
                        case MESSAGE_TYPE::SET_TRANSFORM: {
                            auto data = std::dynamic_pointer_cast<set_transform_payload>(item->data);
                            if (data) {
                                ApplyTransform(data->group_name, data->transform);
                            }
                        }
                            break;
                        case MESSAGE_TYPE::SET_FIELD: {
                            auto data = std::dynamic_pointer_cast<set_field_payload>(item->data);
                            if (data)
                                SetSelectedField(data->group_name,
                                                 data->field_name);

                        }
                            break;
                    }
                }
            }
            if (do_update_group_information_)
                UpdateGroupInformation();
        }


        viz3d::VTKWindow::DrawImGuiWindowConfigurations();
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ChildVTKWindow::DrawSubordinatedImGuiContent() {
        ImGui::Text("Window Parameters:             ");
        ImGui::Separator();

        if (ImGui::CollapsingHeader("Window Configuration")) {
            BackgroundPopup(viz3d::ImGui_HorizontalButton("Background Color"));
            RenderingPopup(viz3d::ImGui_HorizontalButton("Rendering Options"));
        }

        if (ImGui::CollapsingHeader("Group Configurations")) {
            int idx(0);
            for (auto &group: actors_by_group_) {
                GroupOptionsPopup(group.first, viz3d::ImGui_HorizontalButton(group.first.c_str()));
                idx++;
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ChildVTKWindow::SetCapacity(int queue_size) { update_messages_queue.set_max_capacity(queue_size); }

    /* -------------------------------------------------------------------------------------------------------------- */
    MultiPolyDataWindow::MultiPolyDataWindow(std::string &&winname, int max_capacity) {
        window_ = std::make_shared<ChildVTKWindow>(std::move(winname));
        window_->update_messages_queue.set_max_capacity(max_capacity);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::SetSelectedField(const std::string &group_name, const std::string &field) {
        if (window_) {
            standard_message_t message(MESSAGE_TYPE::SET_FIELD);
            message.data = std::make_shared<set_field_payload>(std::string(group_name), std::string(field));
            window_->update_messages_queue.push(message);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::AddPolyData(std::string &&group_name, int id, vtkSmartPointer<vtkPolyData> poly_data) {
        if (window_) {
            standard_message_t message(MESSAGE_TYPE::ADD_POLYDATA);
            message.data = std::make_shared<add_polydata_payload>(std::string(group_name), id, poly_data);
            window_->update_messages_queue.push(message);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::EraseGroup(const std::string &group_name) {
        if (window_) {
            standard_message_t message(MESSAGE_TYPE::REMOVE_GROUP);
            message.data = std::make_shared<remove_item_payload>(std::string(group_name), -1);
            window_->update_messages_queue.push(message);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::RemovePolyData(const std::string &group_name, int id) {
        if (window_) {
            standard_message_t message(MESSAGE_TYPE::REMOVE_POLYDATA);
            message.data = std::make_shared<remove_item_payload>(std::string(group_name), id);
            window_->update_messages_queue.push(message);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ApplyTransform(const std::string &group_name, vtkSmartPointer<vtkTransform> transform) {
        if (window_) {
            standard_message_t message(MESSAGE_TYPE::SET_TRANSFORM);
            message.data = std::make_shared<set_transform_payload>(std::string(group_name), transform);
            window_->update_messages_queue.push(message);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MultiPolyDataWindow::ResetTransform(const std::string &group_name) {
        if (window_) {
            standard_message_t message(MESSAGE_TYPE::SET_TRANSFORM);
            message.data = std::make_shared<set_transform_payload>(std::string(group_name),
                                                                   vtkSmartPointer<vtkTransform>::New());
            window_->update_messages_queue.push(message);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloudQueueVTKWindow::DrawImGuiWindowConfigurations() {

        // Process actors
//        auto vtk_actor_option = {};
//        if (vtk_actor_option.has_value()) {
//            if (vtk_actor_option->Get()) {
//                if (vtk_last_actor) {
//                    RemoveActor(vtk_last_actor);
//                    vtk_last_actor = nullptr;
//                }
//
//                if (update_with_new_actors) {
//                    auto minmax_builder = color_scheme_.GetMinMaxBuilder();
//                    minmax_builder.AddActor(*vtk_actor_option);
//                    color_scheme_.SetMinMax(minmax_builder);
//                }
//                if (apply_to_new_actors)
//                    color_scheme_.ApplyToActor(*vtk_actor_option);
//                color_scheme_.FindScalarFields(*vtk_actor_option);
//
//
//                AddActor(*vtk_actor_option);
//                vtk_last_actor = *vtk_actor_option;
//            }
//        }

        viz3d::VTKWindow::DrawImGuiWindowConfigurations();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloudQueueVTKWindow::ClearWindow() {
        sliding_window_frames_.clear();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    namespace {
        std::string frame_id_to_str(size_t fid) {
            std::stringstream ss;
            ss << std::setfill('0') << std::setw(10) << fid;
            ss << ".ply";
            return ss.str();
        };
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloudQueueVTKWindow::PushNewFrame(slam::PointCloudPtr new_frame) {
        if (!new_frame)
            return;

//        pc_to_vtk_handler_.PushMessage(new_frame);
        if (is_recording) {
            sliding_window_frames_.push_back({
                                                     frame_idx,
                                                     frame_id_to_str(frame_idx),
                                                     new_frame
                                             });
        }

        frame_idx++;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloudQueueVTKWindow::SetSchemaMapper(const PLYSchemaMapper &mapper) {
        pcwriter_handler_.PushMessage(std::make_shared<SetSchemaMapperMC>(mapper));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloudQueueVTKWindow::DrawSubordinatedImGuiContent() {

        if (ImGui::CollapsingHeader("Window Configuration")) {
            BackgroundPopup(viz3d::ImGui_HorizontalButton("Background Color"));
            RenderingPopup(viz3d::ImGui_HorizontalButton("Rendering Options"));
            ModelColorPopup(viz3d::ImGui_HorizontalButton("Colors Options"));
        }
        ImGui::Separator();
        // Add Buttons to record / clear / pause the recording of the frames
        if (!is_recording) {
            if (viz3d::ImGui_HorizontalButton("Record Frames", 0.5f))
                is_recording = true;
        } else {
            if (viz3d::ImGui_HorizontalButton("Pause Recording", 0.5f))
                is_recording = false;
        }

        ImGui::SameLine();
        if (viz3d::ImGui_HorizontalButton("Clear Window"))
            ClearWindow();

        ImGui::InputText("Output Directory", &destination_directory_);
        ImGui::Separator();

        if (!sliding_window_frames_.empty()) {
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0., 1., 0., 1.));
            if (viz3d::ImGui_HorizontalButton("Save Frames")) {
                for (auto &frame: sliding_window_frames_) {
                    pcwriter_handler_.PushMessage(std::make_shared<WriteFrameMC>(
                            frame.pc_ptr,
                            frame.file_name,
                            std::optional<std::string>{destination_directory_}));
                }
            }
            ImGui::PopStyleColor();

            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1., 0., 0., 1.));
            if (viz3d::ImGui_HorizontalButton("Clear Frames"))
                ClearWindow();
            ImGui::PopStyleColor();

            ImGui::Separator();
            ImGui::Text("Frames registered:");
            for (auto &frame: sliding_window_frames_) {
                ImGui::Text("%s", frame.file_name.c_str());
            }

        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PointCloudQueueVTKWindow::ModelColorPopup(bool open) {
        if (open)
            ImGui::OpenPopup("coloring_scheme");

        if (ImGui::BeginPopup("coloring_scheme")) {
            viz3d::ImGui_CenteredText("Coloring Scheme");

            ImGui::Separator();
            ImGui::Checkbox("Apply to new Actors", &apply_to_new_actors);
            ImGui::Checkbox("Update with new Actors", &update_with_new_actors);
            ImGui::InputFloat2("Min-Max Scalars", color_scheme_.GetMinMax());
            if (viz3d::ImGui_HorizontalButton("Apply") && vtk_last_actor) {
                color_scheme_.ApplyToActor(vtk_last_actor);
            }
            ImGui::Separator();

            if (ImGui::BeginCombo("Field Selection", color_scheme_.GetSelectedField().c_str())) {
                auto &field_names = color_scheme_.GetScalarFields();
                auto it = field_names.begin();
                for (int i(0); i < field_names.size(); i++) {
                    const bool is_selected = (color_scheme_.GetSelectedIndex() == i);
                    if (ImGui::Selectable((*it).c_str(), is_selected)) {
                        color_scheme_.GetSelectedField() = i;
                        color_scheme_.SetDefaultField(std::string(*it));
                    }
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                    it++;
                }
                ImGui::EndCombo();
            }
            this->imgui_vars_.color_combo.Draw();
            if (imgui_vars_.color_combo.GetSelectedColorMapType() != color_scheme_.GetColorMapType())
                color_scheme_.SetColorMap(imgui_vars_.color_combo.GetSelectedColorMap());

            ImGui::EndPopup();
        }
    }

}

