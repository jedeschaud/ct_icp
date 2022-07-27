#include <thread>

#include <SlamCore/types.h>
#include <SlamCore-viz3d/viz3d_utils.h>
#include <SlamCore-viz3d/viz3d_windows.h>
#include <SlamCore/experimental/synthetic.h>
#include <SlamCore/experimental/iterator/transform_iterator.h>

#include <ct_icp/ct_icp.h>
#include <ct_icp/odometry.h>
#include <ct_icp/config.h>

#include "testint_utils.h"
#include "ct_icp/reactors/registration.h"


/* ------------------------------------------------------------------------------------------------------------------ */
struct Config {
    ct_icp::CTICPOptions icp_options;
    std::shared_ptr<ct_icp::IMapOptions> map_options = nullptr;
    std::vector<std::shared_ptr<slam::AGeometricPrimitive>> primitives;

    Config() {
        map_options = std::make_shared<ct_icp::MultipleResolutionVoxelMap::Options>();
    }

    void LoadYAML(YAML::Node &node) {
        if (node["icp_options"]) {
            auto icp_node = node["icp_options"];
            icp_options = ct_icp::yaml_to_ct_icp_options(icp_node);
        }

        if (node["map_options"]) {
            auto map_node = node["map_options"];
            map_options = ct_icp::yaml_to_map_options(map_node);
        }

        primitives.clear();
        if (node["scene"]) {
            auto scene_node = node["scene"];
            SLAM_CHECK_STREAM(scene_node.IsSequence(), "The node `scene` is not a sequence");

            for (auto child: scene_node) {
                std::shared_ptr<slam::AGeometricPrimitive> primitive = nullptr;
                if (child["type"]) {
                    std::string type = child["type"].as<std::string>();
                    if (type == "triangle")
                        primitive = std::make_shared<slam::Triangle>();
                    if (type == "line")
                        primitive = std::make_shared<slam::Line>();
                    if (type == "sphere")
                        primitive = std::make_shared<slam::Sphere>();
                    if (type == "ball")
                        primitive = std::make_shared<slam::Ball>();
                }
                if (child["data"] && primitive) {
                    auto data_node = child["data"];
                    primitive->FromYAMLNode(data_node);
                    primitives.push_back(primitive);
                }
            }
        }
    }
};

/* ------------------------------------------------------------------------------------------------------------------ */
struct TEST_CT_ICP_WINDOW : slam::MultiPolyDataWindow {

    explicit TEST_CT_ICP_WINDOW(std::string &&winname) : slam::MultiPolyDataWindow(10) {
        auto window = std::make_shared<_Window>(std::move(winname));

        auto factory = std::make_unique<slam::PrototypeFactory<ct_icp::RegistrationReactor >>();
        factory->prototype.summary_notifier.AddObserverLambda(
                [this](const ct_icp::registration_output_t &output) {
                    // Transform the pose of the
                    auto _window = std::dynamic_pointer_cast<_Window>(window_);
                    if (_window) {
                        _window->registration_data.estimated_pose = output.frame.end_pose;
                        for (auto &point: _window->registration_data.noisy_scan)
                            point.world_point = output.frame.end_pose * point.raw_point.point;
                        auto pc = slam::PointCloud::WrapConstVector(_window->registration_data.noisy_scan,
                                                                    slam::WPoint3D::DefaultSchema(),
                                                                    "world_point");
                        this->AddPolyData("Estimated Scan", 0, slam::polydata_from_pointcloud(pc));
                        _window->BuildNeighbors("Last Scan Neighbors");
                    }
                });
        window->handler.SetFactory(std::move(factory));
        window->handler.Start();

        window_ = window;
    }

    ~TEST_CT_ICP_WINDOW() {
        auto window = std::dynamic_pointer_cast<_Window>(window_);
        if (window) {
            window->handler.Abort();
        }
    }

private:
    struct _Window : slam::MultiPolyDataWindow::ChildVTKWindow {

        _Window(std::string &&winname) : slam::MultiPolyDataWindow::ChildVTKWindow(std::move(winname)),
                                         form(window_name_ + "_form_id", window_name_ + " Form"),
                                         form_params(window_name_ + "_params_form_id", window_name_ + " Params Form") {
            ConstructScene();
            // Add poly datas to the scene
        }

        void DrawImGuiWindowConfigurations() override {
            form.Draw();

            if (handler.ConcurrentQueue().empty()) {
                if (viz3d::ImGui_HorizontalButton("Load Config")) {
                    try {
                        YAML::Node node = YAML::LoadFile(form.config_path.value);
                        config.LoadYAML(node);
                        ct_icp::ct_icp_message_t message(ct_icp::registration::CHANGE_PARAMS);
                        message.options_ptr = std::make_shared<ct_icp::CTICPOptions>();
                        *message.options_ptr = config.icp_options;
                        handler.ConcurrentQueue().push(message);
                        SLAM_LOG(INFO) << "Reloaded config from file: " << form.config_path.value;
                    } catch (...) {
                        SLAM_LOG(ERROR) << "Could not load config from file: " << form.config_path.value;
                    }
                }

                if (viz3d::ImGui_HorizontalButton("ReBuild Scene")) {
                    ConstructScene();
                }

                if (viz3d::ImGui_HorizontalButton("Run ICP")) {
                    ct_icp::ct_icp_message_t _message(ct_icp::registration::REGISTER_FRAME);
                    auto pc = slam::PointCloud::WrapConstVector(registration_data.noisy_scan,
                                                                slam::WPoint3D::DefaultSchema(),
                                                                "world_point");
                    pc.RegisterFieldsFromSchema();
                    _message.registration_message = ct_icp::registration_message_t(pc.DeepCopyPtr());
                    _message.registration_message->map = registration_data.icp_map;
                    _message.registration_message->initial_estimate = {
                            registration_data.estimated_pose,
                            registration_data.estimated_pose,
                    };
                    handler.ConcurrentQueue().push(_message);
                }

                if (viz3d::ImGui_HorizontalButton("Reset ICP")) {
                    registration_data.estimated_pose = registration_data.initial_pose;
                    for (auto &point: registration_data.noisy_scan)
                        point.world_point = registration_data.initial_pose.pose * point.raw_point.point;

                    RemovePolyData("Estimated Scan", 0);
                }
            }

            ChildVTKWindow::DrawImGuiWindowConfigurations();
        }

        void DrawSubordinatedImGuiContent() override {
            ChildVTKWindow::DrawSubordinatedImGuiContent();
            form_params.Draw();
        }

        struct Form : viz3d::ParamGroup {
            VIZ3D_PARAM_WITH_DEFAULT_VALUE(TextParam, config_path, "Config Path",
                                           "Path to the YAML config file", "");
            VIZ3D_PARAM_WITH_DEFAULT_VALUE(IntParam, map_num_points, "Num Points Per Primitive (Map)",
                                           "The number of points per primitive (for the map)", 1000)
            VIZ3D_PARAM_WITH_DEFAULT_VALUE(IntParam, scan_num_points, "Num Points Per Primitive (Scan)",
                                           "The number of points per primitive (for the map)", 300)

            using viz3d::ParamGroup::ParamGroup;
        } form;

        struct ParametersForm : viz3d::ParamGroup {
            using viz3d::ParamGroup::ParamGroup;
            VIZ3D_PARAM_WITH_DEFAULT_VALUE(FloatArray2, random_scale, "Random Scale for initial quat and tr",
                                           "Random noise Scale for initial tr and quat", 0.2f);
            VIZ3D_PARAM_WITH_DEFAULT_VALUE(FloatArray4, gt_quaternion, "Ground Truth Quaternion Parameters",
                                           "The parameters of the quaternion (x, y, z, w-1.f)", 0.f);
            VIZ3D_PARAM_WITH_DEFAULT_VALUE(FloatArray3, gt_tr, "Ground Truth Translation Parameters",
                                           "The parameters of the initial translation (x, y, z)", 0.f);
            VIZ3D_PARAM_WITH_DEFAULT_VALUE(FloatArray4, init_quaternion, "Initial Quaternion Parameters",
                                           "The parameters of the quaternion (x, y, z, w-1.f)", 0.f);
            VIZ3D_PARAM_WITH_DEFAULT_VALUE(FloatArray3, init_tr, "Initial Translation Parameters",
                                           "The parameters of the initial translation (x, y, z)", 0.f);
        } form_params;

        Config config;

        void ConstructScene() {

            // Build the point clouds
            registration_data.map.resize(0);
            registration_data.gt_scan.resize(0);
            registration_data.noisy_scan.resize(0);

            if (config.primitives.empty())
                return;

            // Build the Ground Truth Pose
            auto &gt_pose = registration_data.gt_pose.pose;
            {
                auto &gt_quat = gt_pose.quat;
                gt_quat.x() = form_params.gt_quaternion.value[0];
                gt_quat.y() = form_params.gt_quaternion.value[1];
                gt_quat.z() = form_params.gt_quaternion.value[2];
                gt_quat.w() = form_params.gt_quaternion.value[3] + 1.;

                auto &gt_tr = registration_data.gt_pose.pose.tr;
                gt_tr.x() = form_params.gt_tr.value[0];
                gt_tr.y() = form_params.gt_tr.value[1];
                gt_tr.z() = form_params.gt_tr.value[2];
            }

            // Build the initial pose
            auto &initial_pose = registration_data.initial_pose.pose;
            {
                auto &init_quat = initial_pose.quat;
                auto &init_tr = initial_pose.tr;
                init_quat.x() = form_params.init_quaternion.value[0];
                init_quat.y() = form_params.init_quaternion.value[1];
                init_quat.z() = form_params.init_quaternion.value[2];
                init_quat.w() = form_params.init_quaternion.value[3] + 1.;

                init_tr.x() = form_params.init_tr.value[0];
                init_tr.y() = form_params.init_tr.value[1];
                init_tr.z() = form_params.init_tr.value[2];
            }
            registration_data.estimated_pose = registration_data.initial_pose;


            for (auto &primitive: config.primitives) {
                auto map_points = primitive->GenerateRandomPointCloud(form.map_num_points.value);
                auto scan_points = primitive->GenerateRandomPointCloud(form.scan_num_points.value);

                {
                    auto old_size = registration_data.map.size();
                    // Build the map points
                    registration_data.map.resize(old_size + map_points.size());
                    for (auto idx(0); idx < map_points.size(); idx++) {
                        auto &wpoint = registration_data.map[old_size + idx];
                        wpoint.world_point = map_points[idx];
                        wpoint.raw_point.point = map_points[idx];
                        wpoint.Timestamp() = 0.;
                        wpoint.index_frame = 0;
                    }
                }

                {

                    auto old_size = registration_data.gt_scan.size();
                    // Build the scan points
                    registration_data.gt_scan.resize(old_size + scan_points.size());
                    for (auto idx(0); idx < scan_points.size(); idx++) {
                        auto &wpoint = registration_data.gt_scan[old_size + idx];
                        wpoint.world_point = scan_points[idx];
                        wpoint.raw_point.point = gt_pose.Inverse() * wpoint.world_point;
                        wpoint.Timestamp() = 1.;
                        wpoint.index_frame = 1;
                    }
                }
            }


            registration_data.noisy_scan = registration_data.gt_scan;
            for (auto &point: registration_data.noisy_scan) {
                point.WorldPoint() = initial_pose * point.RawPoint();
            }

            {
                auto pc_map = slam::PointCloud::WrapConstVector(registration_data.map,
                                                                slam::WPoint3D::DefaultSchema(),
                                                                "world_point");

                registration_data.icp_map = config.map_options->MakeMapFromOptions();
                std::vector<size_t> indices;
                registration_data.icp_map->InsertPointCloud(pc_map, indices);
                AddPolyData("Map Points", 0, slam::polydata_from_pointcloud(pc_map));
                AddPolyData("Map", 0, slam::polydata_from_pointcloud(*registration_data.icp_map->MapAsPointCloud()));
            }

//            {
//                auto gt_scan = slam::PointCloud::WrapConstVector(registration_data.gt_scan,
//                                                                 slam::WPoint3D::DefaultSchema(),
//                                                                 "world_point");
//                AddPolyData("Scan Points", 0, slam::polydata_from_pointcloud(gt_scan));
//            }

            {
                auto initial_pc = slam::PointCloud::WrapConstVector(registration_data.noisy_scan,
                                                                    slam::WPoint3D::DefaultSchema(),
                                                                    "world_point");
                AddPolyData("Initial Scan Points", 0, slam::polydata_from_pointcloud(initial_pc));

                // Compute nearest neighbors in the map (and show lines as a poly data)

                {
                    vtkNew<vtkPolyData> nn_poly_data;
                    vtkNew<vtkPoints> points;

                    const auto num_points = registration_data.noisy_scan.size();
                    points->Allocate(num_points * 2);
                    points->GetData()->SetName("Points_XYZ");
                    nn_poly_data->SetPoints(points.GetPointer());

                    // Assign for each cell vertex indices
                    vtkNew<vtkIdTypeArray> cell_ids;
                    cell_ids->Allocate(num_points * 3);
                    vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();

                    size_t line_idx = 0;
                    for (auto &point: registration_data.noisy_scan) {
                        auto neighbor = registration_data.icp_map->ComputeNeighborhood(point.world_point, 1);
                        if (!neighbor.points.empty()) {
                            points->InsertNextPoint(point.world_point.data());
                            points->InsertNextPoint(neighbor.points.front().data());
                            cell_ids->InsertNextValue(2);
                            cell_ids->InsertNextValue(line_idx * 2);
                            cell_ids->InsertNextValue(line_idx * 2 + 1);
                            line_idx++;
                        }
                    }

                    cellArray->SetCells(line_idx, cell_ids.GetPointer());
                    nn_poly_data->SetLines(cellArray);

                    BuildNeighbors("Neighbors");
                }
            }
        }

        void BuildNeighbors(const std::string &name) {
            vtkNew<vtkPolyData> nn_poly_data;
            vtkNew<vtkPoints> points;

            const auto num_points = registration_data.noisy_scan.size();
            points->Allocate(num_points * 2);
            points->GetData()->SetName("Points_XYZ");
            nn_poly_data->SetPoints(points.GetPointer());

            // Assign for each cell vertex indices
            vtkNew<vtkIdTypeArray> cell_ids;
            cell_ids->Allocate(num_points * 3);
            vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();

            size_t line_idx = 0;
            for (auto &point: registration_data.noisy_scan) {
                auto neighbor = registration_data.icp_map->ComputeNeighborhood(point.world_point, 1);
                if (!neighbor.points.empty()) {
                    points->InsertNextPoint(point.world_point.data());
                    points->InsertNextPoint(neighbor.points.front().data());
                    cell_ids->InsertNextValue(2);
                    cell_ids->InsertNextValue(line_idx * 2);
                    cell_ids->InsertNextValue(line_idx * 2 + 1);
                    line_idx++;
                }
            }

            cellArray->SetCells(line_idx, cell_ids.GetPointer());
            nn_poly_data->SetLines(cellArray);

            AddPolyData(name.c_str(), 0, nn_poly_data);
        }

        struct RegistrationData {
            std::shared_ptr<ct_icp::ISlamMap> icp_map = nullptr;
            std::vector<slam::WPoint3D> map, gt_scan, noisy_scan;
            slam::Pose gt_pose, initial_pose, estimated_pose;
        } registration_data;

        slam::Handler<ct_icp::RegistrationReactor, ct_icp::ct_icp_message_t> handler;
    };
};


// Test CT-ICP
// Objectif Chercher à reproduire en environnement synthétiques mes problèmes
// Puis rendre l'ICP robuste à ces problèmes (avec la nouvelle carte + registration)
//
// Problèmes à simuler (Pure ICP) :
//  - Différentes échelles (Far from sensor + Sparse, vs Close from sensor + dense)
//  - Problème du point-to-plane avec des lignes / poteaux / boules
//  - Automatic testing + testing avec démonstration
//
// L'enjeu est de démontrer le gain ! Faire une application qui démontre chaque bénéfice
//
// Synthétique -> Synthétique + bruit -> Réelles
//
// 1. Générer de la donnée
// 2. Choisir plusieurs options possible
// 3. Lancer les algorithmes et observer le résultat
int main(int argc, char **argv) {

    // 1. Génération de donnée synthétique
    std::thread gui_thread{viz3d::GUI::LaunchMainLoop, "TEST CT-ICP"};
    auto &instance = viz3d::GUI::Instance("TEST CT-ICP");

    auto window = std::make_shared<TEST_CT_ICP_WINDOW>("Test CT-ICP");
    window->InitializeWindow();

    gui_thread.join();
    viz3d::GUI::Instance().ClearWindows();

    return 0;
}