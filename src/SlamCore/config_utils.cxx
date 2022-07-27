#include <iostream>
#include <exception>

#include <glog/logging.h>
#include <tclap/CmdLine.h>

#include "SlamCore/utils.h"
#include "SlamCore/config_utils.h"

namespace slam::config {

    /* -------------------------------------------------------------------------------------------------------------- */
    void FindEnumOption(const YAML::Node &node, int &option_enum,
                        std::string option_name,
                        std::initializer_list<std::pair<std::string, int>> options) {
        std::map<std::string, int> map_options(options.begin(), options.end());
        if (node[option_name]) {
            auto entry = node[option_name].as<std::string>();
            if (map_options.find(entry) == map_options.end()) {
                std::stringstream ss_exception;
                ss_exception << "[ERROR] The option " << entry << " is not a valid option. Valid options are: {";
                for (auto it = map_options.begin(); it != map_options.end(); ++it) {
                    if (it != map_options.begin())
                        ss_exception << ", ";
                    ss_exception << it->first;
                }
                ss_exception << "}" << std::endl;

                throw std::invalid_argument(ss_exception.str());
            }

            option_enum = map_options[entry];
        }
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    YAML::Node RootNode(const std::string &config_path, const std::optional<std::vector<std::string>> &overrides_path) {
        try {
            auto root_node = YAML::LoadFile(config_path);
            if (overrides_path) {
                for (auto &path: overrides_path.value()) {

                    CHECK(fs::exists(path)) << "The file path " << path << " does not exist" << std::endl;
                    auto overrides_node = YAML::LoadFile(path);
                    MergeOverridesNode(root_node, overrides_node);
                }
            }
            return root_node;
        }
        catch (...) {
            LOG(ERROR) << "Could not load the file " << config_path << " with "
                       << overrides_path.value_or(std::vector<std::string>{}).size() << " from disk." << std::endl;
            throw;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    YAML::Node FindNode(YAML::Node node, const std::string &path, bool create_if_absent) {
        std::vector<std::string> tokens;
        {
            std::string full_path(path), token;
            while (!full_path.empty()) {
                auto idx = full_path.find_first_of('.');
                if (idx < full_path.size()) {
                    token = full_path.substr(0, idx);
                    full_path = full_path.substr(idx + 1, full_path.size());
                    tokens.push_back(token);
                    continue;
                }
                tokens.push_back(full_path);
                full_path.clear();
            }
        }

        YAML::Node current_node(node);
        for (auto &token: tokens) {
            if (token[0] == '[' && token.back() == ']') {
                CHECK(current_node.IsSequence()) << "Cannot search by index which is not a sequence" << std::endl;
                auto token_idx = std::stoi(token.substr(1, token.size() - 2));
                auto _node = (current_node)[token_idx];
                current_node.reset(_node);
                continue;
            }
            if (!current_node[token]) {
                CHECK (create_if_absent) << "The node does not contain the item at " << path << std::endl;
                current_node[token] = YAML::Node();
            }
            auto _node = current_node[token];
            current_node.reset(_node);
        }

        return current_node;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void SetNode(YAML::Node &base_node, const std::string &path, const YAML::Node &value_node) {
        auto find_node = FindNode(base_node, path, true);
        find_node = value_node;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void MergeOverridesNode(YAML::Node &base_node, YAML::Node &overrides_node) {
        for (auto pair: overrides_node) {
            auto first_node = pair.first;
            auto second_node = pair.second;
            auto tag = first_node.as<std::string>();
            if (tag != "overrides") {
                base_node[tag] = second_node;
            } else {
                CHECK(second_node.IsSequence()) <<
                                                "The `overrides` node in the yaml is not a sequence" << std::endl;
                for (size_t i(0); i < second_node.size(); ++i) {
                    auto child = second_node[i];
                    CHECK(child.IsMap() && child.size() == 1) << "Expects key value pairs for the overrides";

                    auto pair = child.begin();
                    auto full_path = pair->first.as<std::string>();
                    auto value = pair->second;
                    SetNode(base_node, full_path, value);
                }
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    YAML::Node ReadConfigFromArguments(int argc, char **argv, const std::string &description) {
        TCLAP::CmdLine cmd(description,
                           ' ', "v0.9");
        TCLAP::ValueArg<std::string> config_path("c", "config", "Path to the YAML config file on disk",
                                                 true, "", "string");
        TCLAP::MultiArg<std::string> overrides_path("o", "overrides_path",
                                                    "Path to the overrides value on disk",
                                                    false, "string");
        cmd.add(config_path);
        cmd.add(overrides_path);
        cmd.parse(argc, argv);

        std::string config_path_str = config_path.getValue();
        std::optional<std::vector<std::string>> overrides_path_str{};
        if (!overrides_path.getValue().empty())
            overrides_path_str.emplace(overrides_path.getValue());

        return RootNode(config_path_str, overrides_path_str);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    namespace {
        template<typename ValueT>
        YAML::Node ValueArrayToNode(const ValueT *arr, int num_values) {
            YAML::Node array;
            for (int i(0); i < num_values; ++i)
                array[i] = arr[i];
            return array;
        }

        template<typename ValueT>
        void ReadValueArrayFromNode(const YAML::Node &node,
                                    ValueT *arr, int num_values) {
            SLAM_CHECK_STREAM(node.IsSequence(), "The following node is not a sequence:\n" << node);
            SLAM_CHECK_STREAM(node.size() >= num_values, "Not enough values in the YAML::Node sequence");
            for (int i(0); i < num_values; ++i) {
                arr[i] = node[i].template as<ValueT>();
            }
        }
    } // namespace

    void WritePoseToNode(YAML::Node &node, const SE3 &pose) {
        node["quat"] = ValueArrayToNode(pose.quat.coeffs().data(), 4);
        node["tr"] = ValueArrayToNode(pose.tr.data(), 3);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    SE3 PoseFromNode(YAML::Node &node) {
        SE3 pose;
        auto quat_node = node["quat"];
        auto tr_node = node["tr"];
        ReadValueArrayFromNode(quat_node, pose.quat.coeffs().data(), 4);
        ReadValueArrayFromNode(tr_node, pose.tr.data(), 3);
        return pose;
    }

    /* -------------------------------------------------------------------------------------------------------------- */

}

