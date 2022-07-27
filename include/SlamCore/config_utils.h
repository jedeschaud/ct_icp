#ifndef SlamCore_HAML_UTILS_H
#define SlamCore_HAML_UTILS_H

#include <yaml-cpp/yaml.h>
#include <glog/logging.h>

#include "SlamCore/types.h"

namespace slam::config {


    /* A Macro which finds an option in a YAML node */
#define FIND_OPTION(node_name, option_name, param_name, type)             \
    if(node_name[#param_name]) {                                            \
        option_name . param_name = node_name [ #param_name ] . as < type >();\
    }

#define FIND_REQUIRED_OPTION(node_name, option_name, param_name, type) \
    CHECK (node_name[#param_name]) << "The parameter " << #param_name << " not found in the YAML node" << std::endl; \
    FIND_OPTION(node_name, option_name, param_name, type)

    void FindEnumOption(const YAML::Node &node, int &option_enum,
                        std::string option_name,
                        std::initializer_list<std::pair<std::string, int>> options);

#define FIND_ENUM_OPTION(node_name, options_name, param_name, list_candidates)   \
    if (node_name [ #param_name ]) {                                              \
        slam::config::FindEnumOption( node_name, (int&) options_name . param_name, #param_name, list_candidates);\
    }

    /*!
     * @brief   Returns the root node of a YAML file
     *
     * @param overrides_path The optional path on disk of an `overrides` YAML which will be merged to the root config
     */
    YAML::Node RootNode(const std::string &path,
                        const std::optional<std::vector<std::string>> &overrides_path = {});

    /*!
     * @brief   Reads config file from arguments
     */
    YAML::Node ReadConfigFromArguments(int argc, char **argv, const std::string &command_description);

    /*!
     * @brief   Returns the node given an extended path in the yaml
     *
     * The path allows to access child nodes at multiple level of recursions
     * Tokens represent children keys:
     *
     * FindNode(node, "child1.child2.[0].value")
     *
     * Is equivalent to:
     *
     * node["child1"]["child2"][0]["value"]
     *
     */
    YAML::Node FindNode(YAML::Node node,
                        const std::string &path,
                        bool create_if_absent = false);


    /*!
     * @brief   Sets the value of a node at an extended path
     */
    void SetNode(YAML::Node &base_node, const std::string &path,
                 const YAML::Node &value_node);

    /*!
     * @brief   Merges a base node with an `overrides` node
     *
     * It replaces values in the base node by the override values defined in the `overrides_node`
     */
    void MergeOverridesNode(YAML::Node &base_node,
                            YAML::Node &overrides_node);

    /*!
     * @brief Reads a pose from a YAML::Node
     */
    SE3 PoseFromNode(YAML::Node &node);

    /*!
     * @brief Writes a pose in a yaml node
     */
    void WritePoseToNode(YAML::Node &node, const slam::SE3 &pose);

}

#endif //SlamCore_HAML_UTILS_H
