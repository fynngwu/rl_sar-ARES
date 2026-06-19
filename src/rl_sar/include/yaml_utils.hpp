#pragma once

#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

namespace yaml_utils {

// Load a YAML node as either a scalar (replicated to N elements) or a sequence
inline std::vector<float> LoadScalarOrArray(const YAML::Node& node, size_t n)
{
    if (node.IsSequence()) {
        std::vector<float> v;
        for (const auto& e : node)
            v.push_back(e.as<float>());
        return v;
    }
    return std::vector<float>(n, node.as<float>());
}

// Load a YAML node as a vector of floats
inline std::vector<float> LoadFloatArray(const YAML::Node& node)
{
    std::vector<float> v;
    if (node) {
        for (const auto& e : node)
            v.push_back(e.as<float>());
    }
    return v;
}

// Load a YAML node as a vector of ints
inline std::vector<int> LoadIntArray(const YAML::Node& node)
{
    std::vector<int> v;
    if (node) {
        for (const auto& e : node)
            v.push_back(e.as<int>());
    }
    return v;
}

// Load a YAML node as a vector of strings
inline std::vector<std::string> LoadStringArray(const YAML::Node& node)
{
    std::vector<std::string> v;
    if (node) {
        for (const auto& e : node)
            v.push_back(e.as<std::string>());
    }
    return v;
}

} // namespace yaml_utils
