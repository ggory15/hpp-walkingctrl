#pragma once
#include <yaml-cpp/yaml.h>

namespace hpp{
    namespace walkingctrl{




            template <typename YamlType>
            static YamlType readParameter(const YAML::Node& node, const std::string& name) {
              try { return node[name.c_str()].as<YamlType>(); }
              catch (...) { throw std::runtime_error(name); }
            }

            template <typename YamlType>
            void readParameter(const YAML::Node& node, const std::string& name, YamlType& parameter) {
              try { parameter = readParameter<YamlType>(node, name); }
              catch (...) { throw std::runtime_error(name); }
      }
    }
}
