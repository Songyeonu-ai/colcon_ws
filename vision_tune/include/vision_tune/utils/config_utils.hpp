#ifndef VISION_TUNE_UTILS_CONFIG_UTILS_HPP_
#define VISION_TUNE_UTILS_CONFIG_UTILS_HPP_

#include <yaml-cpp/yaml.h>

namespace vision_tune::utils
{

  struct hsv_range
  {
    int h_low = 0;
    int h_high = 0;
    int s_low = 0;
    int s_high = 0;
    int v_low = 0;
    int v_high = 0;
  };

  struct hsv_config
  {
    hsv_range red;
    hsv_range blue;
    hsv_range line;
  };

  inline YAML::Node to_yaml(const hsv_range &hsv)
  {
    YAML::Node node;
    node["h_low"] = hsv.h_low;
    node["h_high"] = hsv.h_high;
    node["s_low"] = hsv.s_low;
    node["s_high"] = hsv.s_high;
    node["v_low"] = hsv.v_low;
    node["v_high"] = hsv.v_high;
    return node;
  }

  inline bool from_yaml(const YAML::Node &node, hsv_range &hsv)
  {
    if (!node)
    {
      return false;
    }

    hsv.h_low = node["h_low"].as<int>();
    hsv.h_high = node["h_high"].as<int>();
    hsv.s_low = node["s_low"].as<int>();
    hsv.s_high = node["s_high"].as<int>();
    hsv.v_low = node["v_low"].as<int>();
    hsv.v_high = node["v_high"].as<int>();
    return true;
  }

} // namespace vision_tune::utils

#endif