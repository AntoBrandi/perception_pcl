/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**

\author Antonio Brandi

**/

#include "pcl_ros/pcl_pipeline.hpp"

namespace pcl_ros
{
PCLPipeline::PCLPipeline()
{
  // Initialize pluginlib class loader
  try {
    loader_ = std::make_shared<pluginlib::ClassLoader<pcl_ros::PCLAlgorithm>>(
      "pcl_ros", "pcl_ros::PCLAlgorithm");
  } catch (const std::exception & e) {
    throw;
  }
}

void PCLPipeline::onInitialize()
{
  // Configure Interface Mapping (External <-> Internal)
  rcl_interfaces::msg::ParameterDescriptor in_desc, out_desc;
  in_desc.description = "Maps the input vector indices to named internal channels.";
  out_desc.description = "Maps named internal channels to the output vector indices.";

  node_->declare_parameter(namespace_ + "input_keys", std::vector<std::string>{}, in_desc);
  node_->declare_parameter(namespace_ + "output_keys", std::vector<std::string>{}, out_desc);

  input_keys_ = node_->get_parameter(namespace_ + "input_keys").as_string_array();
  output_keys_ = node_->get_parameter(namespace_ + "output_keys").as_string_array();
  // Get list of plugins to load
  node_->declare_parameter(namespace_ + "plugins", std::vector<std::string>{});
  std::vector<std::string> plugin_names =
    node_->get_parameter(namespace_ + "plugins").as_string_array();

  // Load plugins
  for (const auto & name : plugin_names) {
    AlgorithmStep step;
    step.instance_name = name;

    // Construct full parameter paths
    std::string step_ns = namespace_ + name + ".";

    if (!node_->has_parameter(step_ns + "plugin")) {
      node_->declare_parameter(step_ns + "plugin", std::string());
    }
    if (!node_->has_parameter(step_ns + "inputs")) {
      node_->declare_parameter(step_ns + "inputs", std::vector<std::string>());
    }
    if (!node_->has_parameter(step_ns + "outputs")) {
      node_->declare_parameter(step_ns + "outputs", std::vector<std::string>());
    }

    step.plugin_type = node_->get_parameter(step_ns + "plugin").as_string();
    step.input_channels = node_->get_parameter(step_ns + "inputs").as_string_array();
    step.output_channels = node_->get_parameter(step_ns + "outputs").as_string_array();

    if (step.plugin_type.empty()) {
      RCLCPP_ERROR(
        node_->get_logger(), "[%s] Step '%s' missing 'plugin' param.",
        name_.c_str(), name.c_str());
      continue;
    }

    try {
      step.instance = loader_->createUniqueInstance(step.plugin_type);
      pipeline_steps_.push_back(step);
      RCLCPP_INFO(
        node_->get_logger(), "[%s] Loaded step '%s' (%s)",
        name_.c_str(), name.c_str(), step.plugin_type.c_str());
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        node_->get_logger(), "[%s] Failed to load '%s': %s",
        name_.c_str(), name.c_str(), ex.what());
    }
  }

  // Initialize plugins
  for (auto & step : pipeline_steps_) {
    step.instance->initialize(node_, step.instance_name);
  }
}

void PCLPipeline::compute(const std::vector<std::any> & inputs, std::vector<std::any> & outputs)
{
  // Map Inputs to Registry
  if (inputs.size() != input_keys_.size()) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "[%s] Input size mismatch. Received %zu, expected %zu (based on input_keys).",
      name_.c_str(), inputs.size(), input_keys_.size());
    return;
  }

  size_t map_count = std::min(inputs.size(), input_keys_.size());
  for (size_t i = 0; i < map_count; ++i) {
    channel_registry_[input_keys_[i]] = inputs[i];
  }

  // Execute Pipeline Steps
  for (auto & step : pipeline_steps_) {
    std::vector<std::any> step_inputs;
    step_inputs.reserve(step.input_channels.size());

    // Gather inputs from registry
    bool missing_input = false;
    for (const auto & channel_key : step.input_channels) {
      if (channel_registry_.find(channel_key) == channel_registry_.end()) {
        RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[%s] Step '%s' missing input channel '%s'.",
          name_.c_str(), step.instance_name.c_str(), channel_key.c_str());
        missing_input = true;
        break;
      }
      step_inputs.push_back(channel_registry_.at(channel_key));
    }
    if (missing_input) {
      continue;
    }
    std::vector<std::any> step_outputs;
    try {
      step.instance->compute(step_inputs, step_outputs);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[%s] Step '%s' crashed: %s",
        name_.c_str(), step.instance_name.c_str(), e.what());
      continue;
    }

    // Distribute outputs to registry
    for (size_t i = 0; i < std::min(step_outputs.size(), step.output_channels.size()); ++i) {
      channel_registry_[step.output_channels[i]] = step_outputs[i];
    }
  }

  // Extract Outputs
  for (const auto & out_key : output_keys_) {
    if (channel_registry_.find(out_key) != channel_registry_.end()) {
      outputs.push_back(channel_registry_.at(out_key));
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "[%s] Output channel '%s' not found in registry.", name_.c_str(), out_key.c_str());
      outputs.push_back(std::any());
    }
  }

  channel_registry_.clear();
}

rcl_interfaces::msg::SetParametersResult PCLPipeline::onParamsChanged(
  const std::vector<rclcpp::Parameter> & params)
{
  for (auto & step : pipeline_steps_) {
    step.instance->onParamsChanged(params);
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}
}  // namespace pcl_ros

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pcl_ros::PCLPipeline, pcl_ros::PCLAlgorithm)
