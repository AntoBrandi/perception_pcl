/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: extract_indices.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include "pcl_ros/filters/extract_indices.hpp"

namespace pcl_ros
{
void ExtractIndicesAlgorithm::onInitialize()
{
  rcl_interfaces::msg::ParameterDescriptor neg_desc;
  neg_desc.name = namespace_ + "negative";
  neg_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  neg_desc.description = "Extract indices or the negative (all-indices)";
  node_->declare_parameter(neg_desc.name, rclcpp::ParameterValue(false), neg_desc);
}

void ExtractIndicesAlgorithm::compute(
  const std::vector<std::any> & inputs, std::vector<std::any> & outputs)
{
  auto input = std::any_cast<pcl::PCLPointCloud2::Ptr>(inputs[0]);
  auto indices = std::any_cast<IndicesPtr>(inputs[1]);
  pcl::PCLPointCloud2::Ptr output(new pcl::PCLPointCloud2);
  if (!input->data.empty() && !indices->indices.empty()) {
    impl_.setInputCloud(input);
    impl_.setIndices(indices);
    impl_.filter(*output);
  } else {
    output = input;
  }
  outputs.push_back(output);
}

rcl_interfaces::msg::SetParametersResult ExtractIndicesAlgorithm::onParamsChanged(
  const std::vector<rclcpp::Parameter> & params)
{
  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == namespace_ + "negative") {
      // Check the current value for the negative flag
      if (impl_.getNegative() != param.as_bool()) {
        RCLCPP_DEBUG(
          node_->get_logger(), "Setting the filter negative flag to: %s.",
          param.as_bool() ? "true" : "false");
        // Call the virtual method in the child
        impl_.setNegative(param.as_bool());
      }
    }
  }

  // Range constraints are enforced by rclcpp::Parameter.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}
}  // namespace pcl_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::ExtractIndices)

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pcl_ros::ExtractIndicesAlgorithm, pcl_ros::PCLAlgorithm)
