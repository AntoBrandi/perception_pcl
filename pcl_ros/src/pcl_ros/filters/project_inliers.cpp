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
 * $Id: project_inliers.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include "pcl_ros/filters/project_inliers.hpp"

namespace pcl_ros
{
void ProjectInliersAlgorithm::onInitialize()
{
  rcl_interfaces::msg::ParameterDescriptor model_type_desc;
  model_type_desc.name = namespace_ + "model_type";
  model_type_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  model_type_desc.description =
    "The type of model to use for segmentation.";
  {
    rcl_interfaces::msg::IntegerRange int_range;
    int_range.from_value = 0;
    int_range.to_value = 17;
    model_type_desc.integer_range.push_back(int_range);
  }
  // Required Parameter - NO Default Value
  node_->declare_parameter(model_type_desc.name, model_type_desc.type);

  rcl_interfaces::msg::ParameterDescriptor copy_all_data_desc;
  copy_all_data_desc.name = namespace_ + "copy_all_data";
  copy_all_data_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  copy_all_data_desc.description =
    "Whether all data will be returned, or only the projected inliers."
    "true if all data should be returned, false if only the projected inliers";
  // Optional Parameter - Default Value: false
  node_->declare_parameter(
    copy_all_data_desc.name, rclcpp::ParameterValue(false), copy_all_data_desc);
}

void ProjectInliersAlgorithm::compute(
  const std::vector<std::any> & inputs, std::vector<std::any> & outputs)
{
  auto input = std::any_cast<const pcl::PCLPointCloud2::Ptr &>(inputs[0]);
  auto indices = std::any_cast<const IndicesPtr &>(inputs[1]);
  auto model = std::any_cast<const CoefficientsPtr &>(inputs[2]);
  pcl::PCLPointCloud2::Ptr output(new pcl::PCLPointCloud2);
  impl_.setInputCloud(input);
  impl_.setIndices(indices);
  impl_.setModelCoefficients(model);
  impl_.filter(*output);
  outputs.push_back(output);
}

rcl_interfaces::msg::SetParametersResult ProjectInliersAlgorithm::onParamsChanged(
  const std::vector<rclcpp::Parameter> & params)
{
  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == namespace_ + "model_type") {
      if (impl_.getModelType() != param.as_int()) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "Setting the model type to: %ld.",
          param.as_int());
        impl_.setModelType(param.as_int());
      }
    }
    if (param.get_name() == namespace_ + "copy_all_data") {
      if (impl_.getCopyAllData() != param.as_bool()) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "Setting copy all data to: %s.",
          (param.as_bool() ? "true" : "false"));
        impl_.setCopyAllData(param.as_bool());
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
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::ProjectInliers)

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pcl_ros::ProjectInliersAlgorithm, pcl_ros::PCLAlgorithm)
