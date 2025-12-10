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
 * $Id: radius_outlier_removal.h 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#ifndef PCL_ROS__FILTERS__RADIUS_OUTLIER_REMOVAL_HPP_
#define PCL_ROS__FILTERS__RADIUS_OUTLIER_REMOVAL_HPP_

// PCL includes
#include <pcl/filters/radius_outlier_removal.h>
#include <vector>
#include "pcl_ros/pcl_algorithm.hpp"

namespace pcl_ros
{
/** \brief @b RadiusOutlierRemoval is a simple filter that removes outliers if the number of neighbors in a certain
  * search radius is smaller than a given K.
  * \note setFilterFieldName (), setFilterLimits (), and setFilterLimitNegative () are ignored.
  * \author Radu Bogdan Rusu
  * \author Antonio Brandi
  */
class RadiusOutlierRemovalAlgorithm : public PCLAlgorithm
{
public:
  using Ptr = std::shared_ptr<RadiusOutlierRemovalAlgorithm>;
  using UniquePtr = std::unique_ptr<RadiusOutlierRemovalAlgorithm>;

  /**
   * @brief Constructor
   */
  RadiusOutlierRemovalAlgorithm() = default;

  /**
   * @brief Initialization method called after construction.
   */
  void onInitialize() override;

  /**
   * @brief Calls the actual RadiusOutlierRemoval PCL filter.
   * @param input the input point cloud dataset.
   * @param output the resultant filtered dataset.
   */
  void compute(
    const std::vector<std::any> & inputs, std::vector<std::any> & outputs) override;

  /**
   * @brief Parameter callback
   * @param params parameter values to set.
   * @return The result of setting the parameters.
   */
  rcl_interfaces::msg::SetParametersResult onParamsChanged(
    const std::vector<rclcpp::Parameter> & params) override;

private:
  /**
   * @brief Tolerance for comparing floating point parameters (e.g., radius_search).
   */
  static constexpr double PARAMETER_TOLERANCE = 1e-6;

  /**
   * @brief The PCL filter implementation used.
   */
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> impl_;
};

class RadiusOutlierRemoval : public PCLAlgorithmNode<RadiusOutlierRemovalAlgorithm,
    Input<pcl::PCLPointCloud2::Ptr>,
    Output<pcl::PCLPointCloud2::Ptr>>
{
public:
  /**
   * @brief Constructor
   * @param options A rclcpp::NodeOptions to be passed to the node.
   */
  explicit RadiusOutlierRemoval(const rclcpp::NodeOptions & options)
  : PCLAlgorithmNode("RadiusOutlierRemovalNode", options, std::vector<std::string>{"input"},
      std::vector<std::string>{"output"}) {}
};
}  // namespace pcl_ros

#endif  // PCL_ROS__FILTERS__RADIUS_OUTLIER_REMOVAL_HPP_
