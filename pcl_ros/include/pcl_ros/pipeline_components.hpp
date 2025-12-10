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

#include "rclcpp/rclcpp.hpp"

#include "pcl_ros/pcl_node.hpp"
#include "pcl_ros/pcl_pipeline.hpp"

namespace pcl_ros
{
class PipelineFilter : public PCLAlgorithmNode<
    PCLPipeline, Input<pcl::PCLPointCloud2::Ptr>, Output<pcl::PCLPointCloud2::Ptr>>
{
public:
  explicit PipelineFilter(const rclcpp::NodeOptions & options)
  : PCLAlgorithmNode("PipelineFilterNode", options, std::vector<std::string>{"input"},
      std::vector<std::string>{"output"}) {}
};

class PipelineSegmentation : public PCLAlgorithmNode<
    PCLPipeline, Input<pcl::PCLPointCloud2::Ptr>,
    Output<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr>>
{
public:
  explicit PipelineSegmentation(const rclcpp::NodeOptions & options)
  : PCLAlgorithmNode("PipelineSegmentationNode", options, std::vector<std::string>{"input"},
      std::vector<std::string>{"indices", "model"}) {}
};

class PipelineExtraction : public PCLAlgorithmNode<
    PCLPipeline, Input<pcl::PCLPointCloud2::Ptr, pcl::PointIndices::Ptr>,
    Output<pcl::PointIndices::Ptr>>
{
public:
  explicit PipelineExtraction(const rclcpp::NodeOptions & options)
  : PCLAlgorithmNode("PipelineExtractionNode", options, std::vector<std::string>{"input",
        "indices"}, std::vector<std::string>{"output"}) {}
};

class PipelineMux : public PCLAlgorithmNode<
    PCLPipeline, Input<pcl::PCLPointCloud2::Ptr, pcl::PCLPointCloud2::Ptr>,
    Output<pcl::PointIndices::Ptr>>
{
public:
  explicit PipelineMux(const rclcpp::NodeOptions & options)
  : PCLAlgorithmNode("PipelineMuxNode", options, std::vector<std::string>{"input1", "input2"},
      std::vector<std::string>{"output"}) {}
};
}  // namespace pcl_ros
