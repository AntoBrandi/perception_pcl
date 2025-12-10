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

#ifndef PCL_ROS__PCL_ALGORITHM_HPP_
#define PCL_ROS__PCL_ALGORITHM_HPP_

#include <any>
#include <memory>
#include <vector>
#include <mutex>
#include <tuple>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "pcl/pcl_base.h"
#include "pcl/point_types.h"
#include "pcl_ros/conversion_traits.hpp"
#include "pcl_ros/pcl_node.hpp"


namespace pcl_ros
{
/**
 * @brief @b PCLAlgorithm represents the base PCL Algorithm class. All PCL algorithms should inherit from
 * this class.
 */
class PCLAlgorithm
{
public:
  using SharedPtr = std::shared_ptr<PCLAlgorithm>;

  /**
   * @brief Virtual destructor
   */
  virtual ~PCLAlgorithm() = default;

  /**
   * @brief Initialization method called after construction.
   * @param node A rclcpp::Node pointer.
   * @param name The name of this algorithm instance.
   */
  void initialize(rclcpp::Node * node, const std::string & name = "")
  {
    if (!node) {
      throw std::runtime_error{"Failed to lock node!"};
    }
    node_ = node;
    name_ = name;
    namespace_ = name_.empty() ? "" : name_ + ".";
    onInitialize();
  }

  /**
   * @brief Virtual abstract compute method. To be implemented by every child.
   * @param inputs the requested inputs dataset.
   * @param output the the resultant filtered outputs.
   */
  virtual void compute(const std::vector<std::any> & inputs, std::vector<std::any> & outputs) = 0;

  /**
   * @brief Callback for parameter changes.
   * Implement this function in your PCL Node.
   * @param params the vector of parameters that have changed.
   */
  virtual rcl_interfaces::msg::SetParametersResult onParamsChanged(
    const std::vector<rclcpp::Parameter> & params) = 0;

  /**
   * @brief Virtual abstract onInitialize method. To be implemented by every child.
   */
  virtual void onInitialize() = 0;

protected:
  /**
   * @brief The name of this algorithm instance.
   */
  std::string name_;

  /**
   * @brief The namespace for the parameters
   */
  std::string namespace_;

  /**
   * @brief A pointer to the node owning this algorithm.
   */
  rclcpp::Node * node_;

  /**
   * @brief Empty class constructor for plugin instantiation.
   */
  PCLAlgorithm() = default;
};

namespace internal
{
template<typename Tuple, std::size_t... I>
void tuple_to_any_vector_impl(const Tuple & t, std::vector<std::any> & v, std::index_sequence<I...>)
{
  (v.push_back(std::get<I>(t)), ...);
}

template<typename ... Args>
void tuple_to_any_vector(const std::tuple<Args...> & t, std::vector<std::any> & v)
{
  tuple_to_any_vector_impl(t, v, std::index_sequence_for<Args...>{});
}

template<typename Tuple, std::size_t... I>
void any_vector_to_tuple_impl(const std::vector<std::any> & v, Tuple & t, std::index_sequence<I...>)
{
  // Note: This assumes the algorithm pushed outputs in the exact order of PclOut types
  // and that the types match exactly.
  ((std::get<I>(t) = std::any_cast<typename std::tuple_element<I, Tuple>::type>(v[I])), ...);
}

template<typename ... Args>
void any_vector_to_tuple(const std::vector<std::any> & v, std::tuple<Args...> & t)
{
  if (v.size() != sizeof...(Args)) {
    throw std::runtime_error("PCLAlgorithm returned unexpected number of outputs.");
  }
  any_vector_to_tuple_impl(v, t, std::index_sequence_for<Args...>{});
}
}


template<typename AlgorithmT, typename InList, typename OutList>
class PCLAlgorithmNode;

template<typename AlgorithmT, typename ... PclIn, typename ... PclOut>
class PCLAlgorithmNode<AlgorithmT, Input<PclIn...>, Output<PclOut...>>
  : public PCLNode<Input<pcl_to_ros_t<PclIn>...>, Output<pcl_to_ros_t<PclOut>...>>
{
  // PCL-native tuples
  using PclInputsTuple = std::tuple<PclIn...>;
  using PclOutputsTuple = std::tuple<PclOut...>;

  // ROS types derived via traits
  using RosInputs = Input<pcl_to_ros_t<PclIn>...>;
  using RosOutputs = Output<pcl_to_ros_t<PclOut>...>;

public:
  explicit PCLAlgorithmNode(
    std::string node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::vector<std::string> input_topics = {},
    std::vector<std::string> output_topics = {})
  : PCLNode<RosInputs, RosOutputs>(node_name, options, input_topics, output_topics)
  {
    static_assert(
      std::is_base_of<PCLAlgorithm, AlgorithmT>::value,
      "AlgorithmT must inherit from pcl_ros::PCLAlgorithm");

    algorithm_ = std::make_unique<AlgorithmT>();
    algorithm_->initialize(this);
  }

  /**
   * @brief Virtual destructor
   */
  virtual ~PCLAlgorithmNode() = default;

  /**
   * @brief Virtual abstract compute method. To be implemented by every child.
   * @param inputs the requested inputs dataset.
   * @param output the the resultant filtered outputs.
   */
  void compute(const pcl_to_ros_t<PclIn> & ... inputs, pcl_to_ros_t<PclOut> & ... output) override
  {
    PclInputsTuple pcl_inputs_tuple;
    // Convert ROS inputs to PCL inputs (using existing utility from conversion_traits)
    tuple_convert_ros_to_pcl(std::tie(inputs ...), pcl_inputs_tuple);

    // Pack PCL Inputs into std::vector<std::any>
    std::vector<std::any> any_inputs;
    any_inputs.reserve(sizeof...(PclIn));
    internal::tuple_to_any_vector(pcl_inputs_tuple, any_inputs);

    // Call the generic algorithm
    std::vector<std::any> any_outputs;
    algorithm_->compute(any_inputs, any_outputs);

    // Convert std::vector<std::any> back to PCL Output Tuple
    PclOutputsTuple pcl_outputs_tuple;

    internal::any_vector_to_tuple(any_outputs, pcl_outputs_tuple);

    // Convert PCL outputs back to ROS outputs
    auto ros_outputs_tuple = std::tie(output ...);
    tuple_convert_pcl_to_ros(pcl_outputs_tuple, ros_outputs_tuple);
  }

  /**
   * @brief Implements the parameter changed callback.
   * This function is called by the PCLNode base when parameters are changed.
   * It delegates the call to the encapsulated PCLAlgorithm instance.
   * @param params The vector of parameters that have changed.
   */
  rcl_interfaces::msg::SetParametersResult onParamsChanged(
    const std::vector<rclcpp::Parameter> & params) override
  {
    // Delegate to the algorithm's implementation
    return algorithm_->onParamsChanged(params);
  }

protected:
  std::unique_ptr<AlgorithmT> algorithm_;
};

}  // namespace pcl_ros

#endif  // PCL_ROS__PCL_ALGORITHM_HPP_
