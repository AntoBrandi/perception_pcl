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

#ifndef PCL_ROS__CONVERSION_TRAITS_HPP_
#define PCL_ROS__CONVERSION_TRAITS_HPP_

#include <tuple>
#include <utility>
#include <memory>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_msgs/msg/point_indices.hpp"
#include "pcl_msgs/msg/model_coefficients.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/pcl_base.h"
#include "pcl/PointIndices.h"
#include "pcl/ModelCoefficients.h"
#include "pcl_conversions/pcl_conversions.h"

namespace pcl_ros
{
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

using PointCloudRgb = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudRgbPtr = PointCloudRgb::Ptr;
using PointCloudRgbConstPtr = PointCloudRgb::ConstPtr;

using Indices = pcl::PointIndices;
using IndicesPtr = Indices::Ptr;
using IndicesConstPtr = Indices::ConstPtr;

using Coefficients = pcl::ModelCoefficients;
using CoefficientsPtr = Coefficients::Ptr;
using CoefficientsConstPtr = Coefficients::ConstPtr;

using PointCloud2 = sensor_msgs::msg::PointCloud2;

using PointIndices = pcl_msgs::msg::PointIndices;
using PointIndicesPtr = PointIndices::SharedPtr;
using PointIndicesConstPtr = PointIndices::ConstSharedPtr;

using ModelCoefficients = pcl_msgs::msg::ModelCoefficients;
using ModelCoefficientsPtr = ModelCoefficients::SharedPtr;
using ModelCoefficientsConstPtr = ModelCoefficients::ConstSharedPtr;

template<typename ... Ts>
struct Input {};

template<typename ... Ts>
struct Output {};

/**
 * @brief Primary, undefined template for RosToPclTrait.
 * This provides a clear "incomplete type" error if a
 * specialization is missing.
 */
template<typename RosMsgType>
struct RosToPclTrait;

/**
 * @brief Specialization for sensor_msgs::msg::PointCloud2
 */
template<>
struct RosToPclTrait<sensor_msgs::msg::PointCloud2>
{
  using PclType = PointCloudPtr;
};

/**
 * @brief Specialization for pcl_msgs::msg::PointIndices
 */
template<>
struct RosToPclTrait<pcl_msgs::msg::PointIndices>
{
  using PclType = IndicesPtr;
};

/**
 * @brief Specialization for pcl_msgs::msg::ModelCoefficients
 */
template<>
struct RosToPclTrait<pcl_msgs::msg::ModelCoefficients>
{
  using PclType = CoefficientsPtr;
};

/**
 * @brief Helper alias template for RosToPclTrait.
 */
template<typename RosMsgType>
using ros_to_pcl_t = typename RosToPclTrait<RosMsgType>::PclType;

/**
 * @brief Primary, undefined template for PclToRosTrait.
 */
template<typename PclType>
struct PclToRosTrait;

/**
 * @brief Specialization for PointCloudPtr
 */
template<>
struct PclToRosTrait<PointCloudPtr>
{
  using RosType = sensor_msgs::msg::PointCloud2;
};

/**
 * @brief Specialization for PointCloudPtr
 */
template<>
struct PclToRosTrait<PointCloudRgbPtr>
{
  using RosType = sensor_msgs::msg::PointCloud2;
};

/**
 * @brief Specialization for IndicesPtr
 */
template<>
struct PclToRosTrait<IndicesPtr>
{
  using RosType = pcl_msgs::msg::PointIndices;
};

/**
 * @brief Specialization for CoefficientsPtr
 */
template<>
struct PclToRosTrait<CoefficientsPtr>
{
  using RosType = pcl_msgs::msg::ModelCoefficients;
};

/**
 * @brief Specialization for pcl::PCLPointCloud2::Ptr
 */
template<>
struct PclToRosTrait<pcl::PCLPointCloud2::Ptr>
{
  using RosType = sensor_msgs::msg::PointCloud2;
};

/**
 * @brief Helper alias template for PclToRosTrait.
 */
template<typename PclType>
using pcl_to_ros_t = typename PclToRosTrait<PclType>::RosType;

inline void convert(const PointCloud2 & ros_msg, PointCloudPtr & pcl_obj)
{
  static_assert(
    std::is_same_v<ros_to_pcl_t<PointCloud2>, PointCloudPtr>,
    "Trait mismatch for PointCloud2 -> PointCloudPtr");
  static_assert(
    std::is_same_v<pcl_to_ros_t<PointCloudPtr>, PointCloud2>,
    "Trait mismatch for PointCloudPtr -> PointCloud2");

  pcl_obj.reset(new PointCloud);
  pcl::fromROSMsg(ros_msg, *pcl_obj);
}

inline void convert(const PointCloud2 & ros_msg, PointCloudRgbPtr & pcl_obj)
{
  static_assert(
    std::is_same_v<pcl_to_ros_t<PointCloudRgbPtr>, PointCloud2>,
    "Trait mismatch for PointCloudRgbPtr -> PointCloud2");

  pcl_obj.reset(new PointCloudRgb);
  pcl::fromROSMsg(ros_msg, *pcl_obj);
}

inline void convert(const PointCloud2 & ros_msg, pcl::PCLPointCloud2::Ptr & pcl_obj)
{
  static_assert(
    std::is_same_v<pcl_to_ros_t<pcl::PCLPointCloud2::Ptr>, PointCloud2>,
    "Trait mismatch for pcl::PCLPointCloud2::Ptr -> PointCloud2");

  pcl_obj.reset(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(ros_msg, *(pcl_obj));
}

inline void convert(const PointIndices & ros_msg, IndicesPtr & pcl_obj)
{
  static_assert(
    std::is_same_v<ros_to_pcl_t<PointIndices>, IndicesPtr>,
    "Trait mismatch for PointIndices -> IndicesPtr");
  static_assert(
    std::is_same_v<pcl_to_ros_t<IndicesPtr>, PointIndices>,
    "Trait mismatch for IndicesPtr -> PointIndices");

  pcl_obj.reset(new Indices);
  pcl_obj->indices = ros_msg.indices;
}

inline void convert(const ModelCoefficients & ros_msg, CoefficientsPtr & pcl_obj)
{
  static_assert(
    std::is_same_v<ros_to_pcl_t<ModelCoefficients>, CoefficientsPtr>,
    "Trait mismatch for ModelCoefficients -> CoefficientsPtr");
  static_assert(
    std::is_same_v<pcl_to_ros_t<CoefficientsPtr>, ModelCoefficients>,
    "Trait mismatch for CoefficientsPtr -> ModelCoefficients");

  pcl_obj.reset(new Coefficients);
  pcl_obj->values = ros_msg.values;
}

inline void convert(const pcl::PCLPointCloud2::Ptr & pcl_obj, PointCloud2 & ros_msg)
{
  static_assert(
    std::is_same_v<pcl_to_ros_t<pcl::PCLPointCloud2::Ptr>, PointCloud2>,
    "Trait mismatch for pcl::PCLPointCloud2::Ptr -> PointCloud2");

  pcl_conversions::moveFromPCL(*pcl_obj, ros_msg);
}

inline void convert(const PointCloudPtr & pcl_obj, PointCloud2 & ros_msg)
{
  static_assert(
    std::is_same_v<ros_to_pcl_t<PointCloud2>, PointCloudPtr>,
    "Trait mismatch for PointCloud2 -> PointCloudPtr");
  static_assert(
    std::is_same_v<pcl_to_ros_t<PointCloudPtr>, PointCloud2>,
    "Trait mismatch for PointCloudPtr -> PointCloud2");

  pcl::toROSMsg(*pcl_obj, ros_msg);
}

inline void convert(const PointCloudRgbPtr & pcl_obj, PointCloud2 & ros_msg)
{
  static_assert(
    std::is_same_v<pcl_to_ros_t<PointCloudRgbPtr>, PointCloud2>,
    "Trait mismatch for PointCloudRgbPtr -> PointCloud2");

  pcl::toROSMsg(*pcl_obj, ros_msg);
}

inline void convert(const IndicesPtr & pcl_obj, PointIndices & ros_msg)
{
  static_assert(
    std::is_same_v<ros_to_pcl_t<PointIndices>, IndicesPtr>,
    "Trait mismatch for PointIndices -> IndicesPtr");
  static_assert(
    std::is_same_v<pcl_to_ros_t<IndicesPtr>, PointIndices>,
    "Trait mismatch for IndicesPtr -> PointIndices");

  ros_msg.indices = pcl_obj->indices;
}

inline void convert(const CoefficientsPtr & pcl_obj, ModelCoefficients & ros_msg)
{
  static_assert(
    std::is_same_v<ros_to_pcl_t<ModelCoefficients>, CoefficientsPtr>,
    "Trait mismatch for ModelCoefficients -> CoefficientsPtr");
  static_assert(
    std::is_same_v<pcl_to_ros_t<CoefficientsPtr>, ModelCoefficients>,
    "Trait mismatch for CoefficientsPtr -> ModelCoefficients");

  ros_msg.values = pcl_obj->values;
}

/**
 * @brief Implementation helper for ROS-to-PCL tuple conversion.
 */
template<typename RosTuple, typename PclTuple, std::size_t... I>
void tuple_convert_ros_to_pcl_impl(
  const RosTuple & ros_tuple, PclTuple & pcl_tuple, std::index_sequence<I...>)
{
  ( ( convert(std::get<I>(ros_tuple), std::get<I>(pcl_tuple)) ), ... );
}

/**
 * @brief Convert a std::tuple of ROS messages to a std::tuple of PCL objects.
 */
template<typename RosTuple, typename PclTuple>
void tuple_convert_ros_to_pcl(const RosTuple & ros_tuple, PclTuple & pcl_tuple)
{
  tuple_convert_ros_to_pcl_impl(
    ros_tuple, pcl_tuple,
    std::make_index_sequence<std::tuple_size<RosTuple>::value>{});
}

/**
 * @brief Implementation helper for PCL-to-ROS tuple conversion.
 */
template<typename PclTuple, typename RosTuple, std::size_t... J>
void tuple_convert_pcl_to_ros_impl(
  const PclTuple & pcl_tuple, RosTuple & ros_tuple, std::index_sequence<J...>)
{
  ( ( convert(std::get<J>(pcl_tuple), std::get<J>(ros_tuple)) ), ... );
}

/**
 * @brief Convert a std::tuple of PCL objects to a std::tuple of ROS messages.
 */
template<typename PclTuple, typename RosTuple>
void tuple_convert_pcl_to_ros(const PclTuple & pcl_tuple, RosTuple & ros_tuple)
{
  tuple_convert_pcl_to_ros_impl(
    pcl_tuple, ros_tuple,
    std::make_index_sequence<std::tuple_size<PclTuple>::value>{});
}

template<typename T>
struct is_const_shared_ptr : std::false_type {};

template<typename T>
struct is_const_shared_ptr<std::shared_ptr<const T>>: std::true_type {};

template<typename T>
static constexpr bool is_const_shared_ptr_v = is_const_shared_ptr<T>::value;
}  // namespace pcl_ros

#endif  // PCL_ROS__CONVERSION_TRAITS_HPP_
