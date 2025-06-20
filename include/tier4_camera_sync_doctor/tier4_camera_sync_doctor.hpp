// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TIER4_CAMERA_SYNC_DOCTOR__TIER4_CAMERA_SYNC_DOCTOR_HPP_
#define TIER4_CAMERA_SYNC_DOCTOR__TIER4_CAMERA_SYNC_DOCTOR_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <rcl_yaml_param_parser/parser.h>

#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <tuple>

namespace tier4_camera_sync_doctor
{
struct SyncStatus
{
  uint8_t level;
  int64_t diff_nsec;
};

enum class PairMatchResult { Match, CameraTimeOld, TriggerTimeOld };

class Tier4CameraSyncDoctor : public rclcpp::Node
{
public:
  explicit Tier4CameraSyncDoctor(const rclcpp::NodeOptions & options);

protected:
  std::optional<int64_t> timestamp_offset_;
  double frame_rate_;
  double phase_;
  std::optional<int> delay_ms_;
  int64_t trigger_time_tolerance_;
  int64_t camera_time_tolerance_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> camera_info_sub_;
  std::shared_ptr<rclcpp::Subscription<builtin_interfaces::msg::Time>> trigger_time_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::TimerBase::SharedPtr camera_info_qos_query_timer_;
  rclcpp::TimerBase::SharedPtr trigger_time_qos_query_timer_;

  std::queue<builtin_interfaces::msg::Time> camera_time_queue_;
  std::queue<builtin_interfaces::msg::Time> trigger_time_queue_;

  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
  std::shared_ptr<diagnostic_updater::FunctionDiagnosticTask> diag_task_;

  std::mutex camera_queue_mutex_;
  std::mutex trigger_queue_mutex_;

  template <typename T>
  std::optional<T> getYamlEntryValue(rcl_params_t * params_st, const std::string & entry_name);

  template <typename T>
  void checkParamFound(
    std::optional<T> & param, const std::string & file_path, const std::string & param_name);

  std::optional<int64_t> parseV4l2ParamFile(const std::string & path);
  std::tuple<double, double> parseTriggerParamFile(const std::string & path);
  std::optional<int> parseReadoutDelayParamFile(const std::string & path);

  template <typename MessageT, typename CallbackT>
  void qosQueryCallback(
    const std::string & topic_name, std::shared_ptr<rclcpp::Subscription<MessageT>> & sub,
    rclcpp::TimerBase::SharedPtr & timer, rclcpp::CallbackGroup::SharedPtr & callback_group,
    CallbackT && callback);

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ptr);
  void triggerTimeCallback(const builtin_interfaces::msg::Time::ConstSharedPtr & msg_ptr);

  int64_t getIdealCameraTimestampDiff();

  SyncStatus diagnoseTriggerTime(const builtin_interfaces::msg::Time & trigger_time);
  SyncStatus diagnoseCameraTime(
    const builtin_interfaces::msg::Time & camera_time,
    const builtin_interfaces::msg::Time & trigger_time);
  PairMatchResult matchPair(
    builtin_interfaces::msg::Time & camera_time, builtin_interfaces::msg::Time & trigger_time);
  void diagnoseSyncStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
};  // class Tier4CameraSyncDoctor
}  // namespace tier4_camera_sync_doctor

#endif  // TIER4_CAMERA_SYNC_DOCTOR__TIER4_CAMERA_SYNC_DOCTOR_HPP_
