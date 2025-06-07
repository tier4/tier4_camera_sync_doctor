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

#include "tier4_camera_sync_doctor/tier4_camera_sync_doctor.hpp"

#include <diagnostic_updater/update_functions.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <rcl_yaml_param_parser/parser.h>
#include <rcutils/allocator.h>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>

namespace tier4_camera_sync_doctor
{
namespace
{
template <typename T>
constexpr bool false_v = false;

static std::string stringize_level(const uint8_t level)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  switch (level) {
    case DiagnosticStatus::OK:
      return "OK";
    case DiagnosticStatus::WARN:
      return "WARN";
    case DiagnosticStatus::ERROR:
      return "ERROR";
    case DiagnosticStatus::STALE:
      return "STALE";
    default:
      return "UNDEFINED";
  }
}

}  // namespace

Tier4CameraSyncDoctor::Tier4CameraSyncDoctor(const rclcpp::NodeOptions & options)
: Node("tier4_camera_sync_doctor", options),
  trigger_time_tolerance_(0.0),
  camera_time_tolerance_(0.0)
{
  auto v4l2_param_path = this->declare_parameter<std::string>("v4l2_param_path", "");
  auto trigger_param_path = this->declare_parameter<std::string>("trigger_param_path", "");
  auto readout_delay_param_path =
    this->declare_parameter<std::string>("readout_delay_param_path", "");

  trigger_time_tolerance_ =
    this->declare_parameter<int64_t>("trigger_time_tolerance_ns", 1e6);  // 1 ms
  camera_time_tolerance_ =
    this->declare_parameter<int64_t>("camera_time_tolerance_ns", 1e7);  // 10 ms

  auto hardware_id = this->declare_parameter<std::string>("hardware_id", "");

  // confirm the specified paths exist
  auto path_check = [this](const std::string & path, const std::string & param_name) {
    if (!std::filesystem::exists(path)) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(), "Specified `" << param_name << "`(" << path << ") does not exist");
      throw std::invalid_argument("Specified parameter path does not exist");
    }
  };

  path_check(v4l2_param_path, "v4l2_param_path");
  path_check(trigger_param_path, "trigger_param_path");
  // readout_delay_param_path may not exist depending on camera type

  // parse each file to find target parameters
  timestamp_offset_ = parseV4l2ParamFile(v4l2_param_path);
  std::tie(frame_rate_, phase_) = parseTriggerParamFile(trigger_param_path);
  delay_ms_ = parseReadoutDelayParamFile(readout_delay_param_path);

  // create subscribers for input topics
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  camera_info_qos_query_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
    this->qosQueryCallback(
      "~/input/camera_info", camera_info_sub_, camera_info_qos_query_timer_, callback_group_,
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg) {
        this->cameraInfoCallback(msg);
      });
  });

  trigger_time_qos_query_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
    this->qosQueryCallback(
      "~/input/trigger_time", trigger_time_sub_, trigger_time_qos_query_timer_, callback_group_,
      [this](const builtin_interfaces::msg::Time::ConstSharedPtr & msg) {
        this->triggerTimeCallback(msg);
      });
  });

  // Register diagnose tasks on updater
  diag_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
  diag_updater_->setHardwareID(hardware_id.empty() ? "none" : hardware_id);
  diag_updater_->setPeriod(1. / frame_rate_);  // align to the sensor rate

  diag_task_ = std::make_shared<diagnostic_updater::FunctionDiagnosticTask>(
    hardware_id + "_sync_status",
    std::bind(&Tier4CameraSyncDoctor::diagnoseSyncStatus, this, std::placeholders::_1));

  diag_updater_->add(*diag_task_);
}

template <typename T>
std::optional<T> Tier4CameraSyncDoctor::getYamlEntryValue(
  rcl_params_t * params_st, const std::string & entry_name)
{
  // XXX: here assumes the target parameter is declared as a global parameter
  // using wildcard nodename specification
  auto param = rcl_yaml_node_struct_get("/**", entry_name.c_str(), params_st);
  if (param == NULL) {
    return std::nullopt;
  }

  if constexpr (std::is_same_v<T, bool>) {
    if (param->bool_value == nullptr) {
      return std::nullopt;
    } else {
      return *(param->bool_value);
    }
  } else if constexpr (std::is_same_v<T, int64_t> || std::is_same_v<T, int>) {
    if (param->integer_value == nullptr) {
      return std::nullopt;
    } else {
      return *(param->integer_value);
    }
  } else if constexpr (std::is_same_v<T, double>) {
    if (param->double_value == nullptr) {
      return std::nullopt;
    } else {
      return *(param->double_value);
    }
  } else if constexpr (std::is_same_v<T, std::string>) {
    if (param->string_value == nullptr) {
      return std::nullopt;
    } else {
      return std::string(param->string_value);
    }
  } else {
    static_assert(false_v<T>, "Unsupported type specified");
  }
}

template <typename T>
void Tier4CameraSyncDoctor::checkParamFound(
  std::optional<T> & param, const std::string & file_path, const std::string & param_name)
{
  if (!param) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Valid parameter `" << param_name << "` was not found in: `" << file_path);
    throw std::runtime_error("Mandatory parameter was not found");
  }
}

std::optional<int64_t> Tier4CameraSyncDoctor::parseV4l2ParamFile(const std::string & path)
{
  auto rcl_param = rcl_yaml_node_struct_init(rcutils_get_default_allocator());
  if (!rcl_parse_yaml_file(path.c_str(), rcl_param)) {
    throw std::runtime_error("Failed to parse v4l2_param_file");
  }

  // Because this value is not mandatory, return std::optional directly
  auto param = getYamlEntryValue<int64_t>(rcl_param, "timestamp_offset");

  rcl_yaml_node_struct_fini(rcl_param);
  return param;
}

std::tuple<double, double> Tier4CameraSyncDoctor::parseTriggerParamFile(const std::string & path)
{
  auto rcl_param = rcl_yaml_node_struct_init(rcutils_get_default_allocator());
  if (!rcl_parse_yaml_file(path.c_str(), rcl_param)) {
    throw std::runtime_error("Failed to parse trigger_param_file");
  }

  std::string key = "frame_rate";
  auto frame_rate = getYamlEntryValue<double>(rcl_param, key);
  checkParamFound(frame_rate, path, key);

  key = "phase";
  auto phase = getYamlEntryValue<double>(rcl_param, key);
  checkParamFound(phase, path, key);

  rcl_yaml_node_struct_fini(rcl_param);

  return {frame_rate.value(), phase.value()};
}

std::optional<int> Tier4CameraSyncDoctor::parseReadoutDelayParamFile(const std::string & path)
{
  auto rcl_param = rcl_yaml_node_struct_init(rcutils_get_default_allocator());
  if (!rcl_parse_yaml_file(path.c_str(), rcl_param)) {
    return std::nullopt;
  }

  std::string key = "delay_ms";
  auto delay_ms = getYamlEntryValue<int>(rcl_param, key);
  if (!delay_ms) {
    key = "readout_delay_ms";
    delay_ms = getYamlEntryValue<int>(rcl_param, key);
  }
  checkParamFound(delay_ms, path, key);

  return delay_ms.value();
}

template <typename MessageT, typename CallbackT>
void Tier4CameraSyncDoctor::qosQueryCallback(
  const std::string & topic_name, std::shared_ptr<rclcpp::Subscription<MessageT>> & sub,
  rclcpp::TimerBase::SharedPtr & timer, rclcpp::CallbackGroup::SharedPtr & callback_group,
  CallbackT && callback)
{
  // Query QoS to publisher to align the QoS for the topics to be published
  auto get_qos = [this](std::string & topic_name, rclcpp::QoS & qos) -> bool {
    auto qos_list = this->get_publishers_info_by_topic(topic_name);
    if (qos_list.size() < 1) {
      RCLCPP_INFO_STREAM_THROTTLE(
        this->get_logger(), *(this->get_clock()), 500 /*[ms]*/,
        "Waiting for " << topic_name << " ...");
      return false;
    } else if (qos_list.size() > 1) {
      RCLCPP_ERROR(
        this->get_logger(), "Multiple publisher for %s are detected. Cannot determine proper QoS",
        topic_name.c_str());
      return false;
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(), "QoS for " << topic_name << " is acquired.");
      qos = qos_list[0].qos_profile();
      return true;
    }
  };

  std::string full_topic_name =
    this->get_node_topics_interface()->resolve_topic_name(topic_name, false);

  // Query QoS to publisher to align the QoS for the topics to be published
  rclcpp::QoS topic_qos(1);
  if (!get_qos(full_topic_name, topic_qos)) {
    // Publisher is not ready yet
    return;
  }

  RCLCPP_INFO_STREAM_ONCE(
    this->get_logger(), full_topic_name << " QoS is identified. Start Subscription");

  // Once the topic QoS is determined, start subscription using that QoS and stop query timer
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;
  sub = this->create_subscription<MessageT>(
    full_topic_name, topic_qos, std::forward<CallbackT>(callback), options);
  timer->cancel();
}

void Tier4CameraSyncDoctor::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ptr)
{
  builtin_interfaces::msg::Time camera_time = msg_ptr->header.stamp;
  {
    // Enqueue message
    std::lock_guard<std::mutex> queue_lock(camera_queue_mutex_);
    camera_time_queue_.push(camera_time);
  }
}

void Tier4CameraSyncDoctor::triggerTimeCallback(
  const builtin_interfaces::msg::Time::ConstSharedPtr & msg_ptr)
{
  builtin_interfaces::msg::Time trigger_time = *msg_ptr;
  {
    // Enqueue message
    std::lock_guard<std::mutex> queue_lock(trigger_queue_mutex_);
    trigger_time_queue_.push(trigger_time);
  }
}

SyncStatus Tier4CameraSyncDoctor::diagnoseTriggerTime(
  const builtin_interfaces::msg::Time & trigger_time)
{
  // check whether trigger time follows configuration
  //// the following implementation based on
  ////  https://github.com/tier4/sensor_trigger/blob/galactic/src/sensor_trigger.cpp
  int64_t observation_in_nsec =
    static_cast<int64_t>(trigger_time.sec * 1e9) + static_cast<int64_t>(trigger_time.nanosec);
  int64_t interval_nsec = static_cast<int64_t>(1e9 / frame_rate_);
  int64_t offset_nsec;
  if (std::abs(phase_) <= std::numeric_limits<double>::epsilon() * 10) {
    offset_nsec = 0;
  } else {
    offset_nsec = interval_nsec * static_cast<int64_t>(phase_ * 10) / 3600;
  }

  int64_t n = static_cast<int64_t>(std::floor((observation_in_nsec - offset_nsec) / interval_nsec));
  int64_t ideal_time = n * interval_nsec + offset_nsec;
  int64_t diff = std::abs(observation_in_nsec - ideal_time);

  SyncStatus trigger_sync_status;
  trigger_sync_status.level = (diff < trigger_time_tolerance_)
                                ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  trigger_sync_status.diff_nsec = diff;

  return trigger_sync_status;
}

SyncStatus Tier4CameraSyncDoctor::diagnoseCameraTime(
  const builtin_interfaces::msg::Time & camera_time,
  const builtin_interfaces::msg::Time & trigger_time)
{
  auto get_timestamp_nsec = [](const auto & time) -> int64_t {
    return static_cast<int64_t>(time.sec * 1e9) + static_cast<int64_t>(time.nanosec);
  };

  int64_t trigger_time_nsec = get_timestamp_nsec(trigger_time);
  int64_t camera_time_nsec = get_timestamp_nsec(camera_time) - timestamp_offset_.value_or(0);
  int64_t timestamp_diff = camera_time_nsec - trigger_time_nsec;  // take signed diff

  SyncStatus camera_sync_status;
  camera_sync_status.level = (timestamp_diff < camera_time_tolerance_)
                               ? diagnostic_msgs::msg::DiagnosticStatus::OK
                               : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  camera_sync_status.diff_nsec = timestamp_diff;
  return camera_sync_status;
}

PairMatchResult Tier4CameraSyncDoctor::matchPair(
  builtin_interfaces::msg::Time & camera_time, builtin_interfaces::msg::Time & trigger_time)
{
  auto get_timestamp_nsec = [](const auto & time) -> int64_t {
    return static_cast<int64_t>(time.sec * 1e9) + static_cast<int64_t>(time.nanosec);
  };

  int64_t trigger_time_nsec = get_timestamp_nsec(trigger_time);
  int64_t camera_time_nsec = get_timestamp_nsec(camera_time) - timestamp_offset_.value_or(0);
  int64_t timestamp_diff = camera_time_nsec - trigger_time_nsec;  // take signed diff

  // TODO(manato): modify here to accept diffent ideal_diff because ideal_diff depends on the camera
  // type to be used. Especially in case of C1, ideal_diff is timestamp_offset_.value (i.e.,
  // exposure length)
  int64_t ideal_diff = static_cast<int64_t>(delay_ms_.value_or(0));
  int64_t interval_nsec = static_cast<int64_t>(1e9 / frame_rate_);

  // check difference between trigger time and camera info header time.
  // the following criteria assumes camera timestamp jitter never exceeds half the interval
  if (timestamp_diff < ideal_diff - interval_nsec / 2) {
    // camera timestamp is too old. drop this camera data
    return PairMatchResult::CameraTimeOld;
  } else if (ideal_diff + interval_nsec / 2 < timestamp_diff) {
    // trigger data is too old. drop trigger data
    return PairMatchResult::TriggerTimeOld;
  } else {
    return PairMatchResult::Match;
  }
}

void Tier4CameraSyncDoctor::diagnoseSyncStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  builtin_interfaces::msg::Time camera_time;
  bool is_camera_queue_empty;
  {
    std::lock_guard<std::mutex> queue_lock(camera_queue_mutex_);
    is_camera_queue_empty = camera_time_queue_.empty();
    if (is_camera_queue_empty) {
      return;
    }
    camera_time = camera_time_queue_.front();
  }

  builtin_interfaces::msg::Time trigger_time;
  bool is_trigger_queue_empty;
  {
    std::lock_guard<std::mutex> queue_lock(trigger_queue_mutex_);
    is_trigger_queue_empty = trigger_time_queue_.empty();
    if (is_trigger_queue_empty) {
      return;
    }
    trigger_time = trigger_time_queue_.front();
  }

  // Find matching pair
  auto match_result = matchPair(camera_time, trigger_time);
  while (match_result != PairMatchResult::Match && !is_camera_queue_empty &&
         !is_trigger_queue_empty) {
    if (match_result == PairMatchResult::CameraTimeOld) {
      std::lock_guard<std::mutex> queue_lock(camera_queue_mutex_);
      camera_time_queue_.pop();
      camera_time = camera_time_queue_.front();
      is_camera_queue_empty = camera_time_queue_.empty();
    } else if (match_result == PairMatchResult::TriggerTimeOld) {
      std::lock_guard<std::mutex> queue_lock(trigger_queue_mutex_);
      trigger_time_queue_.pop();
      trigger_time = trigger_time_queue_.front();
      is_trigger_queue_empty = trigger_time_queue_.empty();
    }
    match_result = matchPair(camera_time, trigger_time);
  }

  if (!is_camera_queue_empty) {
    std::lock_guard<std::mutex> queue_lock(camera_queue_mutex_);
    camera_time_queue_.pop();
  }
  if (!is_trigger_queue_empty) {
    std::lock_guard<std::mutex> queue_lock(trigger_queue_mutex_);
    trigger_time_queue_.pop();
  }

  if (match_result != PairMatchResult::Match) {
    // matched pair is not found
    return;
  }

  // trigger time diagnose
  auto trigger_sync_status = diagnoseTriggerTime(trigger_time);

  // camera - trigger time diagnose
  auto camera_sync_status = diagnoseCameraTime(camera_time, trigger_time);

  // Update diagnostis
  stat.summary(std::max(trigger_sync_status.level, camera_sync_status.level), "");

  auto addKeyValue = [&stat](const std::string & key, const auto & value) {
    std::stringstream value_ss;
    value_ss << value;
    stat.add(key, value_ss.str());
  };

  addKeyValue("Camera status", stringize_level(camera_sync_status.level));
  addKeyValue("Camera diff nanoseconds", camera_sync_status.diff_nsec);
  addKeyValue("Camera diff threshold", camera_time_tolerance_);

  addKeyValue("Trigger status", stringize_level(trigger_sync_status.level));
  addKeyValue("Trigger diff nanoseconds", trigger_sync_status.diff_nsec);
  addKeyValue("Trigger diff threshold", trigger_time_tolerance_);
}

}  // namespace tier4_camera_sync_doctor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_camera_sync_doctor::Tier4CameraSyncDoctor)
