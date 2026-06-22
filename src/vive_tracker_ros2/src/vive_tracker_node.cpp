#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <openvr.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{

std::string sanitizeName(const std::string & value)
{
  // ROS 的 topic/frame 名称不适合直接使用设备 serial，这里统一转成小写安全名称。
  std::string sanitized;
  sanitized.reserve(value.size());

  for (const char c : value) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      sanitized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    } else {
      sanitized.push_back('_');
    }
  }

  sanitized.erase(
    std::unique(sanitized.begin(), sanitized.end(), [](char lhs, char rhs) {
      return lhs == '_' && rhs == '_';
    }),
    sanitized.end());

  while (!sanitized.empty() && sanitized.front() == '_') {
    sanitized.erase(sanitized.begin());
  }
  while (!sanitized.empty() && sanitized.back() == '_') {
    sanitized.pop_back();
  }

  if (sanitized.empty()) {
    return "unknown";
  }
  if (std::isdigit(static_cast<unsigned char>(sanitized.front()))) {
    sanitized.insert(sanitized.begin(), 't');
  }
  return sanitized;
}

std::string getTrackedDeviceString(
  vr::IVRSystem * vr_system,
  vr::TrackedDeviceIndex_t device_index,
  vr::ETrackedDeviceProperty property)
{
  // OpenVR 的字符串属性需要先查询长度，再按长度分配缓冲区读取。
  vr::ETrackedPropertyError error = vr::TrackedProp_Success;
  const uint32_t length = vr_system->GetStringTrackedDeviceProperty(
    device_index, property, nullptr, 0, &error);

  if (length == 0) {
    return {};
  }

  std::vector<char> buffer(length);
  error = vr::TrackedProp_Success;
  vr_system->GetStringTrackedDeviceProperty(
    device_index, property, buffer.data(), static_cast<uint32_t>(buffer.size()), &error);

  if (error != vr::TrackedProp_Success) {
    return {};
  }

  return std::string(buffer.data());
}

geometry_msgs::msg::Quaternion rotationMatrixToQuaternion(const vr::HmdMatrix34_t & matrix)
{
  // OpenVR 返回 3x4 刚体变换矩阵：左侧 3x3 是旋转，最后一列是平移。
  // ROS Pose/TF 使用四元数表达姿态，所以这里把旋转矩阵转成单位四元数。
  const double r00 = matrix.m[0][0];
  const double r01 = matrix.m[0][1];
  const double r02 = matrix.m[0][2];
  const double r10 = matrix.m[1][0];
  const double r11 = matrix.m[1][1];
  const double r12 = matrix.m[1][2];
  const double r20 = matrix.m[2][0];
  const double r21 = matrix.m[2][1];
  const double r22 = matrix.m[2][2];

  geometry_msgs::msg::Quaternion quaternion;
  const double trace = r00 + r11 + r22;

  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    quaternion.w = 0.25 * s;
    quaternion.x = (r21 - r12) / s;
    quaternion.y = (r02 - r20) / s;
    quaternion.z = (r10 - r01) / s;
  } else if (r00 > r11 && r00 > r22) {
    const double s = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
    quaternion.w = (r21 - r12) / s;
    quaternion.x = 0.25 * s;
    quaternion.y = (r01 + r10) / s;
    quaternion.z = (r02 + r20) / s;
  } else if (r11 > r22) {
    const double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
    quaternion.w = (r02 - r20) / s;
    quaternion.x = (r01 + r10) / s;
    quaternion.y = 0.25 * s;
    quaternion.z = (r12 + r21) / s;
  } else {
    const double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
    quaternion.w = (r10 - r01) / s;
    quaternion.x = (r02 + r20) / s;
    quaternion.y = (r12 + r21) / s;
    quaternion.z = 0.25 * s;
  }

  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (norm > 0.0) {
    quaternion.x /= norm;
    quaternion.y /= norm;
    quaternion.z /= norm;
    quaternion.w /= norm;
  }

  return quaternion;
}

// OpenVR 定义了三种 tracking universe，分别是 seated、standing 和 raw，坐标系原点和朝向不同。
// standing 是 SteamVR 房间设置后的常用坐标系；raw 则是未校准追踪空间。若自己进行标定可使用raw
vr::ETrackingUniverseOrigin parseTrackingUniverse(const std::string & value)
{
  // standing 是 SteamVR 房间设置后的常用坐标系；raw 则是未校准追踪空间。
  const auto name = sanitizeName(value);
  if (name == "seated") {
    return vr::TrackingUniverseSeated;
  }
  if (name == "raw" || name == "raw_and_uncalibrated" || name == "rawanduncalibrated") {
    return vr::TrackingUniverseRawAndUncalibrated;
  }
  return vr::TrackingUniverseStanding;
}

std::string trackedDeviceClassToString(vr::ETrackedDeviceClass device_class)
{
  switch (device_class) {
    case vr::TrackedDeviceClass_HMD:
      return "HMD";
    case vr::TrackedDeviceClass_Controller:
      return "Controller";
    case vr::TrackedDeviceClass_GenericTracker:
      return "GenericTracker";
    case vr::TrackedDeviceClass_TrackingReference:
      return "TrackingReference";
    case vr::TrackedDeviceClass_DisplayRedirect:
      return "DisplayRedirect";
    case vr::TrackedDeviceClass_Invalid:
    default:
      return "Invalid";
  }
}

std::string trackingResultToString(vr::ETrackingResult tracking_result)
{
  switch (tracking_result) {
    case vr::TrackingResult_Uninitialized:
      return "Uninitialized";
    case vr::TrackingResult_Calibrating_InProgress:
      return "Calibrating_InProgress";
    case vr::TrackingResult_Calibrating_OutOfRange:
      return "Calibrating_OutOfRange";
    case vr::TrackingResult_Running_OK:
      return "Running_OK";
    case vr::TrackingResult_Running_OutOfRange:
      return "Running_OutOfRange";
    case vr::TrackingResult_Fallback_RotationOnly:
      return "Fallback_RotationOnly";
    default:
      return "Unknown";
  }
}

bool isButtonPressed(const vr::VRControllerState_t & state, vr::EVRButtonId button)
{
  return (state.ulButtonPressed & vr::ButtonMaskFromId(button)) != 0;
}

std::string buttonStateToString(const vr::VRControllerState_t & state)
{
  std::ostringstream stream;
  stream << "trigger="
         << (isButtonPressed(state, vr::k_EButton_SteamVR_Trigger) ? "1" : "0")
         << " grip=" << (isButtonPressed(state, vr::k_EButton_Grip) ? "1" : "0")
         << " trackpad="
         << (isButtonPressed(state, vr::k_EButton_SteamVR_Touchpad) ? "1" : "0")
         << " menu="
         << (isButtonPressed(state, vr::k_EButton_ApplicationMenu) ? "1" : "0")
         << " packet=" << state.unPacketNum;
  return stream.str();
}

}  // namespace

class ViveTrackerNode : public rclcpp::Node
{
public:
  ViveTrackerNode()
  : Node("vive_tracker_node")
  {
    // 参数都放在 launch 中可配置，便于后续切换 frame 名、发布频率或只读取指定 serial。
    m_frame_id = declare_parameter<std::string>("frame_id", "steamvr_base");
    m_child_frame_id = declare_parameter<std::string>("child_frame_id", "tracker_frame");
    m_topic_prefix = declare_parameter<std::string>("topic_prefix", "/vive_tracker");
    m_device_serial_filter = declare_parameter<std::string>("device_serial", "");
    m_publish_tf = declare_parameter<bool>("publish_tf", true);
    m_publish_first_pose_topic = declare_parameter<bool>("publish_first_pose_topic", true);
    m_debug_devices = declare_parameter<bool>("debug_devices", false);
    const double update_rate_hz = declare_parameter<double>("update_rate_hz", 100.0);
    const auto universe_name = declare_parameter<std::string>("tracking_universe", "standing");
    m_tracking_universe = parseTrackingUniverse(universe_name);

    vr::EVRInitError init_error = vr::VRInitError_None;
    // VRApplication_Background 表示后台程序读取 SteamVR/OpenVR 数据，不创建 VR 应用界面。
    m_vr_system = vr::VR_Init(&init_error, vr::VRApplication_Background);
    if (init_error != vr::VRInitError_None || m_vr_system == nullptr) {
      const char * error_name = vr::VR_GetVRInitErrorAsEnglishDescription(init_error);
      throw std::runtime_error(
              std::string("Failed to initialize OpenVR: ") +
              (error_name == nullptr ? "unknown error" : error_name));
    }

    if (m_publish_tf) {
      m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    if (m_publish_first_pose_topic) {
      m_first_pose_publisher =
        create_publisher<geometry_msgs::msg::PoseStamped>(m_topic_prefix + "/pose", 10);
    }
    m_first_buttons_publisher =
      create_publisher<sensor_msgs::msg::Joy>(m_topic_prefix + "/buttons", 10);

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(update_rate_hz, 1.0));
    m_timer = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ViveTrackerNode::pollTrackers, this));

    RCLCPP_INFO(
      get_logger(),
      "OpenVR initialized. Publishing Vive Tracker poses under '%s'",
      m_topic_prefix.c_str());
  }

  ~ViveTrackerNode() override
  {
    // OpenVR 是进程级 runtime，节点退出时要成对调用 VR_Shutdown。
    if (m_vr_system != nullptr) {
      vr::VR_Shutdown();
      m_vr_system = nullptr;
    }
  }

private:
  void pollTrackers()
  {
    // 一次性读取 OpenVR 当前所有 tracked devices 的位姿，后面再筛选 Vive Tracker。
    std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> poses;
    m_vr_system->GetDeviceToAbsoluteTrackingPose(
      m_tracking_universe, 0.0f, poses.data(), vr::k_unMaxTrackedDeviceCount);

    if (m_debug_devices) {
      logDeviceDebugInfo(poses);
    }

    bool published_first_pose = false;
    bool published_first_buttons = false;
    for (vr::TrackedDeviceIndex_t index = 0; index < vr::k_unMaxTrackedDeviceCount; ++index) {
      const auto & pose = poses[index];
      if (!pose.bDeviceIsConnected) {
        continue;
      }
      // 只处理 GenericTracker，也就是 Vive Tracker 这类通用追踪器；忽略头显、手柄、基站。
      if (m_vr_system->GetTrackedDeviceClass(index) != vr::TrackedDeviceClass_GenericTracker) {
        continue;
      }

      const std::string serial = getTrackedDeviceString(
        m_vr_system, index, vr::Prop_SerialNumber_String);
      if (!m_device_serial_filter.empty() && serial != m_device_serial_filter) {
        continue;
      }
      
      // 项目当前只使用一个 tracker，TF frame 固定为 tracker_frame。
      // topic 仍按 serial 区分，便于调试 OpenVR 实际识别到的设备。
      const std::string tracker_name = sanitizeName(serial.empty() ? std::to_string(index) : serial);

      vr::VRControllerState_t controller_state{};
      if (m_vr_system->GetControllerState(index, &controller_state, sizeof(controller_state))) {
        auto joy_msg = makeJoyMessage(controller_state);
        getButtonsPublisher(tracker_name)->publish(joy_msg);
        if (!published_first_buttons) {
          m_first_buttons_publisher->publish(joy_msg);
          published_first_buttons = true;
        }
      }

      if (pose.bPoseIsValid) {
        auto pose_msg = makePoseMessage(pose.mDeviceToAbsoluteTracking);

        getPosePublisher(tracker_name)->publish(pose_msg);
        if (m_publish_first_pose_topic && !published_first_pose) {
          m_first_pose_publisher->publish(pose_msg);
          published_first_pose = true;
        }

        if (m_publish_tf) {
          // TF 表达的是 frame_id -> child_frame_id 的坐标变换。
          geometry_msgs::msg::TransformStamped transform;
          transform.header = pose_msg.header;
          transform.child_frame_id = m_child_frame_id;
          transform.transform.translation.x = pose_msg.pose.position.x;
          transform.transform.translation.y = pose_msg.pose.position.y;
          transform.transform.translation.z = pose_msg.pose.position.z;
          transform.transform.rotation = pose_msg.pose.orientation;
          m_tf_broadcaster->sendTransform(transform);
        }
      }
    }
  }

  void logDeviceDebugInfo(
    const std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> & poses)
  {
    const auto current_time = now();
    if (m_last_debug_log_time.nanoseconds() != 0 &&
      (current_time - m_last_debug_log_time).seconds() < 2.0)
    {
      return;
    }
    m_last_debug_log_time = current_time;

    std::ostringstream stream;
    bool has_connected_device = false;
    stream << "OpenVR devices:";

    for (vr::TrackedDeviceIndex_t index = 0; index < vr::k_unMaxTrackedDeviceCount; ++index) {
      const auto & pose = poses[index];
      if (!pose.bDeviceIsConnected) {
        continue;
      }

      has_connected_device = true;
      const auto device_class = m_vr_system->GetTrackedDeviceClass(index);
      const std::string serial = getTrackedDeviceString(
        m_vr_system, index, vr::Prop_SerialNumber_String);
      const std::string model = getTrackedDeviceString(
        m_vr_system, index, vr::Prop_ModelNumber_String);
      vr::VRControllerState_t controller_state{};
      const bool has_controller_state =
        m_vr_system->GetControllerState(index, &controller_state, sizeof(controller_state));

      stream << "\n  index=" << static_cast<int>(index)
             << " serial=" << (serial.empty() ? "<empty>" : serial)
             << " model=" << (model.empty() ? "<empty>" : model)
             << " class=" << trackedDeviceClassToString(device_class)
             << " connected=" << (pose.bDeviceIsConnected ? "true" : "false")
             << " pose_valid=" << (pose.bPoseIsValid ? "true" : "false")
             << " tracking_result=" << trackingResultToString(pose.eTrackingResult)
             << " buttons="
             << (has_controller_state ? buttonStateToString(controller_state) : "<unavailable>");
    }

    if (!has_connected_device) {
      stream << " no connected devices";
    }

    RCLCPP_INFO(get_logger(), "%s", stream.str().c_str());
  }

  geometry_msgs::msg::PoseStamped makePoseMessage(const vr::HmdMatrix34_t & matrix)
  {
    // matrix.m[0..2][3] 是 tracker 在 OpenVR tracking universe 下的位置，单位是米。
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = m_frame_id;
    pose_msg.pose.position.x = matrix.m[0][3];
    pose_msg.pose.position.y = matrix.m[1][3];
    pose_msg.pose.position.z = matrix.m[2][3];
    pose_msg.pose.orientation = rotationMatrixToQuaternion(matrix);
    return pose_msg;
  }

  sensor_msgs::msg::Joy makeJoyMessage(const vr::VRControllerState_t & state)
  {
    // Joy.buttons 顺序固定为：trigger、grip、trackpad、menu。
    // Pogo pin 输入为低电平有效，但 OpenVR 已经转换成“按下/未按下”的按钮状态。
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = now();
    joy_msg.header.frame_id = m_child_frame_id;
    joy_msg.buttons = {
      isButtonPressed(state, vr::k_EButton_SteamVR_Trigger) ? 1 : 0,
      isButtonPressed(state, vr::k_EButton_Grip) ? 1 : 0,
      isButtonPressed(state, vr::k_EButton_SteamVR_Touchpad) ? 1 : 0,
      isButtonPressed(state, vr::k_EButton_ApplicationMenu) ? 1 : 0,
    };
    joy_msg.axes = {
      state.rAxis[0].x,
      state.rAxis[0].y,
      state.rAxis[1].x,
    };
    return joy_msg;
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr getPosePublisher(
    const std::string & tracker_name)
  {
    // topic 数量取决于运行时识别到的 tracker，因此 publisher 按需创建并缓存。
    const auto found = m_pose_publishers.find(tracker_name);
    if (found != m_pose_publishers.end()) {
      return found->second;
    }

    const std::string topic = m_topic_prefix + "/" + tracker_name + "/pose";
    auto publisher = create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
    m_pose_publishers.emplace(tracker_name, publisher);
    RCLCPP_INFO(get_logger(), "Publishing tracker '%s' on %s", tracker_name.c_str(), topic.c_str());
    return publisher;
  }

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr getButtonsPublisher(
    const std::string & tracker_name)
  {
    const auto found = m_buttons_publishers.find(tracker_name);
    if (found != m_buttons_publishers.end()) {
      return found->second;
    }

    const std::string topic = m_topic_prefix + "/" + tracker_name + "/buttons";
    auto publisher = create_publisher<sensor_msgs::msg::Joy>(topic, 10);
    m_buttons_publishers.emplace(tracker_name, publisher);
    RCLCPP_INFO(
      get_logger(), "Publishing tracker '%s' buttons on %s", tracker_name.c_str(), topic.c_str());
    return publisher;
  }

  vr::IVRSystem * m_vr_system{nullptr};
  vr::ETrackingUniverseOrigin m_tracking_universe{vr::TrackingUniverseStanding};
  std::string m_frame_id;
  std::string m_child_frame_id;
  std::string m_topic_prefix;
  std::string m_device_serial_filter;
  bool m_publish_tf{true};
  bool m_publish_first_pose_topic{true};
  bool m_debug_devices{false};
  rclcpp::Time m_last_debug_log_time{0, 0, RCL_ROS_TIME};
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
  m_pose_publishers;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr> m_buttons_publishers;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_first_pose_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr m_first_buttons_publisher;
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<ViveTrackerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("vive_tracker_node"), "%s", error.what());
  }

  rclcpp::shutdown();
  return 0;
}
