#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <openvr.h>
#include <rclcpp/rclcpp.hpp>
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

}  // namespace

class ViveTrackerNode : public rclcpp::Node
{
public:
  ViveTrackerNode()
  : Node("vive_tracker_node")
  {
    // 参数都放在 launch 中可配置，便于后续切换 frame 名、发布频率或只读取指定 serial。
    m_frame_id = declare_parameter<std::string>("frame_id", "steamvr_world");
    m_child_frame_prefix = declare_parameter<std::string>("child_frame_prefix", "vive_tracker");
    m_topic_prefix = declare_parameter<std::string>("topic_prefix", "/vive_tracker");
    m_device_serial_filter = declare_parameter<std::string>("device_serial", "");
    m_publish_tf = declare_parameter<bool>("publish_tf", true);
    m_publish_first_pose_topic = declare_parameter<bool>("publish_first_pose_topic", true);
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

    bool published_first_pose = false;
    for (vr::TrackedDeviceIndex_t index = 0; index < vr::k_unMaxTrackedDeviceCount; ++index) {
      const auto & pose = poses[index];
      if (!pose.bDeviceIsConnected || !pose.bPoseIsValid) {
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
      
      // 每个 tracker 按 serial 建立独立 topic 和 TF frame，多个 tracker 可以同时发布。
      const std::string tracker_name = sanitizeName(serial.empty() ? std::to_string(index) : serial);
      const std::string child_frame_id = m_child_frame_prefix + "_" + tracker_name;
      auto pose_msg = makePoseMessage(pose.mDeviceToAbsoluteTracking);

      getPublisher(tracker_name)->publish(pose_msg);
      if (m_publish_first_pose_topic && !published_first_pose) {
        m_first_pose_publisher->publish(pose_msg);
        published_first_pose = true;
      }

      if (m_publish_tf) {
        // TF 表达的是 frame_id -> child_frame_id 的坐标变换。
        geometry_msgs::msg::TransformStamped transform;
        transform.header = pose_msg.header;
        transform.child_frame_id = child_frame_id;
        transform.transform.translation.x = pose_msg.pose.position.x;
        transform.transform.translation.y = pose_msg.pose.position.y;
        transform.transform.translation.z = pose_msg.pose.position.z;
        transform.transform.rotation = pose_msg.pose.orientation;
        m_tf_broadcaster->sendTransform(transform);
      }
    }
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

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr getPublisher(
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

  vr::IVRSystem * m_vr_system{nullptr};
  vr::ETrackingUniverseOrigin m_tracking_universe{vr::TrackingUniverseStanding};
  std::string m_frame_id;
  std::string m_child_frame_prefix;
  std::string m_topic_prefix;
  std::string m_device_serial_filter;
  bool m_publish_tf{true};
  bool m_publish_first_pose_topic{true};
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
  m_pose_publishers;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_first_pose_publisher;
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
