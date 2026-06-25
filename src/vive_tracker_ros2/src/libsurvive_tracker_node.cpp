#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

#define SURVIVE_ENABLE_FULL_API
#include <survive.h>
#include <survive_api.h>
#include <tf2_ros/transform_broadcaster.h>

namespace
{

std::string sanitizeName(const std::string & value)
{
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

  return sanitized.empty() ? "unknown" : sanitized;
}

bool isValidPose(const SurvivePose & pose)
{
  const double quaternion_norm =
    pose.Rot[0] * pose.Rot[0] + pose.Rot[1] * pose.Rot[1] +
    pose.Rot[2] * pose.Rot[2] + pose.Rot[3] * pose.Rot[3];

  return std::isfinite(pose.Pos[0]) && std::isfinite(pose.Pos[1]) &&
         std::isfinite(pose.Pos[2]) && std::isfinite(quaternion_norm) &&
         quaternion_norm > 0.5;
}

}  // namespace

class LibsurviveTrackerNode : public rclcpp::Node
{
public:
  LibsurviveTrackerNode()
  : Node("vive_tracker_node"),
    m_tf_broadcaster(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    m_frame_id = declare_parameter<std::string>("frame_id", "steamvr_base");
    m_child_frame_id = declare_parameter<std::string>("child_frame_id", "tracker_frame");
    m_topic_prefix = declare_parameter<std::string>("topic_prefix", "/vive_tracker");
    m_device_serial = declare_parameter<std::string>("device_serial", "");
    m_publish_tf = declare_parameter<bool>("publish_tf", true);
    m_publish_first_pose_topic = declare_parameter<bool>("publish_first_pose_topic", true);
    m_debug_events = declare_parameter<bool>("debug_events", false);

    const int lighthouse_count = declare_parameter<int>("lighthouse_count", 2);
    const int lighthouse_generation = declare_parameter<int>("lighthouse_generation", 1);
    const bool center_on_lighthouse = declare_parameter<bool>("center_on_lighthouse", true);
    const bool force_calibrate = declare_parameter<bool>("force_calibrate", false);
    const bool use_raw_observation = declare_parameter<bool>("use_raw_observation", false);
    const int verbosity = declare_parameter<int>("libsurvive_verbosity", 1);
    const double poll_rate_hz = declare_parameter<double>("poll_rate_hz", 250.0);
    const std::string config_file = declare_parameter<std::string>("config_file", "");

    m_first_pose_publisher =
      create_publisher<geometry_msgs::msg::PoseStamped>(m_topic_prefix + "/pose", 10);
    m_first_button_publisher =
      create_publisher<sensor_msgs::msg::Joy>(m_topic_prefix + "/buttons", 10);
    m_visibility_publisher =
      create_publisher<std_msgs::msg::UInt32MultiArray>(m_topic_prefix + "/visibility", 10);

    std::vector<std::string> arguments{
      "vive_tracker_libsurvive",
      "-l", std::to_string(lighthouse_count),
      "--lighthouse-gen", std::to_string(lighthouse_generation),
      "--v", std::to_string(verbosity),
    };
    if (center_on_lighthouse) {
      arguments.emplace_back("--center-on-lh0");
    }
    if (force_calibrate) {
      arguments.emplace_back("--force-calibrate");
    }
    if (use_raw_observation) {
      arguments.emplace_back("--use-raw-obs");
    }
    if (!config_file.empty()) {
      arguments.emplace_back("-c");
      arguments.emplace_back(config_file);
    }

    std::vector<char *> argv;
    argv.reserve(arguments.size());
    for (auto & argument : arguments) {
      argv.push_back(argument.data());
    }

    m_context = survive_simple_init(static_cast<int>(argv.size()), argv.data());
    if (m_context == nullptr) {
      throw std::runtime_error("libsurvive initialization failed");
    }
    survive_simple_start_thread(m_context);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, poll_rate_hz));
    m_poll_timer = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LibsurviveTrackerNode::pollEvents, this));

    RCLCPP_INFO(
      get_logger(),
      "libsurvive initialized: lighthouses=%d generation=%d center_on_lighthouse=%s "
      "force_calibrate=%s use_raw_observation=%s",
      lighthouse_count, lighthouse_generation, center_on_lighthouse ? "true" : "false",
      force_calibrate ? "true" : "false", use_raw_observation ? "true" : "false");
  }

  ~LibsurviveTrackerNode() override
  {
    if (m_context != nullptr) {
      survive_simple_close(m_context);
      m_context = nullptr;
    }
  }

private:
  using PosePublisher = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>;
  using ButtonPublisher = rclcpp::Publisher<sensor_msgs::msg::Joy>;

  struct ButtonState
  {
    std::array<int32_t, 4> buttons{0, 0, 0, 0};
    std::array<float, 3> axes{0.0F, 0.0F, 0.0F};
  };

  bool acceptsDevice(const SurviveSimpleObject * object) const
  {
    if (object == nullptr ||
      survive_simple_object_get_type(object) != SurviveSimpleObject_OBJECT)
    {
      return false;
    }

    const char * serial_value = survive_simple_serial_number(object);
    const std::string serial = serial_value == nullptr ? "" : serial_value;
    return m_device_serial.empty() || serial == m_device_serial;
  }

  std::string serialOf(const SurviveSimpleObject * object) const
  {
    const char * serial = survive_simple_serial_number(object);
    if (serial != nullptr && serial[0] != '\0') {
      return serial;
    }

    const char * name = survive_simple_object_name(object);
    return name == nullptr ? "unknown" : name;
  }

  PosePublisher::SharedPtr posePublisherFor(const std::string & serial)
  {
    const auto found = m_pose_publishers.find(serial);
    if (found != m_pose_publishers.end()) {
      return found->second;
    }

    const auto topic = m_topic_prefix + "/" + sanitizeName(serial) + "/pose";
    auto publisher = create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
    m_pose_publishers.emplace(serial, publisher);
    RCLCPP_INFO(get_logger(), "Publishing tracker '%s' pose on %s", serial.c_str(), topic.c_str());
    return publisher;
  }

  ButtonPublisher::SharedPtr buttonPublisherFor(const std::string & serial)
  {
    const auto found = m_button_publishers.find(serial);
    if (found != m_button_publishers.end()) {
      return found->second;
    }

    const auto topic = m_topic_prefix + "/" + sanitizeName(serial) + "/buttons";
    auto publisher = create_publisher<sensor_msgs::msg::Joy>(topic, 10);
    m_button_publishers.emplace(serial, publisher);
    RCLCPP_INFO(
      get_logger(), "Publishing tracker '%s' buttons on %s", serial.c_str(), topic.c_str());
    return publisher;
  }

  void publishPose(const SurviveSimplePoseUpdatedEvent & event)
  {
    if (!acceptsDevice(event.object) || !isValidPose(event.pose)) {
      return;
    }

    const auto stamp = now();
    const std::string serial = serialOf(event.object);

    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = m_frame_id;
    message.pose.position.x = event.pose.Pos[0];
    message.pose.position.y = event.pose.Pos[1];
    message.pose.position.z = event.pose.Pos[2];

    // libsurvive 使用 [w, x, y, z]，ROS 消息字段顺序为 [x, y, z, w]。
    message.pose.orientation.w = event.pose.Rot[0];
    message.pose.orientation.x = event.pose.Rot[1];
    message.pose.orientation.y = event.pose.Rot[2];
    message.pose.orientation.z = event.pose.Rot[3];

    posePublisherFor(serial)->publish(message);

    const bool publish_as_primary =
      m_device_serial.empty() ? (m_primary_serial.empty() || m_primary_serial == serial) :
      serial == m_device_serial;
    if (m_primary_serial.empty() && publish_as_primary) {
      m_primary_serial = serial;
    }

    if (m_publish_first_pose_topic && publish_as_primary) {
      m_first_pose_publisher->publish(message);
    }

    if (m_publish_tf && publish_as_primary) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = message.header;
      transform.child_frame_id = m_child_frame_id;
      transform.transform.translation.x = message.pose.position.x;
      transform.transform.translation.y = message.pose.position.y;
      transform.transform.translation.z = message.pose.position.z;
      transform.transform.rotation = message.pose.orientation;
      m_tf_broadcaster->sendTransform(transform);
    }

    if (publish_as_primary) {
      m_primary_object = event.object;
    }
  }

  void publishVisibility(const SurviveSimpleObject * object)
  {
    const auto stamp = now();
    if ((stamp - m_last_visibility_stamp).seconds() < 0.2) {
      return;
    }
    m_last_visibility_stamp = stamp;

    uint32_t measurement_count = 0;
    uint32_t lighthouse_count = 0;
    uint32_t axis_count = 0;
    std::array<std::size_t, NUM_GEN1_LIGHTHOUSES * 2> measurements_per_axis{};

    survive_simple_lock(m_context);
    SurviveObject * survive_object = survive_simple_get_survive_object(object);
    if (survive_object != nullptr) {
      SurviveSensorActivations_valid_counts(
        &survive_object->activations, 0, &measurement_count, &lighthouse_count,
        &axis_count, measurements_per_axis.data());
    }
    survive_simple_unlock(m_context);

    std_msgs::msg::UInt32MultiArray message;
    message.layout.dim.resize(1);
    message.layout.dim[0].label =
      "visible_lighthouses,total_measurements,lh0_x,lh0_y,lh1_x,lh1_y";
    message.layout.dim[0].size = 6;
    message.layout.dim[0].stride = 6;
    message.data = {
      lighthouse_count,
      measurement_count,
      static_cast<uint32_t>(measurements_per_axis[0]),
      static_cast<uint32_t>(measurements_per_axis[1]),
      static_cast<uint32_t>(measurements_per_axis[2]),
      static_cast<uint32_t>(measurements_per_axis[3]),
    };
    m_visibility_publisher->publish(message);
  }

  static int buttonIndex(enum SurviveButton button)
  {
    switch (button) {
      case SURVIVE_BUTTON_TRIGGER:
        return 0;
      case SURVIVE_BUTTON_GRIP:
        return 1;
      case SURVIVE_BUTTON_TRACKPAD:
      case SURVIVE_BUTTON_THUMBSTICK:
      case SURVIVE_BUTTON_A:
        return 2;
      case SURVIVE_BUTTON_MENU:
      case SURVIVE_BUTTON_SYSTEM:
      case SURVIVE_BUTTON_B:
        return 3;
      default:
        return -1;
    }
  }

  void publishButtons(const SurviveSimpleButtonEvent & event)
  {
    if (!acceptsDevice(event.object)) {
      return;
    }

    const std::string serial = serialOf(event.object);
    auto & state = m_button_states[serial];
    if (m_primary_object == nullptr &&
      (m_device_serial.empty() || serial == m_device_serial))
    {
      m_primary_object = event.object;
    }

    if (event.event_type == SURVIVE_INPUT_EVENT_BUTTON_DOWN ||
      event.event_type == SURVIVE_INPUT_EVENT_BUTTON_UP)
    {
      const int index = buttonIndex(event.button_id);
      if (index >= 0) {
        state.buttons[static_cast<std::size_t>(index)] =
          event.event_type == SURVIVE_INPUT_EVENT_BUTTON_DOWN ? 1 : 0;
      }
    }

    for (uint8_t i = 0; i < event.axis_count; ++i) {
      switch (event.axis_ids[i]) {
        case SURVIVE_AXIS_TRACKPAD_X:
          state.axes[0] = event.axis_val[i];
          break;
        case SURVIVE_AXIS_TRACKPAD_Y:
          state.axes[1] = event.axis_val[i];
          break;
        case SURVIVE_AXIS_TRIGGER:
          state.axes[2] = event.axis_val[i];
          break;
        default:
          break;
      }
    }

    sensor_msgs::msg::Joy message;
    message.header.stamp = now();
    message.header.frame_id = m_child_frame_id;
    message.buttons.assign(state.buttons.begin(), state.buttons.end());
    message.axes.assign(state.axes.begin(), state.axes.end());

    buttonPublisherFor(serial)->publish(message);
    if (m_primary_serial.empty()) {
      m_primary_serial = serial;
    }
    if (serial == m_primary_serial || serial == m_device_serial) {
      m_first_button_publisher->publish(message);
    }

    if (m_debug_events) {
      RCLCPP_INFO(
        get_logger(),
        "button serial=%s event=%d id=%d state=[%d %d %d %d] axes=[%.3f %.3f %.3f]",
        serial.c_str(), event.event_type, event.button_id,
        state.buttons[0], state.buttons[1], state.buttons[2], state.buttons[3],
        state.axes[0], state.axes[1], state.axes[2]);
    }
  }

  void pollEvents()
  {
    if (m_context == nullptr || !survive_simple_is_running(m_context)) {
      return;
    }

    SurviveSimpleEvent event{};
    for (int processed = 0; processed < 512; ++processed) {
      const auto type = survive_simple_next_event(m_context, &event);
      if (type == SurviveSimpleEventType_None) {
        break;
      }

      if (type == SurviveSimpleEventType_PoseUpdateEvent) {
        const auto * pose_event = survive_simple_get_pose_updated_event(&event);
        if (pose_event != nullptr) {
          publishPose(*pose_event);
        }
      } else if (type == SurviveSimpleEventType_ButtonEvent) {
        const auto * button_event = survive_simple_get_button_event(&event);
        if (button_event != nullptr) {
          publishButtons(*button_event);
        }
      } else if (type == SurviveSimpleEventType_DeviceAdded) {
        const auto * object_event = survive_simple_get_object_event(&event);
        if (object_event != nullptr && acceptsDevice(object_event->object) &&
          m_primary_object == nullptr)
        {
          m_primary_object = object_event->object;
        }
      }
    }

    if (m_primary_object != nullptr) {
      publishVisibility(m_primary_object);
    }
  }

  SurviveSimpleContext * m_context{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  rclcpp::TimerBase::SharedPtr m_poll_timer;

  std::string m_frame_id;
  std::string m_child_frame_id;
  std::string m_topic_prefix;
  std::string m_device_serial;
  std::string m_primary_serial;
  const SurviveSimpleObject * m_primary_object{nullptr};
  bool m_publish_tf{true};
  bool m_publish_first_pose_topic{true};
  bool m_debug_events{false};

  PosePublisher::SharedPtr m_first_pose_publisher;
  ButtonPublisher::SharedPtr m_first_button_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr m_visibility_publisher;
  rclcpp::Time m_last_visibility_stamp{0, 0, RCL_ROS_TIME};
  std::map<std::string, PosePublisher::SharedPtr> m_pose_publishers;
  std::map<std::string, ButtonPublisher::SharedPtr> m_button_publishers;
  std::map<std::string, ButtonState> m_button_states;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LibsurviveTrackerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("vive_tracker_node"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
