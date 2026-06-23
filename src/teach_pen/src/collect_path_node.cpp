#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

struct PathSample
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Transform transform;
};

class CollectPathNode : public rclcpp::Node
{
public:
  CollectPathNode()
  : Node("collect_path_node"),
    m_tf_buffer(std::make_unique<tf2_ros::Buffer>(get_clock())),
    
    m_tf_listener(std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer))
  {
    m_base_frame = declare_parameter<std::string>("base_frame", "robot_base");
    m_tip_frame = declare_parameter<std::string>("tip_frame", "teaching_pen_tip");
    m_buttons_topic = declare_parameter<std::string>("buttons_topic", "/vive_tracker/buttons");
    m_output_file = declare_parameter<std::string>("output_file", "config/paths/demo_path.yaml");
    m_sample_button = declare_parameter<int>("sample_button", 0);
    m_record_button = declare_parameter<int>("record_button", 1);
    m_undo_button = declare_parameter<int>("undo_button", 2);
    m_save_button = declare_parameter<int>("save_button", 3);
    m_lookup_timeout_sec = declare_parameter<double>("lookup_timeout_sec", 0.2);
    m_min_record_interval_sec = declare_parameter<double>("min_record_interval_sec", 0.05);

    m_buttons_sub = create_subscription<sensor_msgs::msg::Joy>(
      m_buttons_topic,
      rclcpp::QoS(10),
      std::bind(&CollectPathNode::buttonsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Collect path from TF %s <- %s. buttons: sample=%d record=%d undo=%d save=%d topic=%s",
      m_base_frame.c_str(), m_tip_frame.c_str(), m_sample_button, m_record_button,
      m_undo_button, m_save_button, m_buttons_topic.c_str());
  }

private:
  void buttonsCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const bool sample_pressed = isPressed(*msg, m_sample_button);
    const bool record_pressed = isPressed(*msg, m_record_button);
    const bool undo_pressed = isPressed(*msg, m_undo_button);
    const bool save_pressed = isPressed(*msg, m_save_button);

    if (risingEdge(m_sample_button, sample_pressed)) {
      sampleOnce("button");
    }

    if (risingEdge(m_record_button, record_pressed)) {
      m_recording = !m_recording;
      m_last_record_sample_time = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      RCLCPP_INFO(get_logger(), "continuous recording: %s", m_recording ? "on" : "off");
    }

    if (risingEdge(m_undo_button, undo_pressed)) {
      undoLastSample();
    }

    if (risingEdge(m_save_button, save_pressed)) {
      saveSamples();
    }

    if (m_recording) {
      const auto now_time = now();
      if (m_last_record_sample_time.nanoseconds() == 0 ||
        (now_time - m_last_record_sample_time).seconds() >= m_min_record_interval_sec)
      {
        if (sampleOnce("record")) {
          m_last_record_sample_time = now_time;
        }
      }
    }

    m_previous_buttons = msg->buttons;
  }

  bool sampleOnce(const std::string & reason)
  {
    try {
      const auto transform = m_tf_buffer->lookupTransform(
        m_base_frame,
        m_tip_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(m_lookup_timeout_sec));

      PathSample sample;
      sample.stamp = transform.header.stamp;
      sample.transform = transform.transform;
      m_samples.push_back(sample);

      const auto & p = sample.transform.translation;
      const auto & q = sample.transform.rotation;
      RCLCPP_INFO(
        get_logger(),
        "sample %zu (%s): p=[%.4f %.4f %.4f], q=[%.4f %.4f %.4f %.4f]",
        m_samples.size(), reason.c_str(), p.x, p.y, p.z, q.x, q.y, q.z, q.w);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "cannot sample TF %s <- %s: %s",
        m_base_frame.c_str(), m_tip_frame.c_str(), ex.what());
      return false;
    }
  }

  void undoLastSample()
  {
    if (m_samples.empty()) {
      RCLCPP_WARN(get_logger(), "no sample to undo");
      return;
    }

    m_samples.pop_back();
    RCLCPP_INFO(get_logger(), "undo last sample, remaining: %zu", m_samples.size());
  }

  void saveSamples()
  {
    YAML::Node root;
    root["base_frame"] = m_base_frame;
    root["tip_frame"] = m_tip_frame;
    root["sample_count"] = static_cast<int>(m_samples.size());

    YAML::Node samples_node(YAML::NodeType::Sequence);
    for (std::size_t i = 0; i < m_samples.size(); ++i) {
      const auto & sample = m_samples[i];
      const auto & p = sample.transform.translation;
      const auto & q = sample.transform.rotation;

      YAML::Node item;
      item["index"] = static_cast<int>(i);
      item["stamp_sec"] = sample.stamp.seconds();
      item["translation"] = std::vector<double>{p.x, p.y, p.z};
      item["rotation_xyzw"] = std::vector<double>{q.x, q.y, q.z, q.w};
      samples_node.push_back(item);
    }
    root["samples"] = samples_node;

    try {
      const std::filesystem::path output_path(m_output_file);
      if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
      }

      std::ofstream output(m_output_file);
      if (!output.is_open()) {
        RCLCPP_ERROR(get_logger(), "failed to open output file: %s", m_output_file.c_str());
        return;
      }

      output << root;
      output << "\n";
      RCLCPP_INFO(get_logger(), "saved %zu samples to %s", m_samples.size(), m_output_file.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "failed to save %s: %s", m_output_file.c_str(), ex.what());
    }
  }

  bool isPressed(const sensor_msgs::msg::Joy & msg, int index) const
  {
    return index >= 0 &&
           static_cast<std::size_t>(index) < msg.buttons.size() &&
           msg.buttons[static_cast<std::size_t>(index)] != 0;
  }

  bool risingEdge(int index, bool pressed) const
  {
    if (!pressed || index < 0) {
      return false;
    }
    const auto button_index = static_cast<std::size_t>(index);
    if (button_index >= m_previous_buttons.size()) {
      return true;
    }
    return m_previous_buttons[button_index] == 0;
  }

  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_buttons_sub;

  std::string m_base_frame;
  std::string m_tip_frame;
  std::string m_buttons_topic;
  std::string m_output_file;
  int m_sample_button{0};
  int m_record_button{1};
  int m_undo_button{2};
  int m_save_button{3};
  double m_lookup_timeout_sec{0.2};
  double m_min_record_interval_sec{0.05};
  bool m_recording{false};
  rclcpp::Time m_last_record_sample_time{0, 0, RCL_ROS_TIME};
  std::vector<int32_t> m_previous_buttons;
  std::vector<PathSample> m_samples;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollectPathNode>());
  rclcpp::shutdown();
  return 0;
}
