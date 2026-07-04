#include <cmath>
#include <atomic>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

struct PathSample
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Transform T_base_tip;
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
    m_output_file = declare_parameter<std::string>("output_file", "config/paths/demo_path.yaml");
    m_lookup_timeout_sec = declare_parameter<double>("lookup_timeout_sec", 0.2); // TF查找超时时间
    m_min_record_interval_sec = declare_parameter<double>("min_record_interval_sec", 0.05); // 记录模式下的采样周期

    startKeyboardReader();
    m_record_timer = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&CollectPathNode::recordTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Collect path from TF %s <- %s. keys: Space=sample r=record u=undo s=save q=quit",
      m_base_frame.c_str(), m_tip_frame.c_str());
  }

  ~CollectPathNode() override
  {
    m_keyboard_running = false;
    if (m_keyboard_thread.joinable()) {
      m_keyboard_thread.join();
    }
    restoreTerminal();
  }

private:
  void startKeyboardReader()
  {
    if (!isatty(STDIN_FILENO)) {
      RCLCPP_WARN(get_logger(), "stdin is not a TTY, keyboard control is disabled");
      return;
    }

    if (tcgetattr(STDIN_FILENO, &m_original_terminal) != 0) {
      RCLCPP_WARN(get_logger(), "failed to read terminal settings, keyboard control is disabled");
      return;
    }

    m_terminal_configured = true;
    auto raw_terminal = m_original_terminal;
    raw_terminal.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw_terminal.c_cc[VMIN] = 0;
    raw_terminal.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw_terminal) != 0) {
      m_terminal_configured = false;
      RCLCPP_WARN(get_logger(), "failed to configure terminal, keyboard control is disabled");
      return;
    }

    m_keyboard_running = true;
    m_keyboard_thread = std::thread(&CollectPathNode::keyboardLoop, this);
  }

  void restoreTerminal()
  {
    if (m_terminal_configured) {
      tcsetattr(STDIN_FILENO, TCSANOW, &m_original_terminal);
      m_terminal_configured = false;
    }
  }

  void keyboardLoop()
  {
    while (m_keyboard_running && rclcpp::ok()) {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(STDIN_FILENO, &read_fds);

      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;

      const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
      if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
        continue;
      }

      char key = '\0';
      if (read(STDIN_FILENO, &key, 1) != 1) {
        continue;
      }

      handleKey(key);
    }
  }

  void handleKey(char key)
  {
    key = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));

    if (key == ' ') {
      sampleOnce("key");
    } else if (key == 'r') {
      toggleRecording();
    } else if (key == 'u') {
      undoLastSample();
    } else if (key == 's') {
      saveSamples();
    } else if (key == 'q') {
      RCLCPP_INFO(get_logger(), "quit requested from keyboard");
      rclcpp::shutdown();
    }
  }

  void toggleRecording()
  {
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      m_recording = !m_recording;
      m_last_record_sample_time = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
    RCLCPP_INFO(get_logger(), "continuous recording: %s", isRecording() ? "on" : "off");
  }

  bool isRecording()
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_recording;
  }

  void recordTimerCallback()
  {
    if (!isRecording()) {
      return;
    }

    const auto now_time = now();
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      if (m_last_record_sample_time.nanoseconds() != 0 &&
        (now_time - m_last_record_sample_time).seconds() < m_min_record_interval_sec)
      {
        return;
      }
    }

    if (sampleOnce("record")) {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      m_last_record_sample_time = now_time;
    }
  }

  bool sampleOnce(const std::string & reason)
  {
    try {
      // T_A_B means the pose of frame B in frame A.
      const auto T_base_tip_msg = m_tf_buffer->lookupTransform(
        m_base_frame,
        m_tip_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(m_lookup_timeout_sec));

      PathSample sample;
      sample.stamp = T_base_tip_msg.header.stamp;
      sample.T_base_tip = T_base_tip_msg.transform;

      std::size_t sample_count = 0;
      {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        m_samples.push_back(sample);
        sample_count = m_samples.size();
      }

      const auto & p = sample.T_base_tip.translation;
      const auto & q = sample.T_base_tip.rotation;
      RCLCPP_INFO(
        get_logger(),
        "sample %zu (%s): p=[%.4f %.4f %.4f], q=[%.4f %.4f %.4f %.4f]",
        sample_count, reason.c_str(), p.x, p.y, p.z, q.x, q.y, q.z, q.w);
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
    std::lock_guard<std::mutex> lock(m_state_mutex);
    if (m_samples.empty()) {
      RCLCPP_WARN(get_logger(), "no sample to undo");
      return;
    }

    m_samples.pop_back();
    RCLCPP_INFO(get_logger(), "undo last sample, remaining: %zu", m_samples.size());
  }

  void saveSamples()
  {
    std::vector<PathSample> samples;
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      samples = m_samples;
    }

    YAML::Node root;
    root["base_frame"] = m_base_frame;
    root["tip_frame"] = m_tip_frame;
    root["sample_count"] = static_cast<int>(samples.size());

    YAML::Node samples_node(YAML::NodeType::Sequence);
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const auto & sample = samples[i];
      const auto & p = sample.T_base_tip.translation;
      const auto & q = sample.T_base_tip.rotation;

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
      RCLCPP_INFO(get_logger(), "saved %zu samples to %s", samples.size(), m_output_file.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "failed to save %s: %s", m_output_file.c_str(), ex.what());
    }
  }

  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;
  rclcpp::TimerBase::SharedPtr m_record_timer;
  std::thread m_keyboard_thread;
  std::atomic_bool m_keyboard_running{false};
  termios m_original_terminal{};
  bool m_terminal_configured{false};

  std::string m_base_frame;
  std::string m_tip_frame;
  std::string m_output_file;
  double m_lookup_timeout_sec{0.2};
  double m_min_record_interval_sec{0.05};
  std::mutex m_state_mutex;
  bool m_recording{false};
  rclcpp::Time m_last_record_sample_time{0, 0, RCL_ROS_TIME};
  std::vector<PathSample> m_samples;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollectPathNode>());
  rclcpp::shutdown();
  return 0;
}
