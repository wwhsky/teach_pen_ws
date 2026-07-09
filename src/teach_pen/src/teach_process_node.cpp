#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <yaml-cpp/yaml.h>

struct ProcessPathSample
{
  rclcpp::Time stamp;
  tf2::Vector3 position{0.0, 0.0, 0.0};
  tf2::Quaternion orientation{0.0, 0.0, 0.0, 1.0};
};

class TeachProcessNode : public rclcpp::Node
{
public:
  TeachProcessNode()
  : Node("teach_process_node")
  {
    m_input_file = declare_parameter<std::string>("input_file", "config/paths/demo_path.yaml");
    m_output_file = declare_parameter<std::string>("output_file", "config/paths/photo_pose.yaml");
    m_photo_pose_topic =
      declare_parameter<std::string>("photo_pose_topic", "/teach_process/photo_pose");
    m_photo_distance = declare_parameter<double>("photo_distance", 0.30);
    m_min_z_axis_consistency = declare_parameter<double>("min_z_axis_consistency", 0.70);
    m_max_z_axis_angle_deg = declare_parameter<double>("max_z_axis_angle_deg", 45.0);
    m_save_output = declare_parameter<bool>("save_output", true);
    const double publish_period_sec = declare_parameter<double>("publish_period_sec", 1.0);

    m_photo_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
      m_photo_pose_topic,
      rclcpp::QoS(1).reliable().transient_local());

    if (loadSamples() && computePhotoPose()) {
      if (m_save_output) {
        savePhotoPose();
      }

      publishPhotoPose();
      if (publish_period_sec > 0.0) {
        m_publish_timer = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(publish_period_sec)),
          std::bind(&TeachProcessNode::publishPhotoPose, this));
      }
    }
  }

private:
  bool loadSamples()
  {
    YAML::Node root;
    try {
      std::ifstream input(m_input_file);
      if (!input.is_open()) {
        RCLCPP_ERROR(get_logger(), "failed to open input file: %s", m_input_file.c_str());
        return false;
      }
      root = YAML::Load(input);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "failed to load %s: %s", m_input_file.c_str(), ex.what());
      return false;
    }

    if (root["base_frame"]) {
      m_base_frame = root["base_frame"].as<std::string>();
    }
    if (root["tip_frame"]) {
      m_tip_frame = root["tip_frame"].as<std::string>();
    }
    if (!root["samples"]) {
      RCLCPP_ERROR(get_logger(), "no samples found in %s", m_input_file.c_str());
      return false;
    }

    m_samples.clear();
    for (const auto & item : root["samples"]) {
      const auto translation = item["translation"].as<std::vector<double>>();
      const auto rotation = item["rotation_xyzw"].as<std::vector<double>>();
      if (translation.size() != 3 || rotation.size() != 4) {
        RCLCPP_ERROR(get_logger(), "invalid sample in %s", m_input_file.c_str());
        return false;
      }

      ProcessPathSample sample;
      const double stamp_sec = item["stamp_sec"] ? item["stamp_sec"].as<double>() : 0.0;
      sample.stamp = rclcpp::Time(
        static_cast<int64_t>(stamp_sec * 1e9),
        get_clock()->get_clock_type());
      sample.position = tf2::Vector3(translation[0], translation[1], translation[2]);
      sample.orientation = tf2::Quaternion(rotation[0], rotation[1], rotation[2], rotation[3]);
      if (sample.orientation.length2() < 1e-12) {
        RCLCPP_ERROR(get_logger(), "sample quaternion is invalid");
        return false;
      }
      sample.orientation.normalize();
      m_samples.push_back(sample);
    }

    if (m_samples.empty()) {
      RCLCPP_ERROR(get_logger(), "input path has no samples: %s", m_input_file.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "loaded %zu samples from %s", m_samples.size(), m_input_file.c_str());
    return true;
  }

  bool computePhotoPose()
  {
    tf2::Vector3 center(0.0, 0.0, 0.0);
    tf2::Vector3 z_sum(0.0, 0.0, 0.0);

    double qx_sum = 0.0;
    double qy_sum = 0.0;
    double qz_sum = 0.0;
    double qw_sum = 0.0;
    const tf2::Quaternion reference_q = m_samples.front().orientation;

    for (const auto & sample : m_samples) {
      center += sample.position;

      const tf2::Matrix3x3 rotation(sample.orientation);
      const tf2::Vector3 z_axis(rotation[0][2], rotation[1][2], rotation[2][2]);
      z_sum += z_axis.normalized();

      tf2::Quaternion q = sample.orientation;
      if (quaternionDot(reference_q, q) < 0.0) {
        q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
      }
      qx_sum += q.x();
      qy_sum += q.y();
      qz_sum += q.z();
      qw_sum += q.w();
    }

    const double sample_count = static_cast<double>(m_samples.size());
    center /= sample_count;

    const double z_sum_norm = z_sum.length();
    m_z_axis_consistency = z_sum_norm / sample_count;
    if (z_sum_norm < 1e-9 || m_z_axis_consistency < m_min_z_axis_consistency) {
      RCLCPP_ERROR(
        get_logger(),
        "teaching pen z-axis is not consistent enough: %.3f < %.3f",
        m_z_axis_consistency,
        m_min_z_axis_consistency);
      return false;
    }

    m_z_axis = z_sum / z_sum_norm;
    m_center = center;
    m_photo_position = center + m_photo_distance * m_z_axis;

    tf2::Quaternion average_q(qx_sum, qy_sum, qz_sum, qw_sum);
    if (average_q.length2() < 1e-12) {
      average_q = reference_q;
    }
    average_q.normalize();
    m_photo_orientation = average_q;

    const double max_angle_deg = maxZAxisAngleDeg();
    if (max_angle_deg > m_max_z_axis_angle_deg) {
      RCLCPP_WARN(
        get_logger(),
        "teaching pen z-axis max deviation is high: %.2f deg > %.2f deg",
        max_angle_deg,
        m_max_z_axis_angle_deg);
    }

    RCLCPP_INFO(
      get_logger(),
      "photo pose computed: center=[%.4f %.4f %.4f], z_avg=[%.4f %.4f %.4f], distance=%.3f, photo=[%.4f %.4f %.4f], consistency=%.3f, max_z_angle=%.2f deg",
      m_center.x(), m_center.y(), m_center.z(),
      m_z_axis.x(), m_z_axis.y(), m_z_axis.z(),
      m_photo_distance,
      m_photo_position.x(), m_photo_position.y(), m_photo_position.z(),
      m_z_axis_consistency,
      max_angle_deg);
    return true;
  }

  double maxZAxisAngleDeg() const
  {
    double max_angle = 0.0;
    for (const auto & sample : m_samples) {
      const tf2::Matrix3x3 rotation(sample.orientation);
      const tf2::Vector3 z_axis = tf2::Vector3(rotation[0][2], rotation[1][2], rotation[2][2])
                                    .normalized();
      const double dot = std::clamp(z_axis.dot(m_z_axis), -1.0, 1.0);
      max_angle = std::max(max_angle, std::acos(dot) * 180.0 / M_PI);
    }
    return max_angle;
  }

  double quaternionDot(const tf2::Quaternion & a, const tf2::Quaternion & b) const
  {
    return a.x() * b.x() + a.y() * b.y() + a.z() * b.z() + a.w() * b.w();
  }

  void publishPhotoPose()
  {
    if (m_samples.empty()) {
      return;
    }

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = m_base_frame;
    msg.pose.position.x = m_photo_position.x();
    msg.pose.position.y = m_photo_position.y();
    msg.pose.position.z = m_photo_position.z();
    msg.pose.orientation.x = m_photo_orientation.x();
    msg.pose.orientation.y = m_photo_orientation.y();
    msg.pose.orientation.z = m_photo_orientation.z();
    msg.pose.orientation.w = m_photo_orientation.w();
    m_photo_pose_pub->publish(msg);
  }

  void savePhotoPose()
  {
    YAML::Node root;
    root["base_frame"] = m_base_frame;
    root["tip_frame"] = "photo_pose";
    root["source_file"] = m_input_file;
    root["sample_count"] = 1;
    root["photo_distance"] = m_photo_distance;
    root["z_axis_consistency"] = m_z_axis_consistency;
    root["z_axis"] = std::vector<double>{m_z_axis.x(), m_z_axis.y(), m_z_axis.z()};
    root["center"] = std::vector<double>{m_center.x(), m_center.y(), m_center.z()};

    YAML::Node sample;
    sample["index"] = 0;
    sample["stamp_sec"] = now().seconds();
    sample["translation"] =
      std::vector<double>{m_photo_position.x(), m_photo_position.y(), m_photo_position.z()};
    sample["rotation_xyzw"] = std::vector<double>{
      m_photo_orientation.x(),
      m_photo_orientation.y(),
      m_photo_orientation.z(),
      m_photo_orientation.w()};

    YAML::Node samples_node(YAML::NodeType::Sequence);
    samples_node.push_back(sample);
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
      RCLCPP_INFO(get_logger(), "saved photo pose to %s", m_output_file.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "failed to save %s: %s", m_output_file.c_str(), ex.what());
    }
  }

  std::string m_input_file;
  std::string m_output_file;
  std::string m_photo_pose_topic;
  std::string m_base_frame{"robot_base"};
  std::string m_tip_frame{"teaching_pen_tip"};
  double m_photo_distance{0.30};
  double m_min_z_axis_consistency{0.70};
  double m_max_z_axis_angle_deg{45.0};
  bool m_save_output{true};

  std::vector<ProcessPathSample> m_samples;
  tf2::Vector3 m_center{0.0, 0.0, 0.0};
  tf2::Vector3 m_z_axis{0.0, 0.0, 1.0};
  tf2::Vector3 m_photo_position{0.0, 0.0, 0.0};
  tf2::Quaternion m_photo_orientation{0.0, 0.0, 0.0, 1.0};
  double m_z_axis_consistency{0.0};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_photo_pose_pub;
  rclcpp::TimerBase::SharedPtr m_publish_timer;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeachProcessNode>());
  rclcpp::shutdown();
  return 0;
}
