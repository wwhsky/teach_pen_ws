#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

namespace
{

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;

struct PlaneFit
{
  Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
  double d{0.0};
  int inlier_count{0};
  bool valid{false};
};

struct DetectionOutput
{
  sensor_msgs::msg::PointCloud2 source_msg;
  Eigen::Vector3d line_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d line_direction{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d start_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d end_point{Eigen::Vector3d::Zero()};
  CloudPtr plane_a_cloud;
  CloudPtr plane_b_cloud;
  bool valid{false};
};

struct StaticTransform
{
  std::string parent_frame;
  std::string child_frame;
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  bool valid{false};
};

Eigen::Vector3d normalizeOrZero(const Eigen::Vector3d & value)
{
  const double norm = value.norm();
  if (norm < 1e-9) {
    return Eigen::Vector3d::Zero();
  }
  return value / norm;
}

bool intersectPlanes(
  const PlaneFit & a,
  const PlaneFit & b,
  Eigen::Vector3d & point,
  Eigen::Vector3d & direction)
{
  direction = a.normal.cross(b.normal);
  const double denom = direction.squaredNorm();
  if (denom < 1e-10) {
    return false;
  }

  // 平面方程为 n.dot(x) + d = 0。这个闭式解给出交线上距离原点最近的点。
  point = ((b.d * a.normal - a.d * b.normal).cross(direction)) / denom;
  direction = normalizeOrZero(direction);
  return direction.norm() > 0.0;
}

}  // namespace

class SeamPerceptionNode : public rclcpp::Node
{
public:
  SeamPerceptionNode()
  : Node("seam_perception_node"),
    m_tf_buffer(std::make_unique<tf2_ros::Buffer>(get_clock())),
    m_tf_listener(std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer))
  {
    m_input_cloud_topic =
      declare_parameter<std::string>("input_cloud_topic", "/seam_camera/roi_points");
    m_input_file = declare_parameter<std::string>("input_file", "");
    m_input_file_frame = declare_parameter<std::string>("input_file_frame", "seam_camera_frame");
    m_target_frame = declare_parameter<std::string>("target_frame", "robot_base");
    m_teaching_path_file =
      declare_parameter<std::string>("teaching_path_file", "config/paths/demo_path.yaml");
    m_output_path_file =
      declare_parameter<std::string>("output_path_file", "config/paths/detected_seam_path.yaml");
    m_path_orientation_file =
      declare_parameter<std::string>("path_orientation_file", "config/paths/photo_pose.yaml");
    m_hand_eye_file =
      declare_parameter<std::string>("hand_eye_file", "config/calibration/camera_hand_eye.yaml");
    m_measured_path_topic =
      declare_parameter<std::string>("measured_path_topic", "/seam_tracking/measured_path");
    m_debug_cloud_topic =
      declare_parameter<std::string>("debug_cloud_topic", "/seam_tracking/debug_cloud");
    m_distance_threshold = declare_parameter<double>("distance_threshold", 0.002);
    m_voxel_leaf_size = declare_parameter<double>("voxel_leaf_size", 0.001);
    m_min_plane_inliers = declare_parameter<int>("min_plane_inliers", 100);
    m_path_roi_radius = declare_parameter<double>("path_roi_radius", 0.05);
    m_max_iterations = declare_parameter<int>("max_iterations", 200);
    m_min_plane_angle_deg = declare_parameter<double>("min_plane_angle_deg", 20.0);
    m_max_second_plane_candidates = declare_parameter<int>("max_second_plane_candidates", 8);
    m_line_length = declare_parameter<double>("line_length", 0.3);
    m_line_support_radius = declare_parameter<double>("line_support_radius", 0.008);
    m_line_bin_size = declare_parameter<double>("line_bin_size", 0.003);
    m_min_points_per_bin = declare_parameter<int>("min_points_per_bin", 2);
    m_min_segment_length = declare_parameter<double>("min_segment_length", 0.02);
    m_publish_debug_cloud = declare_parameter<bool>("publish_debug_cloud", true);
    m_save_output_path = declare_parameter<bool>("save_output_path", true);
    m_republish_interval_sec = declare_parameter<double>("republish_interval_sec", 1.0);

    loadTeachingPath();
    loadHandEyeTransform();
    loadPathOrientation();

    m_cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      m_input_cloud_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&SeamPerceptionNode::cloudCallback, this, std::placeholders::_1));

    m_path_pub = create_publisher<nav_msgs::msg::Path>(m_measured_path_topic, 10);
    m_debug_cloud_pub =
      create_publisher<sensor_msgs::msg::PointCloud2>(m_debug_cloud_topic, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "Seam perception started. input=%s measured_path=%s",
      m_input_cloud_topic.c_str(), m_measured_path_topic.c_str());

    if (!m_input_file.empty()) {
      processInputFile();
      if (m_last_output.valid && m_republish_interval_sec > 0.0) {
        m_republish_timer = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(m_republish_interval_sec)),
          std::bind(&SeamPerceptionNode::republishLastOutput, this));
      }
    }
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    CloudPtr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    processCloud(cloud, *msg);
  }

  void processInputFile()
  {
    CloudPtr cloud(new CloudT);
    const int load_result = loadPointCloudFile(m_input_file, cloud);
    if (load_result != 0) {
      RCLCPP_ERROR(get_logger(), "failed to load point cloud file: %s", m_input_file.c_str());
      return;
    }

    sensor_msgs::msg::PointCloud2 source_msg;
    pcl::toROSMsg(*cloud, source_msg);
    source_msg.header.stamp = now();
    source_msg.header.frame_id = m_input_file_frame;

    RCLCPP_INFO(
      get_logger(),
      "loaded point cloud file %s with %zu points",
      m_input_file.c_str(), cloud->size());
    processCloud(cloud, source_msg);
  }

  int loadPointCloudFile(const std::string & path, CloudPtr & cloud) const
  {
    const std::string lower_path = toLower(path);
    if (endsWith(lower_path, ".ply")) {
      return pcl::io::loadPLYFile<PointT>(path, *cloud);
    }
    if (endsWith(lower_path, ".pcd")) {
      return pcl::io::loadPCDFile<PointT>(path, *cloud);
    }

    RCLCPP_ERROR(get_logger(), "unsupported point cloud file extension: %s", path.c_str());
    return -1;
  }

  void processCloud(CloudPtr cloud, const sensor_msgs::msg::PointCloud2 & source_msg)
  {
    sensor_msgs::msg::PointCloud2 working_msg = source_msg;
    CloudPtr working_cloud = transformCloudToTargetFrame(cloud, working_msg);
    if (!working_cloud) {
      return;
    }

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*working_cloud, *working_cloud, valid_indices);
    if (working_cloud->empty()) {
      RCLCPP_WARN(get_logger(), "input cloud is empty after NaN removal");
      return;
    }

    working_cloud = downsample(working_cloud);
    working_cloud = cropCloudByTeachingPath(working_cloud);
    if (!working_cloud) {
      return;
    }

    if (working_cloud->size() < static_cast<std::size_t>(m_min_plane_inliers * 2)) {
      RCLCPP_WARN(
        get_logger(),
        "not enough points for two-plane fitting: %zu",
        working_cloud->size());
      return;
    }

    CloudPtr remaining(new CloudT);
    *remaining = *working_cloud;

    PlaneFit plane_a;
    CloudPtr plane_a_cloud(new CloudT);
    if (!fitAndExtractPlane(remaining, plane_a, plane_a_cloud)) {
      RCLCPP_WARN(get_logger(), "failed to fit first plane");
      return;
    }

    PlaneFit plane_b;
    CloudPtr plane_b_cloud(new CloudT);
    if (!fitSecondPlaneWithAngle(remaining, plane_a, plane_b, plane_b_cloud)) {
      RCLCPP_WARN(get_logger(), "failed to fit second plane");
      return;
    }

    Eigen::Vector3d line_point;
    Eigen::Vector3d line_direction;
    if (!intersectPlanes(plane_a, plane_b, line_point, line_direction)) {
      RCLCPP_WARN(get_logger(), "two fitted planes are nearly parallel");
      return;
    }

    Eigen::Vector3d start_point;
    Eigen::Vector3d end_point;
    const bool endpoints_ok = computeProjectedEndpoints(
      line_point, line_direction, plane_a_cloud, plane_b_cloud, start_point, end_point);
    if (!endpoints_ok) {
      const Eigen::Vector3d half = 0.5 * m_line_length * line_direction;
      start_point = line_point - half;
      end_point = line_point + half;
      RCLCPP_WARN(
        get_logger(),
        "failed to compute projected endpoints, fallback to fixed line_length=%.4f",
        m_line_length);
    }

    m_last_output.source_msg = working_msg;
    m_last_output.line_point = line_point;
    m_last_output.line_direction = line_direction;
    m_last_output.start_point = start_point;
    m_last_output.end_point = end_point;
    m_last_output.plane_a_cloud = plane_a_cloud;
    m_last_output.plane_b_cloud = plane_b_cloud;
    m_last_output.valid = true;
    if (m_save_output_path) {
      saveDetectedPath(working_msg, start_point, end_point);
    }
    republishLastOutput();

    RCLCPP_INFO(
      get_logger(),
      "seam line fitted: plane_inliers=[%d,%d], angle=%.2f deg, point=[%.4f %.4f %.4f], dir=[%.4f %.4f %.4f], start=[%.4f %.4f %.4f], end=[%.4f %.4f %.4f]",
      plane_a.inlier_count, plane_b.inlier_count,
      planeAngleDeg(plane_a, plane_b),
      line_point.x(), line_point.y(), line_point.z(),
      line_direction.x(), line_direction.y(), line_direction.z(),
      start_point.x(), start_point.y(), start_point.z(),
      end_point.x(), end_point.y(), end_point.z());
  }

  bool loadTeachingPath()
  {
    if (m_teaching_path_file.empty()) {
      return true;
    }

    YAML::Node root;
    try {
      std::ifstream input(m_teaching_path_file);
      if (!input.is_open()) {
        RCLCPP_WARN(get_logger(), "teaching path file not found: %s", m_teaching_path_file.c_str());
        return false;
      }
      root = YAML::Load(input);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "failed to load teaching path %s: %s", m_teaching_path_file.c_str(), ex.what());
      return false;
    }

    if (root["base_frame"]) {
      m_teaching_path_frame = root["base_frame"].as<std::string>();
    }
    if (!root["samples"]) {
      RCLCPP_WARN(get_logger(), "teaching path has no samples: %s", m_teaching_path_file.c_str());
      return false;
    }

    m_teaching_points.clear();
    for (const auto & item : root["samples"]) {
      const auto translation = item["translation"].as<std::vector<double>>();
      if (translation.size() != 3) {
        RCLCPP_ERROR(get_logger(), "teaching path translation must have 3 values");
        m_teaching_points.clear();
        return false;
      }
      m_teaching_points.emplace_back(translation[0], translation[1], translation[2]);
    }

    RCLCPP_INFO(
      get_logger(),
      "loaded teaching path: %zu points frame=%s file=%s",
      m_teaching_points.size(),
      m_teaching_path_frame.c_str(),
      m_teaching_path_file.c_str());
    return true;
  }

  bool loadHandEyeTransform()
  {
    if (m_hand_eye_file.empty()) {
      return true;
    }

    const std::filesystem::path path(m_hand_eye_file);
    if (!std::filesystem::exists(path)) {
      RCLCPP_WARN(get_logger(), "hand-eye calibration file not found: %s", m_hand_eye_file.c_str());
      return false;
    }

    YAML::Node root;
    try {
      std::ifstream input(m_hand_eye_file);
      root = YAML::Load(input);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "failed to load hand-eye file %s: %s", m_hand_eye_file.c_str(), ex.what());
      return false;
    }

    const YAML::Node data = root["calibration_result"] ? root["calibration_result"] : root;
    if (!data["parent_frame"] || !data["child_frame"] ||
      !data["translation"] || !data["rotation_xyzw"])
    {
      RCLCPP_ERROR(get_logger(), "hand-eye file missing parent_frame/child_frame/translation/rotation_xyzw");
      return false;
    }

    const auto translation = data["translation"].as<std::vector<double>>();
    const auto rotation = data["rotation_xyzw"].as<std::vector<double>>();
    if (translation.size() != 3 || rotation.size() != 4) {
      RCLCPP_ERROR(get_logger(), "hand-eye translation or rotation size is invalid");
      return false;
    }

    Eigen::Quaterniond q(rotation[3], rotation[0], rotation[1], rotation[2]);
    if (q.norm() < 1e-12) {
      RCLCPP_ERROR(get_logger(), "hand-eye quaternion is invalid");
      return false;
    }
    q.normalize();

    m_hand_eye.parent_frame = data["parent_frame"].as<std::string>();
    m_hand_eye.child_frame = data["child_frame"].as<std::string>();
    m_hand_eye.transform = Eigen::Isometry3d::Identity();
    m_hand_eye.transform.linear() = q.toRotationMatrix();
    m_hand_eye.transform.translation() = Eigen::Vector3d(
      translation[0],
      translation[1],
      translation[2]);
    m_hand_eye.valid = true;

    RCLCPP_INFO(
      get_logger(),
      "loaded hand-eye transform %s -> %s from %s",
      m_hand_eye.parent_frame.c_str(),
      m_hand_eye.child_frame.c_str(),
      m_hand_eye_file.c_str());
    return true;
  }

  bool loadPathOrientation()
  {
    if (m_path_orientation_file.empty()) {
      return true;
    }

    YAML::Node root;
    try {
      std::ifstream input(m_path_orientation_file);
      if (!input.is_open()) {
        RCLCPP_WARN(
          get_logger(),
          "path orientation file not found, using identity orientation: %s",
          m_path_orientation_file.c_str());
        return false;
      }
      root = YAML::Load(input);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        get_logger(),
        "failed to load path orientation file %s, using identity orientation: %s",
        m_path_orientation_file.c_str(),
        ex.what());
      return false;
    }

    if (!root["samples"] || !root["samples"][0] || !root["samples"][0]["rotation_xyzw"]) {
      RCLCPP_WARN(
        get_logger(),
        "path orientation file has no samples[0].rotation_xyzw, using identity orientation: %s",
        m_path_orientation_file.c_str());
      return false;
    }

    const auto rotation = root["samples"][0]["rotation_xyzw"].as<std::vector<double>>();
    if (rotation.size() != 4) {
      RCLCPP_WARN(
        get_logger(),
        "path orientation rotation_xyzw must have 4 values, using identity orientation");
      return false;
    }

    const double norm = std::sqrt(
      rotation[0] * rotation[0] +
      rotation[1] * rotation[1] +
      rotation[2] * rotation[2] +
      rotation[3] * rotation[3]);
    if (norm < 1e-12) {
      RCLCPP_WARN(get_logger(), "path orientation quaternion is invalid, using identity orientation");
      return false;
    }

    m_path_orientation_xyzw = {
      rotation[0] / norm,
      rotation[1] / norm,
      rotation[2] / norm,
      rotation[3] / norm};
    m_path_orientation_loaded = true;
    RCLCPP_INFO(
      get_logger(),
      "loaded path orientation from %s: [%.4f %.4f %.4f %.4f]",
      m_path_orientation_file.c_str(),
      m_path_orientation_xyzw[0],
      m_path_orientation_xyzw[1],
      m_path_orientation_xyzw[2],
      m_path_orientation_xyzw[3]);
    return true;
  }

  CloudPtr transformCloudToTargetFrame(
    const CloudPtr & cloud,
    sensor_msgs::msg::PointCloud2 & source_msg)
  {
    const std::string source_frame = source_msg.header.frame_id;
    if (m_target_frame.empty() || source_frame.empty() || source_frame == m_target_frame) {
      source_msg.header.frame_id = source_frame.empty() ? m_target_frame : source_frame;
      return cloud;
    }

    Eigen::Isometry3d T_target_source;
    if (!lookupCloudTransform(source_frame, T_target_source)) {
      RCLCPP_ERROR(
        get_logger(),
        "failed to transform point cloud from %s to %s",
        source_frame.c_str(),
        m_target_frame.c_str());
      return nullptr;
    }

    CloudPtr transformed(new CloudT);
    transformed->reserve(cloud->size());
    transformed->header = cloud->header;
    transformed->is_dense = cloud->is_dense;
    for (const auto & point : cloud->points) {
      const Eigen::Vector3d p = T_target_source * Eigen::Vector3d(point.x, point.y, point.z);
      transformed->points.emplace_back(
        static_cast<float>(p.x()),
        static_cast<float>(p.y()),
        static_cast<float>(p.z()));
    }
    transformed->width = static_cast<std::uint32_t>(transformed->points.size());
    transformed->height = 1;
    source_msg.header.frame_id = m_target_frame;
    RCLCPP_INFO(
      get_logger(),
      "transformed point cloud %s -> %s with %zu points",
      source_frame.c_str(),
      m_target_frame.c_str(),
      transformed->size());
    return transformed;
  }

  bool lookupCloudTransform(
    const std::string & source_frame,
    Eigen::Isometry3d & T_target_source)
  {
    try {
      const auto transform_msg = m_tf_buffer->lookupTransform(
        m_target_frame,
        source_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(0.2));
      T_target_source = transformMsgToEigen(transform_msg.transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "TF lookup %s <- %s failed, fallback to hand-eye YAML if possible: %s",
        m_target_frame.c_str(),
        source_frame.c_str(),
        ex.what());
    }

    if (!m_hand_eye.valid || m_hand_eye.child_frame != source_frame) {
      return false;
    }

    if (m_hand_eye.parent_frame == m_target_frame) {
      T_target_source = m_hand_eye.transform;
      return true;
    }

    try {
      const auto transform_msg = m_tf_buffer->lookupTransform(
        m_target_frame,
        m_hand_eye.parent_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(0.2));
      T_target_source = transformMsgToEigen(transform_msg.transform) * m_hand_eye.transform;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        get_logger(),
        "TF lookup %s <- %s for hand-eye parent failed: %s",
        m_target_frame.c_str(),
        m_hand_eye.parent_frame.c_str(),
        ex.what());
      return false;
    }
  }

  Eigen::Isometry3d transformMsgToEigen(const geometry_msgs::msg::Transform & transform) const
  {
    Eigen::Quaterniond q(
      transform.rotation.w,
      transform.rotation.x,
      transform.rotation.y,
      transform.rotation.z);
    if (q.norm() < 1e-12) {
      q = Eigen::Quaterniond::Identity();
    }
    q.normalize();

    Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
    output.linear() = q.toRotationMatrix();
    output.translation() = Eigen::Vector3d(
      transform.translation.x,
      transform.translation.y,
      transform.translation.z);
    return output;
  }

  CloudPtr cropCloudByTeachingPath(const CloudPtr & cloud) const
  {
    if (m_teaching_points.empty() || m_path_roi_radius <= 0.0) {
      return cloud;
    }

    if (m_teaching_path_frame != m_target_frame) {
      RCLCPP_WARN(
        get_logger(),
        "teaching path frame %s does not match target frame %s; skip path ROI crop",
        m_teaching_path_frame.c_str(),
        m_target_frame.c_str());
      return cloud;
    }

    CloudPtr cropped(new CloudT);
    cropped->reserve(cloud->size());
    const double radius_sq = m_path_roi_radius * m_path_roi_radius;
    for (const auto & point : cloud->points) {
      const Eigen::Vector3d p(point.x, point.y, point.z);
      if (distanceToTeachingPathSquared(p) <= radius_sq) {
        cropped->points.push_back(point);
      }
    }

    cropped->width = static_cast<std::uint32_t>(cropped->points.size());
    cropped->height = 1;
    cropped->is_dense = cloud->is_dense;

    RCLCPP_INFO(
      get_logger(),
      "path ROI crop: %zu -> %zu points, radius=%.3f",
      cloud->size(),
      cropped->size(),
      m_path_roi_radius);

    if (cropped->empty()) {
      RCLCPP_WARN(get_logger(), "path ROI crop removed all points");
      return nullptr;
    }
    return cropped;
  }

  double distanceToTeachingPathSquared(const Eigen::Vector3d & p) const
  {
    if (m_teaching_points.size() == 1) {
      return (p - m_teaching_points.front()).squaredNorm();
    }

    double best = std::numeric_limits<double>::max();
    for (std::size_t i = 1; i < m_teaching_points.size(); ++i) {
      const Eigen::Vector3d a = m_teaching_points[i - 1];
      const Eigen::Vector3d b = m_teaching_points[i];
      const Eigen::Vector3d ab = b - a;
      const double ab_sq = ab.squaredNorm();
      double t = 0.0;
      if (ab_sq > 1e-12) {
        t = std::clamp((p - a).dot(ab) / ab_sq, 0.0, 1.0);
      }
      const Eigen::Vector3d closest = a + t * ab;
      best = std::min(best, (p - closest).squaredNorm());
    }
    return best;
  }

  void republishLastOutput()
  {
    if (!m_last_output.valid) {
      return;
    }

    auto source_msg = m_last_output.source_msg;
    source_msg.header.stamp = now();
    publishLinePath(source_msg, m_last_output.start_point, m_last_output.end_point);
    if (m_publish_debug_cloud) {
      publishDebugCloud(source_msg, m_last_output.plane_a_cloud, m_last_output.plane_b_cloud);
    }
  }

  std::string toLower(const std::string & value) const
  {
    std::string output = value;
    std::transform(output.begin(), output.end(), output.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return output;
  }

  bool endsWith(const std::string & value, const std::string & suffix) const
  {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
  }

  CloudPtr downsample(const CloudPtr & cloud) const
  {
    if (m_voxel_leaf_size <= 0.0) {
      CloudPtr copy(new CloudT);
      *copy = *cloud;
      return copy;
    }

    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(
      static_cast<float>(m_voxel_leaf_size),
      static_cast<float>(m_voxel_leaf_size),
      static_cast<float>(m_voxel_leaf_size));

    CloudPtr filtered(new CloudT);
    voxel.filter(*filtered);
    return filtered;
  }

  bool fitAndExtractPlane(
    CloudPtr & remaining,
    PlaneFit & plane,
    CloudPtr & plane_cloud) const
  {
    if (!remaining || remaining->size() < static_cast<std::size_t>(m_min_plane_inliers)) {
      return false;
    }

    pcl::SACSegmentation<PointT> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(m_max_iterations);
    segmentation.setDistanceThreshold(m_distance_threshold);
    segmentation.setInputCloud(remaining);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    segmentation.segment(*inliers, *coefficients);

    if (coefficients->values.size() < 4 ||
      inliers->indices.size() < static_cast<std::size_t>(m_min_plane_inliers))
    {
      return false;
    }

    Eigen::Vector3d normal(
      coefficients->values[0],
      coefficients->values[1],
      coefficients->values[2]);
    normal = normalizeOrZero(normal);
    if (normal.norm() == 0.0) {
      return false;
    }

    double d = coefficients->values[3];
    const double raw_norm = std::sqrt(
      coefficients->values[0] * coefficients->values[0] +
      coefficients->values[1] * coefficients->values[1] +
      coefficients->values[2] * coefficients->values[2]);
    if (raw_norm > 1e-9) {
      d /= raw_norm;
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(remaining);
    extract.setIndices(inliers);

    plane_cloud.reset(new CloudT);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    CloudPtr new_remaining(new CloudT);
    extract.setNegative(true);
    extract.filter(*new_remaining);
    remaining = new_remaining;

    plane.normal = normal;
    plane.d = d;
    plane.inlier_count = static_cast<int>(inliers->indices.size());
    plane.valid = true;
    return true;
  }

  bool fitSecondPlaneWithAngle(
    CloudPtr remaining,
    const PlaneFit & first_plane,
    PlaneFit & second_plane,
    CloudPtr & second_plane_cloud) const
  {
    for (int i = 0; i < m_max_second_plane_candidates; ++i) {
      PlaneFit candidate;
      CloudPtr candidate_cloud(new CloudT);
      if (!fitAndExtractPlane(remaining, candidate, candidate_cloud)) {
        return false;
      }

      const double angle_deg = planeAngleDeg(first_plane, candidate);
      if (angle_deg >= m_min_plane_angle_deg) {
        second_plane = candidate;
        second_plane_cloud = candidate_cloud;
        RCLCPP_INFO(
          get_logger(),
          "second plane accepted after %d candidate(s), angle=%.2f deg",
          i + 1, angle_deg);
        return true;
      }

      RCLCPP_INFO(
        get_logger(),
        "second plane candidate %d rejected: angle=%.2f deg < %.2f deg, inliers=%d",
        i + 1, angle_deg, m_min_plane_angle_deg, candidate.inlier_count);
    }

    return false;
  }

  double planeAngleDeg(const PlaneFit & a, const PlaneFit & b) const
  {
    const double dot = std::clamp(std::abs(a.normal.dot(b.normal)), 0.0, 1.0);
    return std::acos(dot) * 180.0 / M_PI;
  }

  bool computeProjectedEndpoints(
    const Eigen::Vector3d & line_point,
    const Eigen::Vector3d & line_direction,
    const CloudPtr & plane_a_cloud,
    const CloudPtr & plane_b_cloud,
    Eigen::Vector3d & start_point,
    Eigen::Vector3d & end_point) const
  {
    std::vector<double> support_a;
    std::vector<double> support_b;
    collectLineSupportProjections(plane_a_cloud, line_point, line_direction, support_a);
    collectLineSupportProjections(plane_b_cloud, line_point, line_direction, support_b);

    if (support_a.empty() || support_b.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "line support is empty: plane_a=%zu plane_b=%zu within radius=%.4f",
        support_a.size(), support_b.size(), m_line_support_radius);
      return false;
    }

    double min_s = std::numeric_limits<double>::max();
    double max_s = -std::numeric_limits<double>::max();
    updateProjectionRange(support_a, min_s, max_s);
    updateProjectionRange(support_b, min_s, max_s);
    if (max_s <= min_s || m_line_bin_size <= 0.0) {
      return false;
    }

    const int bin_count = static_cast<int>(std::ceil((max_s - min_s) / m_line_bin_size));
    if (bin_count <= 0) {
      return false;
    }

    std::vector<int> count_a(static_cast<std::size_t>(bin_count), 0);
    std::vector<int> count_b(static_cast<std::size_t>(bin_count), 0);
    fillSupportBins(support_a, min_s, count_a);
    fillSupportBins(support_b, min_s, count_b);

    int best_start = -1;
    int best_end = -1;
    int current_start = -1;

    for (int i = 0; i < bin_count; ++i) {
      const bool supported =
        count_a[static_cast<std::size_t>(i)] >= m_min_points_per_bin &&
        count_b[static_cast<std::size_t>(i)] >= m_min_points_per_bin;

      if (supported && current_start < 0) {
        current_start = i;
      }

      const bool end_of_segment = !supported || i == bin_count - 1;
      if (current_start >= 0 && end_of_segment) {
        const int current_end = supported && i == bin_count - 1 ? i : i - 1;
        if (best_start < 0 || (current_end - current_start) > (best_end - best_start)) {
          best_start = current_start;
          best_end = current_end;
        }
        current_start = -1;
      }
    }

    if (best_start < 0 || best_end < best_start) {
      RCLCPP_WARN(get_logger(), "no continuous line-support segment found");
      return false;
    }

    const double start_s = min_s + static_cast<double>(best_start) * m_line_bin_size;
    const double end_s =
      std::min(max_s, min_s + static_cast<double>(best_end + 1) * m_line_bin_size);
    if ((end_s - start_s) < m_min_segment_length) {
      RCLCPP_WARN(
        get_logger(),
        "line-support segment too short: %.4f < %.4f",
        end_s - start_s, m_min_segment_length);
      return false;
    }

    start_point = line_point + start_s * line_direction;
    end_point = line_point + end_s * line_direction;
    RCLCPP_INFO(
      get_logger(),
      "line-support segment: bins=%d best=[%d,%d] s=[%.4f, %.4f] length=%.4f support_points=[%zu,%zu]",
      bin_count, best_start, best_end, start_s, end_s, end_s - start_s,
      support_a.size(), support_b.size());
    return true;
  }

  void collectLineSupportProjections(
    const CloudPtr & cloud,
    const Eigen::Vector3d & line_point,
    const Eigen::Vector3d & line_direction,
    std::vector<double> & projections) const
  {
    if (!cloud) {
      return;
    }

    projections.reserve(projections.size() + cloud->size());
    for (const auto & point : cloud->points) {
      const Eigen::Vector3d p(point.x, point.y, point.z);
      const Eigen::Vector3d delta = p - line_point;
      const double s = line_direction.dot(delta);
      const Eigen::Vector3d closest = line_point + s * line_direction;
      const double distance_to_line = (p - closest).norm();
      if (distance_to_line <= m_line_support_radius) {
        projections.push_back(s);
      }
    }
  }

  void updateProjectionRange(
    const std::vector<double> & projections,
    double & min_s,
    double & max_s) const
  {
    for (const double s : projections) {
      min_s = std::min(min_s, s);
      max_s = std::max(max_s, s);
    }
  }

  void fillSupportBins(
    const std::vector<double> & projections,
    double min_s,
    std::vector<int> & counts) const
  {
    for (const double s : projections) {
      const int bin = static_cast<int>(std::floor((s - min_s) / m_line_bin_size));
      if (bin >= 0 && static_cast<std::size_t>(bin) < counts.size()) {
        counts[static_cast<std::size_t>(bin)] += 1;
      }
    }
  }

  void publishLinePath(
    const sensor_msgs::msg::PointCloud2 & source_msg,
    const Eigen::Vector3d & start_point,
    const Eigen::Vector3d & end_point)
  {
    nav_msgs::msg::Path path;
    path.header = source_msg.header;
    path.poses.reserve(2);

    const std::array<Eigen::Vector3d, 2> endpoints = {start_point, end_point};
    for (const auto & endpoint : endpoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = endpoint.x();
      pose.pose.position.y = endpoint.y();
      pose.pose.position.z = endpoint.z();
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    m_path_pub->publish(path);
  }

  void saveDetectedPath(
    const sensor_msgs::msg::PointCloud2 & source_msg,
    const Eigen::Vector3d & start_point,
    const Eigen::Vector3d & end_point)
  {
    if (m_output_path_file.empty()) {
      return;
    }

    YAML::Node root;
    root["base_frame"] = source_msg.header.frame_id;
    root["tip_frame"] = "welding_torch_tip";
    root["source_cloud_frame"] = source_msg.header.frame_id;
    root["source_teaching_path"] = m_teaching_path_file;
    root["sample_count"] = 2;

    const std::array<Eigen::Vector3d, 2> points = {start_point, end_point};
    YAML::Node samples_node(YAML::NodeType::Sequence);
    for (std::size_t i = 0; i < points.size(); ++i) {
      YAML::Node item;
      item["index"] = static_cast<int>(i);
      item["stamp_sec"] = now().seconds();
      item["translation"] = std::vector<double>{
        points[i].x(),
        points[i].y(),
        points[i].z()};
      item["rotation_xyzw"] = std::vector<double>{
        m_path_orientation_xyzw[0],
        m_path_orientation_xyzw[1],
        m_path_orientation_xyzw[2],
        m_path_orientation_xyzw[3]};
      samples_node.push_back(item);
    }
    root["samples"] = samples_node;

    try {
      const std::filesystem::path output_path(m_output_path_file);
      if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
      }

      std::ofstream output(m_output_path_file);
      if (!output.is_open()) {
        RCLCPP_ERROR(get_logger(), "failed to open output path file: %s", m_output_path_file.c_str());
        return;
      }

      output << root;
      output << "\n";
      RCLCPP_INFO(get_logger(), "saved detected seam path to %s", m_output_path_file.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(),
        "failed to save detected seam path %s: %s",
        m_output_path_file.c_str(),
        ex.what());
    }
  }

  void publishDebugCloud(
    const sensor_msgs::msg::PointCloud2 & source_msg,
    const CloudPtr & plane_a_cloud,
    const CloudPtr & plane_b_cloud)
  {
    CloudPtr debug_cloud(new CloudT);
    if (plane_a_cloud) {
      *debug_cloud += *plane_a_cloud;
    }
    if (plane_b_cloud) {
      *debug_cloud += *plane_b_cloud;
    }

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*debug_cloud, out);
    out.header = source_msg.header;
    m_debug_cloud_pub->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_debug_cloud_pub;
  rclcpp::TimerBase::SharedPtr m_republish_timer;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;

  std::string m_input_cloud_topic;
  std::string m_input_file;
  std::string m_input_file_frame;
  std::string m_target_frame;
  std::string m_teaching_path_file;
  std::string m_output_path_file;
  std::string m_path_orientation_file;
  std::string m_hand_eye_file;
  std::string m_measured_path_topic;
  std::string m_debug_cloud_topic;
  std::string m_teaching_path_frame{"robot_base"};
  double m_distance_threshold{0.002};
  double m_voxel_leaf_size{0.001};
  int m_min_plane_inliers{100};
  double m_path_roi_radius{0.05};
  int m_max_iterations{200};
  double m_min_plane_angle_deg{20.0};
  int m_max_second_plane_candidates{8};
  double m_line_length{0.3};
  double m_line_support_radius{0.008};
  double m_line_bin_size{0.003};
  int m_min_points_per_bin{2};
  double m_min_segment_length{0.02};
  double m_republish_interval_sec{1.0};
  bool m_publish_debug_cloud{true};
  bool m_save_output_path{true};
  bool m_path_orientation_loaded{false};
  std::array<double, 4> m_path_orientation_xyzw{0.0, 0.0, 0.0, 1.0};
  std::vector<Eigen::Vector3d> m_teaching_points;
  StaticTransform m_hand_eye;
  DetectionOutput m_last_output;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SeamPerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
