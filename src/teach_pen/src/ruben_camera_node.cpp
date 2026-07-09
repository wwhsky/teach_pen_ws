#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <ruben_msgs/srv/ru_ben_image_and_point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ruben_sdk_lib/ruben_sdk.h"

class RubenCameraNode : public rclcpp::Node
{
public:
  using CaptureService = ruben_msgs::srv::RuBenImageAndPointCloud;

  RubenCameraNode()
  : Node("ruben_camera_node")
  {
    m_frame_id = declare_parameter<std::string>("frame_id", "seam_camera_frame");
    m_capture_service_name =
      declare_parameter<std::string>("capture_service", "/seam_camera/capture");
    m_rgb_topic = declare_parameter<std::string>("rgb_topic", "/seam_camera/rgb_image");
    m_point_cloud_topic =
      declare_parameter<std::string>("point_cloud_topic", "/seam_camera/roi_points");
    m_capture_once_on_start = declare_parameter<bool>("capture_once_on_start", false);

    m_camera_sdk = std::make_unique<RubenSDK>();
    if (!m_camera_sdk->Connect()) {
      RCLCPP_ERROR(get_logger(), "failed to connect Ruben camera");
      return;
    }

    if (!m_camera_sdk->Open()) {
      RCLCPP_ERROR(get_logger(), "failed to open Ruben camera");
      m_camera_sdk->Disconnect();
      return;
    }

    m_camera_ready = true;
    RCLCPP_INFO(get_logger(), "Ruben camera ready, SN: %s", m_camera_sdk->m_device_info.sn);

    m_rgb_pub = create_publisher<sensor_msgs::msg::Image>(m_rgb_topic, 10);
    m_point_cloud_pub =
      create_publisher<sensor_msgs::msg::PointCloud2>(m_point_cloud_topic, rclcpp::SensorDataQoS());
    m_capture_service = create_service<CaptureService>(
      m_capture_service_name,
      std::bind(
        &RubenCameraNode::captureServiceCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    if (m_capture_once_on_start) {
      m_start_capture_timer = create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
          m_start_capture_timer->cancel();
          sensor_msgs::msg::Image rgb_msg;
          sensor_msgs::msg::PointCloud2 cloud_msg;
          captureFrame(rgb_msg, cloud_msg);
        });
    }
  }

  ~RubenCameraNode() override
  {
    if (m_camera_sdk && m_camera_sdk->IsOpened()) {
      m_camera_sdk->Close();
    }
    if (m_camera_sdk && m_camera_sdk->IsConnected()) {
      m_camera_sdk->Disconnect();
    }
  }

private:
  void captureServiceCallback(
    const std::shared_ptr<CaptureService::Request> request,
    std::shared_ptr<CaptureService::Response> response)
  {
    if (!request->input_str.empty() && request->input_str != "capture") {
      RCLCPP_WARN(
        get_logger(),
        "unsupported capture command: %s",
        request->input_str.c_str());
      return;
    }

    sensor_msgs::msg::Image rgb_msg;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    if (!captureFrame(rgb_msg, cloud_msg)) {
      return;
    }

    response->rgb_image = rgb_msg;
    response->point_cloud = cloud_msg;
  }

  bool captureFrame(
    sensor_msgs::msg::Image & rgb_msg,
    sensor_msgs::msg::PointCloud2 & cloud_msg)
  {
    if (!m_camera_ready || !m_camera_sdk) {
      RCLCPP_ERROR(get_logger(), "Ruben camera is not ready");
      return false;
    }

    if (!m_camera_sdk->Capture3D()) {
      RCLCPP_ERROR(get_logger(), "Ruben Capture3D failed");
      return false;
    }

    const auto stamp = now();

    const cv::Mat rgb = m_camera_sdk->GetImage(CameraNumber::RGB);
    if (rgb.empty()) {
      RCLCPP_WARN(get_logger(), "Capture3D succeeded, but RGB image is empty");
    } else {
      rgb_msg = matToRosImage(rgb, "bgr8", stamp);
      m_rgb_pub->publish(rgb_msg);
    }

    const auto point_cloud = m_camera_sdk->GetPointCloud();
    if (!point_cloud || point_cloud->empty()) {
      RCLCPP_ERROR(get_logger(), "Capture3D succeeded, but point cloud is empty");
      return false;
    }

    pcl::toROSMsg(*point_cloud, cloud_msg);
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = m_frame_id;
    m_point_cloud_pub->publish(cloud_msg);

    RCLCPP_INFO(
      get_logger(),
      "captured Ruben frame: image=%dx%d points=%zu frame=%s",
      rgb.cols,
      rgb.rows,
      point_cloud->size(),
      m_frame_id.c_str());
    return true;
  }

  sensor_msgs::msg::Image matToRosImage(
    const cv::Mat & image,
    const std::string & encoding,
    const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = m_frame_id;
    msg.height = static_cast<uint32_t>(image.rows);
    msg.width = static_cast<uint32_t>(image.cols);
    msg.encoding = encoding;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);

    const std::size_t data_size = image.step * static_cast<std::size_t>(image.rows);
    msg.data.resize(data_size);
    std::memcpy(msg.data.data(), image.data, data_size);
    return msg;
  }

  std::unique_ptr<RubenSDK> m_camera_sdk;
  bool m_camera_ready{false};
  std::string m_frame_id;
  std::string m_capture_service_name;
  std::string m_rgb_topic;
  std::string m_point_cloud_topic;
  bool m_capture_once_on_start{false};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_rgb_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_cloud_pub;
  rclcpp::Service<CaptureService>::SharedPtr m_capture_service;
  rclcpp::TimerBase::SharedPtr m_start_capture_timer;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RubenCameraNode>());
  rclcpp::shutdown();
  return 0;
}
