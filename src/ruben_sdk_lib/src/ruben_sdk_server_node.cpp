/**
 * @file      ruben_sdk_server_node.cpp
 * @brief     如本相机 ROS2 服务节点
 * @details   接收客户端采集请求，调用相机 SDK 获取 RGB 图像与点云数据，
 *            通过服务响应返回，并同步发布到 ROS2 话题。
 */

#include <memory>
#include <string>
#include <chrono>
#include <thread>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "RVC/RVC.h"
#include "ruben_sdk_lib/ruben_sdk.h"
#include "ruben_msgs/srv/ruben_service.hpp"

using RubenService = ruben_msgs::srv::RubenService;
using Request = RubenService::Request;
using Response = RubenService::Response;
using PointCloudXYZPtr = std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>;

class RubenCameraServerNode : public rclcpp::Node
{
public:
    explicit RubenCameraServerNode(const std::string & node_name)
        : Node(node_name)
    {
        declare_parameter<std::string>("camera_id", "");
        declare_parameter<std::string>("camera_paras", "");
        declare_parameter<std::string>("service_name", "/ruben/get_camera_data");
        declare_parameter<std::string>("rgb_topic", "/ruben/camera/rgb_image");
        declare_parameter<std::string>("pc_topic", "/ruben/camera/point_cloud");
        declare_parameter<std::string>("frame_id", "ruben_camera_link");
        declare_parameter<int>("retry_count", 3);
        declare_parameter<int>("retry_interval_ms", 1000);

        get_parameter("camera_id", camera_id_);
        get_parameter("camera_paras", camera_paras);
        get_parameter("service_name", service_name_);
        get_parameter("rgb_topic", rgb_topic_);
        get_parameter("pc_topic", point_cloud_topic_);
        get_parameter("frame_id", frame_id_);
        get_parameter("retry_count", retry_count_);
        get_parameter("retry_interval_ms", retry_interval_ms_);

        RCLCPP_INFO(this->get_logger(), "========== Ruben SDK Server Configuration ==========");
        RCLCPP_INFO(this->get_logger(), "camera_id    : %s", camera_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "camera_paras : %s", camera_paras.c_str());
        RCLCPP_INFO(this->get_logger(), "service_name : %s", service_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "rgb_topic    : %s", rgb_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "pc_topic     : %s", point_cloud_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "frame_id     : %s", frame_id_.c_str());

        if (!initialize_camera())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera.");
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Camera is ready. SN=%s",
            camera_sdk_->m_device_info.sn);

        camera_service_ = this->create_service<RubenService>(
            service_name_,
            std::bind(
                &RubenCameraServerNode::handle_service_request,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        rgb_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            rgb_topic_,
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());

        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic_,
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());

        RCLCPP_INFO(this->get_logger(), "Ruben SDK server node started successfully.");
    }

    ~RubenCameraServerNode() override
    {
        if (camera_sdk_ && camera_sdk_->IsOpened())
        {
            camera_sdk_->Disconnect();
            RCLCPP_INFO(this->get_logger(), "Camera disconnected.");
        }
    }

private:
    bool connect_camera()
    {
        RCLCPP_INFO(this->get_logger(), "Connecting to camera...");
        for (int i = 0; i < retry_count_; ++i)
        {
            if (camera_sdk_->Connect(camera_id_))
            {
                return true;
            }
            RCLCPP_WARN(this->get_logger(), "Connect failed, retry %d/%d", i + 1, retry_count_);
            std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms_));
        }

        RCLCPP_ERROR(this->get_logger(), "Failed to connect to camera.");
        return false;
    }

    bool open_camera()
    {
        RCLCPP_INFO(this->get_logger(), "Opening camera...");
        for (int i = 0; i < retry_count_; ++i)
        {
            if (camera_sdk_->Open(camera_paras))
            {
                camera_opened_ = true;
                return true;
            }
            RCLCPP_WARN(this->get_logger(), "Open failed, retry %d/%d", i + 1, retry_count_);
            std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms_));
        }

        RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
        camera_opened_ = false;
        return false;
    }

    bool initialize_camera()
    {
        camera_opened_ = false;
        camera_sdk_.reset();
        camera_sdk_ = std::make_unique<RubenSDK>();

        if (!connect_camera())
        {
            return false;
        }
        if (!open_camera())
        {
            return false;
        }

        return true;
    }

    bool restart_camera()
    {
        RCLCPP_WARN(this->get_logger(), "Restarting Ruben camera with full init flow...");
        if (!initialize_camera())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to restart camera.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Camera restarted successfully.");
        return true;
    }

    sensor_msgs::msg::Image mat_to_ros_image(
        const cv::Mat & image,
        const std::string & encoding) const
    {
        sensor_msgs::msg::Image ros_image;
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = frame_id_;

        ros_image = *cv_bridge::CvImage(header, encoding, image).toImageMsg();
        return ros_image;
    }

    sensor_msgs::msg::PointCloud2 pcl_to_ros_point_cloud2(
        const PointCloudXYZPtr & pcl_cloud) const
    {
        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(*pcl_cloud, ros_cloud);
        ros_cloud.header.stamp = this->now();
        ros_cloud.header.frame_id = frame_id_;
        return ros_cloud;
    }

    void handle_service_request(
    const std::shared_ptr<Request> request,
    std::shared_ptr<Response> response)
    {
        const std::string & command = request->input;

        RCLCPP_INFO(this->get_logger(), "Received request: %s", command.c_str());

        if (!camera_sdk_ || !camera_opened_)
        {
            RCLCPP_ERROR(this->get_logger(), "Camera is not ready.");
            return;
        }

        try
        {
            if (command == "capture2D")
            {
                handle_capture_2d(response);
            }
            else if (command == "capture3D")
            {
                handle_capture_3d(response, false);
            }
            else if (command == "capture")
            {
                handle_capture_3d(response, true);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unsupported request command: %s", command.c_str());
            }
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception occurred while handling request: %s", e.what());
        }
    }

    void handle_capture_2d(const std::shared_ptr<Response> response)
    {
        bool captured = false;
        for (int i = 0; i < retry_count_; ++i)
        {
            if (camera_sdk_->Capture2D(true))
            {
                captured = true;
                break;
            }
            RCLCPP_WARN(this->get_logger(), "Capture2D failed, retry %d/%d", i + 1, retry_count_);
            std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms_));
        }
        if (!captured)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture 2D RGB image.");
            return;
        }

        const cv::Mat rgb_image = camera_sdk_->GetImage(CameraNumber::RGB);
        if (rgb_image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "RGB image is empty after Capture2D.");
            return;
        }

        auto ros_rgb_image = mat_to_ros_image(rgb_image, "bgr8");
        response->rgb_image = ros_rgb_image;
        rgb_publisher_->publish(ros_rgb_image);

        RCLCPP_INFO(
            this->get_logger(),
            "2D capture completed successfully: rgb=%dx%d",
            rgb_image.cols,
            rgb_image.rows);
    }

    void handle_capture_3d(
        const std::shared_ptr<Response> response,
        bool with_rgb)
    {
        PointCloudXYZPtr point_cloud;
        cv::Mat rgb_image;
        bool captured = false;
        for (int i = 0; i < retry_count_; ++i)
        {
            if (camera_sdk_->Capture3D())
            {
                point_cloud = camera_sdk_->GetPointCloud();
                if (!point_cloud || point_cloud->empty())
                {
                    RCLCPP_ERROR(this->get_logger(), "Point cloud is empty after Capture3D.");
                }
                else
                {
                    if (with_rgb)
                    {
                        rgb_image = camera_sdk_->GetImage(CameraNumber::RGB);
                        if (rgb_image.empty())
                        {
                            RCLCPP_ERROR(this->get_logger(), "RGB image is empty after Capture3D.");
                        }
                        else
                        {
                            captured = true;
                            break;
                        }
                    }
                    else
                    {
                        captured = true;
                        break;
                    }
                }
            }

            RCLCPP_WARN(
                this->get_logger(),
                "Capture3D failed or returned invalid data, restart retry %d/%d",
                i + 1,
                retry_count_);
            if (!restart_camera())
            {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms_));
        }
        if (!captured)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture 3D data.");
            return;
        }

        auto ros_point_cloud = pcl_to_ros_point_cloud2(point_cloud);
        response->point_cloud = ros_point_cloud;
        point_cloud_publisher_->publish(ros_point_cloud);

        if (with_rgb)
        {
            auto ros_rgb_image = mat_to_ros_image(rgb_image, "bgr8");
            response->rgb_image = ros_rgb_image;
            rgb_publisher_->publish(ros_rgb_image);

            RCLCPP_INFO(
                this->get_logger(),
                "Full capture completed successfully: rgb=%dx%d, points=%zu",
                rgb_image.cols,
                rgb_image.rows,
                point_cloud->size());
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "3D capture completed successfully: points=%zu",
                point_cloud->size());
        }
    }

private:
    std::unique_ptr<RubenSDK> camera_sdk_;

    rclcpp::Service<RubenService>::SharedPtr camera_service_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

    bool camera_opened_ = false;
    int retry_count_ = 3;
    int retry_interval_ms_ = 1000;
    std::string camera_id_ ;
    std::string service_name_;
    std::string rgb_topic_;
    std::string point_cloud_topic_;
    std::string frame_id_;
    std::string camera_paras;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RubenCameraServerNode>("ruben_sdk_server_node");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Ruben SDK server node stopped.");
    rclcpp::shutdown();
    return 0;
}
