/**
 * ==================================================================================
 *                      Copyright (c) 2025 Friendess Co., Ltd.
 *                          Unpublished - All rights reserved
 * This software is confidential and proprietary product of Friendess Co., Ltd,
 * protected by copyright law and international conventions.
 * Unauthorized reproduction or distribution of this program or any part thereof will
 * be subject to severe legal sanctions, and will also be prosecuted to the maximum
 * extent possible under the law.
 * ==================================================================================
 * @author    张浩东、夏晓武
 * @date      2026-03-25
 * @file      cam_client.cpp
 * @version   v1.0
 * @brief     如本相机 ROS2 客户端节点
 * @details   调用相机服务，获取 RGB 图像与点云数据，并打印返回结果
 * ==================================================================================
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ruben_msgs/srv/ruben_service.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using RubenService = ruben_msgs::srv::RubenService;
using Request = RubenService::Request;
using Response = RubenService::Response;

class RubenCameraClientNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     *
     * 初始化客户端并完成一次服务调用流程
     */
    RubenCameraClientNode()
        : Node("ruben_camera_client_node")
    {
        client_ = this->create_client<RubenService>(kServiceName);

        this->get_logger().set_level(rclcpp::Logger::Level::Info);
        RCLCPP_INFO(this->get_logger(), "Camera client node started.");

        if (!wait_for_service_ready())
        {
            return;
        }

        send_capture_request();
    }

private:
    /**
     * @brief 等待服务上线
     *
     * @return true  服务可用
     * @return false 等待过程中节点被中断
     */
    bool wait_for_service_ready()
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for service [%s] to become available...",
            kServiceName);

        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Interrupted while waiting for service [%s].",
                    kServiceName);
                return false;
            }

            RCLCPP_WARN(
                this->get_logger(),
                "Service [%s] is not available yet, retrying...",
                kServiceName);
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Service [%s] is now available.",
            kServiceName);
        return true;
    }

    /**
     * @brief 发送抓取请求并等待响应
     */
    void send_capture_request()
    {
        auto request = std::make_shared<Request>();
        request->input = "capture";

        RCLCPP_INFO(
            this->get_logger(),
            "Sending capture request to service [%s]...",
            kServiceName);

        auto future_result = client_->async_send_request(request);

        const auto result = rclcpp::spin_until_future_complete(
            this->get_node_base_interface(),
            future_result);

        if (result != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to call service [%s].",
                kServiceName);
            return;
        }

        const auto response = future_result.get();
        log_response(*response);
    }

    /**
     * @brief 打印服务返回结果
     *
     * @param response 服务响应
     */
    void log_response(const Response & response)
    {
        const auto & rgb_image = response.rgb_image;
        const auto & point_cloud = response.point_cloud;

        RCLCPP_INFO(this->get_logger(), "========== Camera Service Response ==========");
        RCLCPP_INFO(
            this->get_logger(),
            "RGB image  : width=%u, height=%u",
            rgb_image.width,
            rgb_image.height);

        RCLCPP_INFO(
            this->get_logger(),
            "Point cloud: width=%u, height=%u, frame_id=%s",
            point_cloud.width,
            point_cloud.height,
            point_cloud.header.frame_id.c_str());

        RCLCPP_INFO(this->get_logger(), "Camera data request completed successfully.");
    }

private:
    static constexpr const char * kServiceName = "/camera_A/get_data";

    rclcpp::Client<RubenService>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RubenCameraClientNode>();

    rclcpp::shutdown();
    return 0;
}