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
 * @author    夏晓武
 * @date      2026-03-25
 * @file      ruben_sdk_test_node.cpp
 * @version   v1.0
 * @brief     如本相机 SDK 全功能测试节点
 * @details   用于测试相机连接、2D/3D 采集、参数读取以及文件保存功能
 *            注意事项:
 *            - 3D 点云可视化依赖 PCL
 *            - 若运行环境中 libusb 版本冲突，需手动设置:
 *              export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
 * ==================================================================================
 */

#include <array>
#include <chrono>
#include <iomanip>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "RVC/RVC.h"
#include "ruben_sdk_lib/ruben_sdk.h"

#include "rclcpp/rclcpp.hpp"

class RubenSDKTestNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     *
     * @param node_name ROS2 节点名称
     */
    explicit RubenSDKTestNode(const std::string & node_name)
        : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "========== Ruben SDK test started ==========");

        camera_sdk_ = std::make_unique<RubenSDK>();

        if (!initialize_camera())
        {
            RCLCPP_ERROR(this->get_logger(), "Camera initialization failed.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Camera initialization completed successfully.");
    }

    /**
     * @brief 析构函数
     *
     * 负责释放 OpenCV 窗口与相机连接
     */
    ~RubenSDKTestNode() override
    {
        cleanup_resources();
    }

    /**
     * @brief 测试 2D 采集与图像获取
     */
    void test_2d_capture()
    {
        RCLCPP_INFO(this->get_logger(), "========== Start 2D capture test ==========");

        test_rgb_capture();
        test_stereo_gray_capture();

        RCLCPP_INFO(this->get_logger(), "========== 2D capture test finished ==========");
    }

    /**
     * @brief 测试 3D 采集与点云获取
     */
    void test_3d_capture()
    {
        RCLCPP_INFO(this->get_logger(), "========== Start 3D capture test ==========");

        if (!check_camera_ready())
        {
            return;
        }

        if (!camera_sdk_->Capture3D())
        {
            RCLCPP_ERROR(this->get_logger(), "3D capture failed.");
            return;
        }

        const cv::Mat rgb_image = camera_sdk_->GetImage(CameraNumber::RGB);
        const auto point_cloud = camera_sdk_->GetPointCloud();

        if (rgb_image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "RGB image is empty after 3D capture.");
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "RGB image acquired successfully: width=%d, height=%d",
                rgb_image.cols,
                rgb_image.rows);
        }

        if (!point_cloud || point_cloud->empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Point cloud is empty after 3D capture.");
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Point cloud acquired successfully: points=%zu",
            point_cloud->size());

        // 如需启用 PCL 可视化，可取消以下注释
        //
        // pcl::visualization::PCLVisualizer::Ptr viewer(
        //     new pcl::visualization::PCLVisualizer("Ruben Cloud Viewer"));
        //
        // viewer->setBackgroundColor(0.05, 0.05, 0.05);
        // viewer->addPointCloud<pcl::PointXYZ>(point_cloud, "sample_cloud");
        //
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "Point cloud viewer opened. Press 'q' to close the preview window.");
        //
        // while (rclcpp::ok() && !viewer->wasStopped())
        // {
        //     viewer->spin();
        // }

        RCLCPP_INFO(this->get_logger(), "========== 3D capture test finished ==========");
    }

    /**
     * @brief 测试相机内外参读取
     */
    void test_camera_parameters()
    {
        RCLCPP_INFO(this->get_logger(), "========== Start camera parameter test ==========");

        if (!check_camera_ready())
        {
            return;
        }

        const std::vector<std::pair<CameraNumber, std::string>> camera_list = {
            {CameraNumber::LEFT, "LEFT"},
            {CameraNumber::RIGHT, "RIGHT"},
            {CameraNumber::RGB, "RGB"}
        };

        for (const auto & camera_item : camera_list)
        {
            const CameraNumber camera_number = camera_item.first;
            const std::string & camera_name = camera_item.second;

            RCLCPP_INFO(this->get_logger(), "----- Camera: %s -----", camera_name.c_str());

            print_intrinsic_parameters(camera_number);
            print_extrinsic_parameters(camera_number, camera_name);
        }

        RCLCPP_INFO(this->get_logger(), "========== Camera parameter test finished ==========");
    }

    /**
     * @brief 测试文件保存功能
     *
     * @param save_path 保存目录
     */
    void test_io_save(const std::string & save_path)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "========== Start file save test: path=%s ==========",
            save_path.c_str());

        if (!check_camera_ready())
        {
            return;
        }

        try
        {
            camera_sdk_->SaveImage(save_path);
            camera_sdk_->SavePointCloud(save_path);
            camera_sdk_->SaveTexturePointCloud(save_path);

            RCLCPP_INFO(this->get_logger(), "All files were saved successfully.");
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "File save failed: %s",
                e.what());
        }

        RCLCPP_INFO(this->get_logger(), "========== File save test finished ==========");
    }

private:
    /**
     * @brief 初始化相机连接与打开流程
     *
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool initialize_camera()
    {
        if (!camera_sdk_)
        {
            RCLCPP_ERROR(this->get_logger(), "Camera SDK instance is null.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Connecting to camera...");
        if (!camera_sdk_->Connect("M3GM620B009"))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to camera.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Opening camera...");
        if (!camera_sdk_->Open("CameraSettingweld.json"))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Camera is ready. SN=%s",
            camera_sdk_->m_device_info.sn);

        return true;
    }

    /**
     * @brief 检查相机是否处于可用状态
     *
     * @return true 相机可用
     * @return false 相机不可用
     */
    bool check_camera_ready() const
    {
        if (!camera_sdk_)
        {
            RCLCPP_ERROR(this->get_logger(), "Camera SDK instance is not initialized.");
            return false;
        }

        if (!camera_sdk_->IsOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Camera is not opened.");
            return false;
        }

        return true;
    }

    /**
     * @brief 测试 RGB 图像采集
     */
    void test_rgb_capture()
    {
        RCLCPP_INFO(this->get_logger(), "[2D] Capturing RGB image...");

        if (!check_camera_ready())
        {
            return;
        }

        if (!camera_sdk_->Capture2D(true))
        {
            RCLCPP_ERROR(this->get_logger(), "[2D] RGB capture failed.");
            return;
        }

        const cv::Mat rgb_image = camera_sdk_->GetImage(CameraNumber::RGB);
        if (rgb_image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "[2D] RGB capture succeeded but image is empty.");
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[2D] RGB image acquired successfully: width=%d, height=%d",
            rgb_image.cols,
            rgb_image.rows);

        show_image("RGB Image", rgb_image, 2000);
    }

    /**
     * @brief 测试双目灰度图像采集
     */
    void test_stereo_gray_capture()
    {
        RCLCPP_INFO(this->get_logger(), "[2D] Capturing stereo gray images...");

        if (!check_camera_ready())
        {
            return;
        }

        if (!camera_sdk_->Capture2D(false))
        {
            RCLCPP_ERROR(this->get_logger(), "[2D] Stereo gray capture failed.");
            return;
        }

        const cv::Mat left_gray_image = camera_sdk_->GetImage(CameraNumber::LEFT);
        const cv::Mat right_gray_image = camera_sdk_->GetImage(CameraNumber::RIGHT);

        if (left_gray_image.empty() || right_gray_image.empty())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[2D] Stereo gray capture succeeded but image data is empty.");
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[2D] Stereo gray images acquired successfully: width=%d, height=%d",
            left_gray_image.cols,
            left_gray_image.rows);

        cv::imshow("Left Gray Image", left_gray_image);
        cv::imshow("Right Gray Image", right_gray_image);
        cv::waitKey(5000);
        cv::destroyAllWindows();
    }

    /**
     * @brief 显示单张图像
     *
     * @param window_name 窗口名称
     * @param image 图像数据
     * @param wait_time_ms 等待时间，单位 ms
     */
    void show_image(
        const std::string & window_name,
        const cv::Mat & image,
        int wait_time_ms) const
    {
        if (image.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Image display skipped because image data is empty: window=%s",
                window_name.c_str());
            return;
        }

        cv::imshow(window_name, image);
        cv::waitKey(wait_time_ms);
        cv::destroyAllWindows();
    }

    /**
     * @brief 打印相机内参
     *
     * @param camera_number 相机编号
     */
    void print_intrinsic_parameters(CameraNumber camera_number)
    {
        const auto params = camera_sdk_->GetCameraIntrinsicParams(camera_number);

        RCLCPP_INFO(this->get_logger(), "Intrinsic matrix K [3x3]:");
        RCLCPP_INFO(
            this->get_logger(),
            "  [%8.3f %8.3f %8.3f]",
            params.intrinsic[0], params.intrinsic[1], params.intrinsic[2]);
        RCLCPP_INFO(
            this->get_logger(),
            "  [%8.3f %8.3f %8.3f]",
            params.intrinsic[3], params.intrinsic[4], params.intrinsic[5]);
        RCLCPP_INFO(
            this->get_logger(),
            "  [%8.3f %8.3f %8.3f]",
            params.intrinsic[6], params.intrinsic[7], params.intrinsic[8]);

        RCLCPP_INFO(
            this->get_logger(),
            "Distortion coefficients D: [k1=%.6f, k2=%.6f, p1=%.6f, p2=%.6f, k3=%.6f]",
            params.distortion[0],
            params.distortion[1],
            params.distortion[2],
            params.distortion[3],
            params.distortion[4]);
    }

    /**
     * @brief 打印相机外参
     *
     * @param camera_number 相机编号
     * @param camera_name 相机名称
     */
    void print_extrinsic_parameters(
        CameraNumber camera_number,
        const std::string & camera_name)
    {
        std::array<float, 16> extrinsic_matrix {};

        if (!camera_sdk_->GetCameraExtrinsicMatrix(extrinsic_matrix, camera_number))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to get extrinsic matrix for camera %s.",
                camera_name.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Extrinsic matrix T [4x4]:");
        RCLCPP_INFO(
            this->get_logger(),
            "  [%8.4f %8.4f %8.4f %8.4f]",
            extrinsic_matrix[0], extrinsic_matrix[1], extrinsic_matrix[2], extrinsic_matrix[3]);
        RCLCPP_INFO(
            this->get_logger(),
            "  [%8.4f %8.4f %8.4f %8.4f]",
            extrinsic_matrix[4], extrinsic_matrix[5], extrinsic_matrix[6], extrinsic_matrix[7]);
        RCLCPP_INFO(
            this->get_logger(),
            "  [%8.4f %8.4f %8.4f %8.4f]",
            extrinsic_matrix[8], extrinsic_matrix[9], extrinsic_matrix[10], extrinsic_matrix[11]);
        RCLCPP_INFO(
            this->get_logger(),
            "  [%8.4f %8.4f %8.4f %8.4f]",
            extrinsic_matrix[12], extrinsic_matrix[13], extrinsic_matrix[14], extrinsic_matrix[15]);
    }

    /**
     * @brief 释放资源
     */
    void cleanup_resources()
    {
        cv::destroyAllWindows();

        if (camera_sdk_ && camera_sdk_->IsOpened())
        {
            camera_sdk_->Disconnect();
        }
    }

private:
    std::unique_ptr<RubenSDK> camera_sdk_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RubenSDKTestNode>("ruben_sdk_test_node");

    // 按需启用以下测试项
    // node->test_2d_capture();
    node->test_3d_capture();
    // node->test_camera_parameters();
    node->test_io_save("/home/fscut/code/zhd/ruben_sdk_ws/data");

    RCLCPP_INFO(node->get_logger(), "========== All tests finished ==========");
    rclcpp::shutdown();
    return 0;
}