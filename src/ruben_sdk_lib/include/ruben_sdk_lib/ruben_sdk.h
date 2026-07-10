/**
* ==================================================================================
*                      Copyright (c) 2024 Friendess Co., Ltd.
*                          Unpublished - All rights reserved
* This software is confidential and proprietary product of Friendess Co., Ltd,
* protected by copyright law and international conventions.
* Unauthorized reproduction or distribution of this program or any part thereof will
* be subject to severe legal sanctions, and will also be prosecuted to the maximum
* extent possible under the law.
* ==================================================================================
* @author    :xia xiaowu
* @date      :2026-2-2
* ==================================================================================
* @file      :ruben_sdk.h ruben_sdk.cpp
* @version   :v1.0
* ==================================================================================
*/
#pragma once

#include <vector>
#include <string>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <RVC/RVC.h>

constexpr size_t MAX_DEVICES = 10;

/**
 * @brief 双目图像结构体
 * @details
 * 该结构体用于保存双目相机采集的图像数据，包含左目图像和右目图像，灰度图。
 */
struct StereoImages
{
    cv::Mat left_image;
    cv::Mat right_image;
};


/**
 * @brief 相机数据结构体
 * @details
 * 该结构体用于保存一次相机采集得到的完整数据，
 * 包含三维点云数据以及对应的双目图像数据。
 */
struct CameraData {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cv::Mat rgb_image;
    CameraData() = default; 
};


/**
 * @brief 相机内参结构体
 * @details
 * 该结构体用于保存相机的内参矩阵和畸变参数，
 * 通常用于图像校正、三维重建等计算过程。
 */
struct CameraIntrinsicParams
{
    float intrinsic[9]{};  // 相机内参矩阵 (fx, 0, cx, 0, fy, cy, 0, 0, 1)
    float distortion[5]{}; // 相机畸变参数 k1, k2, p1, p2, k3
};

/**
 * @brief 相机视角枚举
 * @details
 * 用于指定当前操作或获取图像时所使用的相机视角。
 */
enum class CameraNumber
{
    LEFT = 0,
    RIGHT = 1,
    RGB = 2
};


/**
 * @brief Ruben 相机 SDK
 * @details
 * 该类封装了 Ruben M2600三合一相机的底层操作接口，仅支持X2型号的相机
 * 提供相机连接、数据采集、图像与点云获取与保存等功能
 */
class RubenSDK
{
public:
    using PointT = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointT>;

    /**
     * @brief 构造函数
     */
    RubenSDK();

    /**
     * @brief 析构函数
     */
    virtual ~RubenSDK();

    /**
     * @brief 禁止拷贝构造
     */
    RubenSDK(const RubenSDK&) = delete;

    /**
     * @brief 禁止拷贝赋值
     */
    RubenSDK& operator=(const RubenSDK&) = delete;

    /**
     * @brief 连接相机设备
     * @param camera_index 相机ID
     * @return 连接成功返回 true，否则返回 false
     */
    bool Connect(int camera_index);

    /**
     * @brief 连接相机设备
     * @param target_sn 相机sn
     * @return 连接成功返回 true，否则返回 false
     */
    bool Connect(const std::string& target_sn);

    /**
     * @brief 断开相机连接
     */
    void Disconnect(); 

    /**
     * @brief 打开相机
     * @return 打开成功返回 true，否则返回 false
     */
    bool Open(const std::string& cam_para);

    /**
     * @brief 关闭相机
     */
    void Close();

    /**
     * @brief 采集图像
     * @param is_rgb 是否采集 RGB 图像，默认为 true，否则采集双目灰度图像
     * @return 采集成功返回 true，否则返回 false
     */
    bool Capture2D(bool is_rgb = true);

    /**
     * @brief 采集三维点云数据和双目图像
     * @return 采集成功返回 true，否则返回 false
     */
    bool Capture3D();

    /**
     * @brief 获取当前相机数据
     * @return 相机数据结构体
     */
    CameraData GetCameraData();

    /**
     * @brief 获取 XYZ 点云数据
     * @return PCL 点云指针
     */
    PointCloud::Ptr GetPointCloud();

    /**
     * @brief 获取双目图像
     * @return 双目图像结构体
     */
    StereoImages GetStereoImages();

    /**
     * @brief 获取单目图像
     * @param camera_id 指定相机视角（默认RGB）
     * @return OpenCV 图像
     */
    cv::Mat GetImage(CameraNumber camera_id = CameraNumber::RGB);

    /**
     * @brief 获取相机内参
     * @param camera_id 指定相机视角（默认RGB）
     * @return 相机内参结构体
     */
    CameraIntrinsicParams GetCameraIntrinsicParams(CameraNumber camera_id = CameraNumber::RGB);

    /**
     * @brief 获取相机外参矩阵
     * @param T_extrinsic 输出 4x4 外参矩阵（列主序 / 行主序由实现决定）
     * @param camera_id 指定相机视角（默认RGB）
     * @return 获取成功返回 true，否则返回 false
     */
    bool GetCameraExtrinsicMatrix(
        std::array<float, 16>& T_extrinsic,
        CameraNumber camera_id = CameraNumber::RGB);

    /**
     * @brief 保存双目灰度图
     * @param dir 图像保存目录路径
     */
    void SaveStereoImage(const std::string& dir);

    /**
     * @brief 保存RGB图像
     * @param dir 图像保存目录路径
     */
    void SaveImage(const std::string& dir);

    /**
     * @brief 保存原始点云
     * @param dir 点云保存目录路径
     */
    void SavePointCloud(const std::string& dir);

    /**
     * @brief 保存纹理点云
     * @param dir 点云保存目录路径
     */
    void SaveTexturePointCloud(const std::string& dir);

    /**
     * @brief 判断相机是否已连接
     * @return 已连接返回 true，否则返回 false
     */
    inline bool IsConnected() const { return m_is_connected; }

    /**
     * @brief 判断相机是否已打开
     * @return 已打开返回 true，否则返回 false
     */
    inline bool IsOpened() const { return m_is_opened; }

private:

    /**
     * @brief 相机采集的内部数据结构
     */
    struct RubenCaptureData
    {
        RVC::PointMap point_map;   // 点云数据
        RVC::Image left_image;     // 左目图像
        RVC::Image right_image;    // 右目图像
    };

    /**
     * @brief 将 RVC 点云转换为 PCL 点云
     * @param point_map RVC 点云数据
     * @return PCL 点云指针
     */
    PointCloud::Ptr ConvertRVCPointMapToPCL(const RVC::PointMap& point_map);

    /**
     * @brief 将 RVC 图像转换为 OpenCV 图像
     * @param img RVC 图像
     * @return OpenCV 图像
     */
    cv::Mat ConvertRVCImageToCVMat(const RVC::Image& img);

    /**
     * @brief 提取有效点云数据
     * @param pm RVC 点云数据
     * @return 有效点集合
     */
    std::vector<double> ExtractValidPoint(RVC::PointMap& pm);

    /**
     * @brief 检查并创建目录
     * @param dir 目录路径
     */
    void CheckDirectory(const std::string& dir);

private:
    bool m_is_connected = false;                          // 相机连接状态
    bool m_is_opened = false;                             // 相机打开状态

    RVC::X2 m_x2_camera;                                  // 相机对象
    RVC::PointMap m_rvc_point_map;                        // 点云数据
    RVC::Image m_rvc_rgb_image;                           // RGB 图像
    RVC::Image m_rvc_left_image;                          // 左目图像
    RVC::Image m_rvc_right_image;                         // 右目图像
    std::mutex m_capture_data_mutex;                      // 数据队列互斥锁
    std::mutex m_x2_mutex;                                // 相机硬件操作互斥锁

public:
    RVC::DeviceInfo m_device_info;                        // 相机设备信息
};
