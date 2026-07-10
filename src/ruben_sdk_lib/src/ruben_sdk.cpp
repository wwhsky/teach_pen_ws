#include <iostream>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ruben_sdk_lib/ruben_sdk.h"
#include "ruben_sdk_lib/Utils/Timer.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"

RubenSDK::RubenSDK(): m_is_connected(false), m_is_opened(false)
{
    RVC::SystemInit();
}

RubenSDK::~RubenSDK()
{
    Disconnect();
    RVC::SystemShutdown();
}

bool RubenSDK::Connect(int camera_index)
{
    if(m_is_connected) {
        std::cout << "[Connect] RVC X2 Camera is already connected!" << std::endl;
        return true;
    }

    RVC::Device devices[MAX_DEVICES];
    size_t actual_size = 0;
    SystemListDevices(devices, MAX_DEVICES, &actual_size, RVC::SystemListDeviceType::All);
    if (actual_size == 0) {
        std::cout << "[Connect] Can not find any Camera!" << std::endl;
        return false;
    }

    devices[camera_index].GetDeviceInfo(&m_device_info);
    devices[camera_index].Print();
    if(m_device_info.support_x2 == false)
	{
        std::cout << "[Connect] The camera does not support the x2 function!" << std::endl;
        return false;
    }

    {
        std::unique_lock<std::mutex> lock(m_x2_mutex);
        m_x2_camera = RVC::X2::Create(devices[camera_index]);
    }

    m_is_connected = true;
    return true;
}

bool RubenSDK::Connect(const std::string& target_sn)
{
    if (m_is_connected) {
        std::cout << "[Connect] RVC X2 Camera is already connected!" << std::endl;
        return true;
    }

    RVC::Device devices[MAX_DEVICES];
    size_t actual_size = 0;
    SystemListDevices(devices, MAX_DEVICES, &actual_size, RVC::SystemListDeviceType::All);

    if (actual_size == 0) {
        std::cout << "[Connect] Can not find any Camera!" << std::endl;
        return false;
    }

    // 遍历设备，按 SN 匹配
    RVC::Device* target_device = nullptr;
    for (size_t i = 0; i < actual_size; ++i) {
        RVC::DeviceInfo info;
        devices[i].GetDeviceInfo(&info);

        // 对比序列号 
        std::string current_sn(info.sn);
        if (current_sn == target_sn) {
            target_device = &devices[i];
            m_device_info = info; // 保存设备信息
            break;
        }
    }

    if (!target_device) {
        std::cout << "[Connect] Camera with SN = " << target_sn << " not found!" << std::endl;
        return false;
    }

    // 检查是否支持 X2
    if (m_device_info.support_x2 == false) {
        std::cout << "[Connect] The camera does not support the x2 function!" << std::endl;
        return false;
    }

    // 创建 X2 对象
    {
        std::unique_lock<std::mutex> lock(m_x2_mutex);
        m_x2_camera = RVC::X2::Create(*target_device);
    }

    m_is_connected = true;
    std::cout << "[Connect] Successfully connected to camera SN: " << target_sn << std::endl;
    return true;
}

void RubenSDK::Disconnect()
{
    Close();

    std::unique_lock<std::mutex> lock(m_x2_mutex);
    RVC::X2::Destroy(m_x2_camera);
    m_is_connected = false;
}

bool RubenSDK::Open(const std::string& cam_para)
{
    if (m_is_opened)
    {
        std::cout << "[Open] RVC X2 Camera is already opened!" << std::endl;
        return true;
    }

    std::unique_lock<std::mutex> lock(m_x2_mutex);
    m_x2_camera.Open();

    if (!m_x2_camera.IsOpen()) {
        std::cout << "[Open] Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X2." << std::endl;
        m_is_opened = false;
        return false;
    }

    // load config
    std::string config_path = ament_index_cpp::get_package_share_directory("ruben_sdk_lib") 
                          + "/config/" + cam_para;
    std::cout << "[Connect] Load camera config"  << std::endl;

    if (!m_x2_camera.LoadSettingFromFile(config_path.c_str())) {
        std::cout << "[Connect] Failed to load config from: " << config_path << std::endl;
        return false;
    }

    std::cout << "[Open] RVC X2 Camera opened!" << std::endl;
    m_is_opened = true;
    return true;
}

void RubenSDK::Close()
{
    if (m_is_opened) {
        std::unique_lock<std::mutex> lock(m_x2_mutex);
        m_x2_camera.Close();
        m_is_opened = false;
    }
}

bool RubenSDK::Capture2D(bool is_rgb)
{
    Timer timer("Capture2D");
    if (!m_is_opened) {
        std::cerr << "[Capture2D] Camera not opened!" << std::endl;
        return false;
    }

    std::unique_lock<std::mutex> lock(m_x2_mutex);

    bool capture_status = false;
    if (is_rgb && m_device_info.support_extra) {
        capture_status = m_x2_camera.Capture2D(RVC::CameraID_Extra);  // RGB
    } else {
        capture_status = m_x2_camera.Capture2D(RVC::CameraID_Both);   // Gray
    }
    if(!capture_status)
    {
        std::cout << "[Capture2D] capture 2D failed!" << std::endl;
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        return false;
    }

    if (is_rgb && m_device_info.support_extra) {
        m_rvc_rgb_image = m_x2_camera.GetImage(RVC::CameraID_Extra);
    }
    else {
        m_rvc_left_image = m_x2_camera.GetImage(RVC::CameraID_Left);
        m_rvc_right_image = m_x2_camera.GetImage(RVC::CameraID_Right);
    }

    std::cout << "[Capture2D] capture 2D successed!" << std::endl;

    return true;
}

bool RubenSDK::Capture3D()
{
    Timer timer("Capture3D");
    if (!m_is_opened) {
        std::cerr << "[Capture3D] Camera not opened!" << std::endl;
        return false;
    }

    std::unique_lock<std::mutex> lock(m_x2_mutex);

    // RVC::X2::CaptureOptions cap_opt;
    // m_x2_camera.LoadCaptureOptionParameters(cap_opt);
    // cap_opt.exposure_time_2d = 200;
    bool capture_status = m_x2_camera.Capture();

    if(!capture_status)
    {
        std::cout << "[Capture3D] capture 3D failed!" << std::endl;
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        return false;
    }

    RVC::CameraID cid = RVC::CameraID_Left;
    if (m_device_info.support_extra) {
        cid = RVC::CameraID_Extra;
    }

    m_rvc_rgb_image = m_x2_camera.GetImage(cid);
    m_rvc_point_map = m_x2_camera.GetPointMap();

    std::cout << "rgb image size: " << m_rvc_rgb_image.GetSize().width << " " << m_rvc_rgb_image.GetSize().height << std::endl;
    std::cout << "[Capture3D] capture 3D successed!" << std::endl;

    return true;
}

CameraData RubenSDK::GetCameraData() {
    std::unique_lock<std::mutex> lock(m_capture_data_mutex);
    CameraData camera_data;
    camera_data.point_cloud = ConvertRVCPointMapToPCL(m_rvc_point_map);
    camera_data.rgb_image = ConvertRVCImageToCVMat(m_rvc_rgb_image);
    return camera_data;
}

RubenSDK::PointCloud::Ptr RubenSDK::GetPointCloud()
{
    std::unique_lock<std::mutex> lock(m_capture_data_mutex);
    return ConvertRVCPointMapToPCL(m_rvc_point_map);
}

StereoImages RubenSDK::GetStereoImages()
{
    StereoImages stereo_images;
    stereo_images.left_image = cv::Mat(); // 初始化为空图像
    stereo_images.right_image = cv::Mat();

    std::unique_lock<std::mutex> lock(m_capture_data_mutex);

    if(m_rvc_left_image.GetSize().width == 0 || m_rvc_right_image.GetSize().width == 0)
    {
        std::cout << "No stereo images available!" << std::endl;
        return stereo_images;
    }

    stereo_images.left_image = ConvertRVCImageToCVMat(m_rvc_left_image);
    stereo_images.right_image = ConvertRVCImageToCVMat(m_rvc_right_image);

    return stereo_images;
}

cv::Mat RubenSDK::GetImage(CameraNumber camera_id)
{
    RVC::Image image;
    if (camera_id == CameraNumber::LEFT)
    {
        std::cout << "[GetImage] Get left image" << std::endl;
        image = m_rvc_left_image;
    }
    else if (camera_id == CameraNumber::RIGHT)
    {
        std::cout << "[GetImage] Get right image" << std::endl;
        image = m_rvc_right_image;
    }
    else if (camera_id == CameraNumber::RGB)
    {
        std::cout << "[GetImage] Get RGB image" << std::endl;
        image = m_rvc_rgb_image;
    }
    else
    {
        std::cout << "Invalid CameraNumber selection!" << std::endl;
        return cv::Mat();
    }

    return ConvertRVCImageToCVMat(image);
}

CameraIntrinsicParams RubenSDK::GetCameraIntrinsicParams(CameraNumber camera_id)
{
    CameraIntrinsicParams params;

    if (!m_is_connected) {
        std::cerr << "[GetCameraIntrinsic] Error: Camera not connected!" << std::endl;
        return params;
    }

    RVC::CameraID cid;
    if (camera_id == CameraNumber::LEFT) {
        cid = RVC::CameraID_Left;
    } else if (camera_id == CameraNumber::RIGHT) {
        cid = RVC::CameraID_Right;
    } else if (camera_id == CameraNumber::RGB) {
        cid = RVC::CameraID_Extra;
    } else {
        std::cerr << "[GetCameraIntrinsic] Error: Invalid camera_id selection!" << std::endl;
        return params;
    }

    std::unique_lock<std::mutex> lock(m_x2_mutex);
    m_x2_camera.GetIntrinsicParameters(cid, params.intrinsic, params.distortion);

    return params;
}

bool RubenSDK::GetCameraExtrinsicMatrix(std::array<float, 16>& T_extrinsic, CameraNumber camera_id)
{
    if (!m_is_connected) {
        std::cerr << "[GetCameraExtrinsicMatrix] Error: Camera not connected!" << std::endl;
        return false;
    }

    RVC::CameraID cid;
    std::string eye_name;

    if (camera_id == CameraNumber::LEFT) {
        cid = RVC::CameraID_Left;
        eye_name = "left";
    } else if (camera_id == CameraNumber::RIGHT) {
        cid = RVC::CameraID_Right;
        eye_name = "right";
    } else if (camera_id == CameraNumber::RGB) {
        cid = RVC::CameraID_Extra;
        eye_name = "rgb";
    } else {
        std::cerr << "Error: Invalid camera camera_id selection (" << static_cast<int>(cid) << ")" << std::endl;
        return false;
    }

    std::unique_lock<std::mutex> lock(m_x2_mutex);
    if (!m_x2_camera.GetExtrinsicMatrix(cid, T_extrinsic.data())) {
        std::cerr << "Error: Failed to get " << eye_name << " camera extrinsic matrix" << std::endl;
        return false;
    }

    return true;
}

RubenSDK::PointCloud::Ptr RubenSDK::ConvertRVCPointMapToPCL(const RVC::PointMap& point_map)
{
    auto cloud = std::make_shared<PointCloud>();

    const uint32_t width  = point_map.GetSize().width;
    const uint32_t height = point_map.GetSize().height;

    cloud->width  = width;
    cloud->height = height;
    cloud->is_dense = false;

    // resize points，不要用 cloud->resize()
    cloud->points.resize(static_cast<size_t>(width) * height);

    const double* pm_data = point_map.GetPointDataConstPtr();

    for (size_t i = 0; i < cloud->points.size(); ++i, pm_data += 3) {
        cloud->points[i].x = static_cast<float>(pm_data[0]);
        cloud->points[i].y = static_cast<float>(pm_data[1]);
        cloud->points[i].z = static_cast<float>(pm_data[2]);
    }

    return cloud;
}

cv::Mat RubenSDK::ConvertRVCImageToCVMat(const RVC::Image& img) 
{
	cv::Mat cv_image;
	if (img.GetType() == RVC::ImageType::Mono8) {
		cv::Mat cv_image_temp = cv::Mat(img.GetSize().height, img.GetSize().width, CV_8UC1, (void*)img.GetDataConstPtr());
		cv::cvtColor(cv_image_temp, cv_image, cv::COLOR_GRAY2BGR);
	}
	else if (img.GetType() == RVC::ImageType::RGB8 || img.GetType() == RVC::ImageType::BGR8) {
		cv_image = cv::Mat(img.GetSize().height, img.GetSize().width, CV_8UC3,
			(void*)img.GetDataConstPtr());
	}
	else {
		std::cout << "Unsupported image type!" << std::endl;
        std::cout << img.GetType() << std::endl;
	}
	return cv_image;
}

std::vector<double> RubenSDK::ExtractValidPoint(RVC::PointMap &pm) 
{
    const unsigned int pm_sz = pm.GetSize().cols * pm.GetSize().rows, len = sizeof(double) * 3;
    std::vector<double> valid_xyzs(pm_sz * 3);
    const double *pm_data = pm.GetPointDataPtr();
    double *valid_data = valid_xyzs.data();
    double valid_num = 0;
    for (int i = 0; i < pm_sz; i++, pm_data += 3) {
        if (pm_data[2] == pm_data[2]) {  // z != nan
            memcpy(valid_data, pm_data, len);
            valid_data += 3;
            valid_num++;
        }
    }
    valid_xyzs.resize(valid_num * 3);
    return valid_xyzs;
}

void RubenSDK::SaveStereoImage(const std::string& dir)
{
    CheckDirectory(dir);
    m_rvc_left_image.SaveImage((dir + "/image_left.png").c_str());
    m_rvc_right_image.SaveImage((dir + "/image_right.png").c_str());
}

void RubenSDK::SaveImage(const std::string& dir)
{
    CheckDirectory(dir);
    m_rvc_rgb_image.SaveImage((dir + "/image_rgb.png").c_str());
}

void RubenSDK::SavePointCloud(const std::string& dir)
{
    CheckDirectory(dir);
    m_rvc_point_map.Save((dir + "/pointcloud.ply").c_str(), RVC::PointMapUnit::Millimeter, true);
}

void RubenSDK::SaveTexturePointCloud(const std::string& dir)
{
    CheckDirectory(dir);
    m_rvc_point_map.Save((dir + "/pointcloud_color.ply").c_str(), 
    RVC::PointMapUnit::Millimeter, true, m_rvc_rgb_image);
}

void RubenSDK::CheckDirectory(const std::string& dir) {
    namespace fs = std::filesystem;
    
    fs::path base_path(dir);
    fs::path device_path = base_path / m_device_info.sn / "x2";
    
    if (!fs::exists(device_path)) {
        fs::create_directories(device_path);  // 递归创建
    }
}
