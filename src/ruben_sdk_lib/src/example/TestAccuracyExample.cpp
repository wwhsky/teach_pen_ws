#include <RVC/RVC.h>
#include <RVC/experimental/MarkerDetection.h>

#include <iostream>
#include <vector>

#include "ruben_sdk_lib/IO/FileIO.h"
int main(int argc, char *argv[]) {
    // Initialize RVC system.
    RVC::SystemInit();

    // Choose RVC Camera type (USB, GigE or All)
    RVC::Device devices[10];
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);

    // Find whether any Camera is connected or not.
    if (actual_size == 0) {
        std::cout << "Can not find any Camera!" << std::endl;
        RVC::SystemShutdown();
        return -1;
    }
    if(devices[0].IsFirmwareMatch() == false){
        std::cout << "device firmware mismatch, Please use RVCManager to upgrade the firmware" << std::endl;
        RVC::SystemShutdown();
        return -1;
    }
    // Create and open RVC Camera.
    RVC::Device device = devices[0];
    RVC::X2 x2 = RVC::X2::Create(device);
    x2.Open();
    if (!x2.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X2." << std::endl;
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return -1;
    }

    RVC::DeviceInfo info;
    device.GetDeviceInfo(&info);
    RVC::CameraID camera_id = RVC::CameraID_Left;
    if (info.support_extra) {
        camera_id = RVC::CameraID_Extra;
    }

    // Capture
    // Capture using the internal parameters of the camera.
    // We suggest that you adjust the parameters in RVC Manager first.
    bool ret = x2.Capture();
    if (ret == false) {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        x2.Close();
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return -1;
    }

    // Data Process
    std::string dir = "./Data/";
    MakeDirectories(dir);
    dir += info.sn;
    MakeDirectories(dir);
    dir += "/x2";
    MakeDirectories(dir);

    RVC::PointMap pointcloud = x2.GetPointMap();
    RVC::Image image = x2.GetImage(camera_id);
    float camera_intrinsic[9]{};
    float camera_distortion[5]{};
    x2.GetIntrinsicParameters(camera_id, camera_intrinsic, camera_distortion);
    int caliboard_pattern_size_width = 4;
    int caliboard_pattern_size_height = 11;
    int circle_num = caliboard_pattern_size_width * caliboard_pattern_size_height;
    // Reference value(m): A1:0.112 A2:0.08 A3:0.056 A4:*0.04 A5:0.028 A6:0.02 A7:0.014 A8:0.01 A9:0.007 A10:0.0048
    float caliboard_circle_center_standard_3d_distance_step = 0.04f;
    std::vector<float> circle_center_2d((size_t)circle_num * 2);
    std::vector<float> circle_center_3d((size_t)circle_num * 3);
    float measuring_distance;
    float error_percentage;
    int error_code = RVC::TestAccuracy(image, pointcloud, camera_intrinsic, camera_distortion,
                                       caliboard_pattern_size_width, caliboard_pattern_size_height,
                                       caliboard_circle_center_standard_3d_distance_step, circle_center_2d.data(),
                                       circle_center_3d.data(), measuring_distance, error_percentage);
    if (error_code != 0) {
        std::cout << "TestAccuracy failed with error code: " << error_code << std::endl;
        x2.Close();
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return -1;
    }

    std::cout << "measuring_distance: " << measuring_distance << "m" << std::endl;
    std::cout << "error_percentage: " << error_percentage << "%" << std::endl;
    pointcloud.Save((dir + "/pointcloud_color.ply").c_str(), RVC::PointMapUnit::Millimeter, true, image);
    image.SaveImage((dir + "/image.png").c_str());

    // Close RVC Camera.
    x2.Close();
    RVC::X2::Destroy(x2);
    RVC::SystemShutdown();

    return 0;
}
