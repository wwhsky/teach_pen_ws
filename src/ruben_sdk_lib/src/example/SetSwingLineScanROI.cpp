#include <RVC/RVC.h>

#include <iostream>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"

int main(int argc, char* argv[]) {
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
    if (devices[0].IsFirmwareMatch() == false) {
        std::cout << "device firmware mismatch, Please use RVCManager to upgrade the firmware" << std::endl;
        RVC::SystemShutdown();
        return -1;
    }
    // Create and open RVC Camera.
    RVC::Device device = devices[0];
    RVC::X2 x2 = RVC::X2::Create(device);
    x2.Open();
    if (!x2.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not "
                     "occupied and supports X2."
                  << std::endl;
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
    // Set capture parameters
    RVC::X2::CaptureOptions cap_opt;
    x2.LoadCaptureOptionParameters(cap_opt);
    // Set capture mode
    cap_opt.capture_mode = RVC::CaptureMode_SwingLineScan;
    // Set the scan time. The longer the scan time, the denser the point cloud will be.
    cap_opt.line_scanner_scan_time_ms = 1000;
    // Set exposure time
    cap_opt.line_scanner_exposure_time_us = 300;
    // Set minimum distance
    cap_opt.line_scanner_min_distance = 400;
    // Set maximum distance
    cap_opt.line_scanner_max_distance = 1500;
    // Set whether to enable point clouds to correspond to 2D images.
    // If set to true, a depth map will be generated, otherwise no depth map will be generated.
    cap_opt.correspond2d = false;

    // Set ROI's width, height, offset x and y
    cap_opt.roi.width = 512;
    cap_opt.roi.height = 512;
    cap_opt.roi.x = 256;
    cap_opt.roi.y = 256;

    // Check ROI
    if (x2.CheckRoi(cap_opt.roi) == false) {
        std::cout << "roi is not valid,need to adjust. " << std::endl;

        // Auto Adjust Roi
        cap_opt.roi = x2.AutoAdjustRoi(cap_opt.roi);
    }

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    // Capture a point map and a image (default can be x1.Capture();)
    if (x2.Capture(cap_opt) == true) {
        std::cout << "RVC Camera capture successed!" << std::endl;
        // Get point map data (m).
        RVC::PointMap pm = x2.GetPointMap();
        std::string pm_addr = save_directory + "test.ply";
        std::cout << "save point map to file: " << pm_addr << std::endl;
        pm.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true);
        // Get left image data
        RVC::Image img = x2.GetImage(camera_id);
        std::string img_addr = save_directory + "test.png";
        std::cout << "save image to file: " << img_addr << std::endl;
        img.SaveImage(img_addr.c_str());
    } else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "RVC Camera capture failed!" << std::endl;
    }

    // Close RVC Camera
    x2.Close();

    // Destroy RVC Camera
    RVC::X2::Destroy(x2);

    // Shut Down RVC System
    RVC::SystemShutdown();
    return 0;
}
