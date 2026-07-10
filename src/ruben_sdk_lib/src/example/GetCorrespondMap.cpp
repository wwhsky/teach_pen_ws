#include <RVC/RVC.h>

#include <iostream>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"
/**
 * Note:CorrespondMap is only effective when the mode is SwingLineScan mode and correspond2d is set to false.
 * In other cases, the 3D points already correspond to the 2D image.
 */
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
    cap_opt.line_scanner_scan_time_ms = 2500;
    // Set exposure time
    cap_opt.line_scanner_exposure_time_us = 300;
    // Set minimum distance
    cap_opt.line_scanner_min_distance = 400;
    // Set maximum distance
    cap_opt.line_scanner_max_distance = 800;
    // Set whether to enable point clouds to correspond to 2D images.
    // If set to true, a depth map will be generated, otherwise no depth map will be generated.
    cap_opt.correspond2d = false;

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    // Capture a point map and a image with default setting.
    if (x2.Capture(cap_opt) == true) {
        // Get point map data (m).
        RVC::PointMap pm = x2.GetPointMap();
        std::string pm_addr = save_directory + "test.ply";
        std::cout << "save point map to file: " << pm_addr << std::endl;
        pm.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true);

        // Get image data. choose left or right side. the point map is map to left image.
        RVC::Image img = x2.GetImage(camera_id);
        std::string img_addr = save_directory + "test.png";
        std::cout << "save image to file: " << img_addr << std::endl;
        img.SaveImage(img_addr.c_str());

        if (cap_opt.capture_mode == RVC::CaptureMode_SwingLineScan && cap_opt.correspond2d == false) {
            RVC::CorrespondMap correspond_map = x2.GetCorrespondMap();

            const RVC::Size sz = correspond_map.GetSize();
            int select_point3d_index = (sz.height * sz.width) / 2;
            double correspond2d_x = *(correspond_map.GetDataConstPtr() + 2 * select_point3d_index + 0);
            double correspond2d_y = *(correspond_map.GetDataConstPtr() + 2 * select_point3d_index + 1);
            printf("The %dth 3D point corresponds to the 2D pixel: (%f, %f)\n", select_point3d_index, correspond2d_x,
                   correspond2d_y);
        } else {
            std::cout << "CorrespondMap is only effective when the mode is SwingLineScan mode and correspond2d is set "
                         "to false.In other cases, the 3D points already correspond to the 2D image."
                      << std::endl;
        }
    } else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "RVC Camera capture failed!" << std::endl;
    }

    // Close RVC Camera.
    x2.Close();

    // Destroy RVC Camera.
    RVC::X2::Destroy(x2);

    // Shutdown RVC System.
    RVC::SystemShutdown();

    return 0;
}