#include <RVC/RVC.h>

#include <iostream>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"

int main(int argc, char *argv[]) {
    // Initialize RVC system.
    RVC::SystemInit();

    // Scan all RVC Camera devices.
    RVC::Device devices[10];
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);

    // Find whether any RVC Camera is connected or not.
    if (actual_size == 0) {
        std::cout << "Can not find any RVC Camera!" << std::endl;
        return -1;
    }
    if(devices[0].IsFirmwareMatch() == false){
        std::cout << "device firmware mismatch, Please use RVCManager to upgrade the firmware" << std::endl;
        RVC::SystemShutdown();
        return -1;
    }
    // Create a RVC Camera and choose use left side camera.
    RVC::X1 x1 = RVC::X1::Create(devices[0], RVC::CameraID_Left);

    // Open RVC Camera.
    x1.Open();

    // Test RVC Camera is opened or not.
    if (!x1.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X1." << std::endl;
        RVC::X1::Destroy(x1);
        RVC::SystemShutdown();
        return 1;
    }

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    RVC::X1::CaptureOptions cap_opt;
    x1.LoadCaptureOptionParameters(cap_opt);
    cap_opt.transform_to_camera = true;

    // Set 2d and 3d exposure time. Capture with white light, range [11, 100]ms, others [3, 100]ms.
    cap_opt.exposure_time_2d = 20;
    cap_opt.exposure_time_3d = 100;

    cap_opt.filter_range = 0;
    cap_opt.phase_filter_range = 0;

    cap_opt.projector_brightness = 240;

    cap_opt.light_contrast_threshold = 3;

    // Set ROI
    RVC::ROI roi = RVC::ROI(10, 10, 200, 200);

    // Get auto capture setting, exposure_time_2d, exposure_time_3d, projector_brightness,
    // light_contrast_threshold will be adjusted automatically

    bool ret = x1.GetAutoCaptureSetting(cap_opt, roi);
    if (ret) {
        std::cout << "get auto capture setting success" << std::endl;
        std::cout << "auto exposure_time_2d:" << cap_opt.exposure_time_2d << std::endl;
        std::cout << "auto exposure_time_3d:" << cap_opt.exposure_time_3d << std::endl;
        std::cout << "auto projector_brightness:" << cap_opt.projector_brightness << std::endl;
        std::cout << "auto light_contrast_threshold:" << cap_opt.light_contrast_threshold << std::endl;
    } else {
        std::cout << "get auto capture setting failed, custom setting will be used" << std::endl;
    }

    // Capture a point map and a image with default setting.
    if (x1.Capture(cap_opt) == true) {
        std::cout << "RVC Camera capture successed!" << std::endl;

        // Get point map data (m).
        RVC::PointMap pm = x1.GetPointMap();
        std::string pm_addr = save_directory + "test.ply";
        std::cout << "save point map to file: " << pm_addr << std::endl;
        pm.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true);
        // Get image data.
        RVC::Image img = x1.GetImage();
        std::string img_addr = save_directory + "test.png";
        std::cout << "save image to file: " << img_addr << std::endl;
        img.SaveImage(img_addr.c_str());
    } else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "RVC Camera capture failed!" << std::endl;
    }

    // Close RVC Camera.
    x1.Close();

    // Destroy RVC Camera.
    RVC::X1::Destroy(x1);

    // Shutdown RVC System.
    RVC::SystemShutdown();

    return 0;
}
