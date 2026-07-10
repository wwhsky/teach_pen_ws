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

    // Set capture parameters
    RVC::X2::CaptureOptions cap_opt;
    x2.LoadCaptureOptionParameters(cap_opt);
    // Set HDR exposure times [0, 2, 3]. 0 presents not use hdr. 2 and 3 presents hdr times.
    // Capture with white light, range [11, 100]ms, others [3, 100]ms.
    int min_exposure_time,max_exposure_time;
    x2.GetExposureTimeRange(&min_exposure_time,&max_exposure_time);
    cap_opt.hdr_exposure_times = 3;
    cap_opt.hdr_exposuretime_content[0] = min_exposure_time;
    cap_opt.hdr_exposuretime_content[1] = (min_exposure_time + max_exposure_time)/2;
    cap_opt.hdr_exposuretime_content[2] = max_exposure_time;

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    // Capture a point map and a image (default can be x2.Capture();)
    if (x2.Capture(cap_opt) == true) {
        std::cout << "RVC Camera capture successed!" << std::endl;
        // Get point map data (m)
        RVC::PointMap pm = x2.GetPointMap();
        std::string pm_addr = save_directory + "test.ply";
        std::cout << "save point map to file: " << pm_addr << std::endl;
        pm.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true);
        // Get image data
        if (!info.support_extra) {
            RVC::Image left_img = x2.GetImage(RVC::CameraID_Left);
            RVC::Image right_img = x2.GetImage(RVC::CameraID_Right);
            std::cout << "save image to directory: " << save_directory << std::endl;
            left_img.SaveImage((save_directory + "left.png").c_str());
            right_img.SaveImage((save_directory + "right.png").c_str());
        } else {
            RVC::Image img = x2.GetImage(RVC::CameraID_Extra);
            std::cout << "save image to directory: " << save_directory << std::endl;
            img.SaveImage((save_directory + "img.png").c_str());
        }
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
