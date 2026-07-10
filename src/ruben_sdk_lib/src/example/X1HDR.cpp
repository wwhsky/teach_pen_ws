#include <RVC/RVC.h>

#include <iostream>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"

int main(int argc, char* argv[]) {
    // Initialize RVC system
    RVC::SystemInit();

    // Scan all RVC Camera devices
    RVC::Device devices[10];
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);

    // Find whether any RVC Camera is connected or not
    if (actual_size == 0) {
        std::cout << "Can not find any RVC Camera!" << std::endl;
        return -1;
    }
    if(devices[0].IsFirmwareMatch() == false){
        std::cout << "device firmware mismatch, Please use RVCManager to upgrade the firmware" << std::endl;
        RVC::SystemShutdown();
        return -1;
    }
    // Create a RVC Camera and choose use left side camera
    RVC::X1 x1 = RVC::X1::Create(devices[0], RVC::CameraID_Left);

    // Open RVC Camera
    x1.Open();

    // Test RVC Camera is opened or not
    if (!x1.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X1." << std::endl;
        RVC::X1::Destroy(x1);
        RVC::SystemShutdown();
        return 1;
    }
    // Set capture parameters
    RVC::X1::CaptureOptions cap_opt;
    x1.LoadCaptureOptionParameters(cap_opt);
    // Set HDR exposure times [0, 2, 3]. 0 presents not use hdr. 2 and 3 presents hdr times.
    // Capture with white light, range [11, 100]ms, others [3, 100]ms.
    int min_exposure_time,max_exposure_time;
    x1.GetExposureTimeRange(&min_exposure_time,&max_exposure_time);
    cap_opt.hdr_exposure_times = 3;
    cap_opt.hdr_exposuretime_content[0] = min_exposure_time;
    cap_opt.hdr_exposuretime_content[1] = (min_exposure_time + max_exposure_time)/2;
    cap_opt.hdr_exposuretime_content[2] = max_exposure_time;

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    // Capture a point map and a image (default can be x1.Capture();)
    if (x1.Capture(cap_opt) == true) {
        std::cout << "RVC Camera capture successed!" << std::endl;
        // Get point map data (m).
        RVC::PointMap pm = x1.GetPointMap();
        std::string pm_addr = save_directory + "test.ply";
        std::cout << "save point map to file: " << pm_addr << std::endl;
        pm.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true);
        // Get image data
        RVC::Image img = x1.GetImage();
        std::string img_addr = save_directory + "test.png";
        std::cout << "save image to file: " << img_addr << std::endl;
        img.SaveImage(img_addr.c_str());
    } else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "RVC Camera capture failed!" << std::endl;
    }

    // Close RVC Camera
    x1.Close();

    // Destroy RVC Camera
    RVC::X1::Destroy(x1);

    // Shut Down RVC System
    RVC::SystemShutdown();
    return 0;
}
