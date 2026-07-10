#include <RVC/RVC.h>
#include <iostream>
#include "ruben_sdk_lib/IO/FileIO.h"

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

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);
    
    RVC::X1::CaptureOptions cap_opt;
    cap_opt.exposure_time_2d = 50;
    // X1 Capture 2D
    if (x1.Capture2D(cap_opt)) {
        std::cout << "Capture 2D success!" << std::endl;
        RVC::Image img = x1.GetImage();
        std::string img_addr = save_directory + "test_2d.png";
        std::cout << "Save image to file: " << img_addr << std::endl;
        img.SaveImage(img_addr.c_str());
    } else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "Capture 2D failed!" << std::endl;
    }

    // X1 Capture 3D
    if (x1.Capture()) {
        std::cout << "Capture 3D successed!" << std::endl;
    } else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "Capture 3D failed!" << std::endl;
    }

    // Close RVC Camera
    x1.Close();

    // Destroy RVC Camera
    RVC::X1::Destroy(x1);

    // Shut Down RVC System
    RVC::SystemShutdown();
    return 0;
}