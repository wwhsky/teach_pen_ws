#include <RVC/RVC.h>
#include <iostream>
#include "ruben_sdk_lib/IO/FileIO.h"

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
    RVC::CameraID camera_id = RVC::CameraID_Left;
    if (info.support_extra) {
        camera_id = RVC::CameraID_Extra;
    }

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    RVC::X2::CaptureOptions cap_opt;
    cap_opt.exposure_time_2d = 50;
    // X2 Capture 2D with left camera
    if (x2.Capture2D(camera_id, cap_opt)) {
        std::cout << "Capture 2D success!" << std::endl;
        RVC::Image img = x2.GetImage(camera_id);
        std::string img_addr = save_directory + "test_2d.png";
        std::cout << "Save image to file: " << img_addr << std::endl;
        img.SaveImage(img_addr.c_str());
    } else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "Capture 2D failed!" << std::endl;
    }

    // X2 Capture 3D
    if (x2.Capture()) {
        std::cout << "Capture 3D successed!" << std::endl;
    } else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "Capture 3D failed!" << std::endl;
    }

    // Close RVC Camera
    x2.Close();

    // Destroy RVC Camera
    RVC::X2::Destroy(x2);

    // Shut Down RVC System
    RVC::SystemShutdown();
    return 0;
}