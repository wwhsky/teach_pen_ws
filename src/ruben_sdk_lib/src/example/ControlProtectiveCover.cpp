#include <RVC/RVC.h>

#include <iostream>
#include <thread>
#include <chrono>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"

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
    if(info.support_protective_cover == false){
        std::cout << "The device does not support protective cover" << std::endl;
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return -1;
    }
    
    RVC::ProtectiveCoverStatus status = RVC::ProtectiveCoverStatus_Unknown;
    x2.GetProtectiveCoverStatus(status);
    if(status != RVC::ProtectiveCoverStatus_Open)
        x2.OpenProtectiveCover();

    RVC::X2::CaptureOptions opts;
    x2.LoadCaptureOptionParameters(opts);

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);
    // Capture a point map and a image with default setting.
    if (x2.Capture(opts) == true) {
        std::cout << "RVC Camera capture success!" << std::endl;
    } else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "RVC Camera capture failed!" << std::endl;
    }

    x2.CloseProtectiveCover();

    // Close RVC Camera.
    x2.Close();

    // Destroy RVC Camera.
    RVC::X2::Destroy(x2);

    // Shutdown RVC System.
    RVC::SystemShutdown();

    return 0;
}
