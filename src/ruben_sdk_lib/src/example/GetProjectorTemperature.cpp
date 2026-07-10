#include <RVC/RVC.h>

#include <iostream>

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
        return 1;
    }

    if(devices[0].IsFirmwareMatch() == false){
        std::cout << "device firmware mismatch, Please use RVCManager to upgrade the firmware" << std::endl;
        RVC::SystemShutdown();
        return -1;
    }
    // Create a RVC Camera
    RVC::X2 x2 = RVC::X2::Create(devices[0]);
    x2.Open();

    // Test RVC Camera is opened or not.
    if (!x2.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X2." << std::endl;
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return 1;
    }

    float temperature;

    bool ret = x2.GetProjectorTemperature(temperature);

    if (ret) {
        std::cout << "projector temperature:" << temperature << std::endl;
    }

    // Close RVC Camera.
    x2.Close();

    // Destroy RVC Camera.
    RVC::X2::Destroy(x2);

    // Shutdown RVC System.
    RVC::SystemShutdown();

    return 0;
}
