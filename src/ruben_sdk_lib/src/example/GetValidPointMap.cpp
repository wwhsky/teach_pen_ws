
#include <RVC/RVC.h>

#include <cstring>
#include <iostream>
#include <vector>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"

// Extract valid point of PointMap.
std::vector<double> ExtractValidPoint(RVC::PointMap &pm) {
    const unsigned int pm_sz = pm.GetSize().cols * pm.GetSize().rows, len = sizeof(double) * 3;
    std::vector<double> valid_xyzs(pm_sz * 3);
    const double *pm_data = pm.GetPointDataPtr();
    double *valid_data = valid_xyzs.data();
    double valid_num = 0;
    for (int i = 0; i < pm_sz; i++, pm_data += 3) {
        if (pm_data[2] == pm_data[2]) {  // z != nan
            memcpy(valid_data, pm_data, len);
            valid_data += 3;
            valid_num++;
        }
    }
    valid_xyzs.resize(valid_num * 3);
    return valid_xyzs;
}

int main(int argc, char **argv) {
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

    // Capture a point map and a image with default setting.
    if (x1.Capture() == true) {
        std::cout << "RVC Camera capture successed!" << std::endl;

        // Get point map data (m).
        RVC::PointMap pm = x1.GetPointMap();
        std::vector<double> valid_xyzs = ExtractValidPoint(pm);

        // Save PointMap.
        std::string pm_addr = save_directory + "test.ply";
        std::cout << "save point map to file: " << pm_addr << std::endl;
        SavePlyFile(pm_addr.c_str(), valid_xyzs.data(), valid_xyzs.size() / 3);
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