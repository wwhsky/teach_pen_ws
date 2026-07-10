#include <RVC/RVC.h>
#include <stdio.h>
#include <iostream>
#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"

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
        std::string pm_addr = save_directory + "test.ply";
        std::cout << "save point map to file: " << pm_addr << std::endl;
        pm.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true);
        // Get image data.
        RVC::Image img = x1.GetImage();
        std::string img_addr = save_directory + "test.png";
        std::cout << "save image to file: " << img_addr << std::endl;
        img.SaveImage(img_addr.c_str());

        const unsigned char *img_data;
        const double *xyz;
        const size_t img_w = img.GetSize().width, img_h = img.GetSize().height;
        bool is_color = img.GetType() == RVC::ImageType::Mono8 ? false : true;
        for (size_t r = 0; r < img_h; r += 10) {
            for (size_t c = 0; c < img_w; c += 10) {
                xyz = pm.GetPointDataPtr() + (r * img_w + c) * 3;
                if (is_color) {
                    img_data = img.GetDataPtr() + (r * img_w + c) * 3;
                    printf("image position(xy): (%zd, %zd) color: (%d, %d, %d), corresponce xyz: (%.6f, %.6f, %.6f)\n", c,
                           r, img_data[0], img_data[1], img_data[2], xyz[0], xyz[1], xyz[2]);
                } else {
                    img_data = img.GetDataPtr() + (r * img_w + c);
                    printf("image position(xy): (%zd, %zd) density: %d, corresponce xyz: (%.6f, %.6f, %.6f)\n", c, r,
                           img_data[0], xyz[0], xyz[1], xyz[2]);
                }
            }
        }
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