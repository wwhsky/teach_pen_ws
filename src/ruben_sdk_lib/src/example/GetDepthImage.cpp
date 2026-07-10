/**
 * This program demonstrates the process of acquiring a depth map.
 *
 * Important: When the capture mode is set to `CaptureMode_SwingLineScan` and `correspond2d` is set to false, acquiring
 * a depth map is not supported. To obtain a depth map in swinglinescan mode, you must enable `correspond2d` by setting
 * it to true. Be advised that enabling `correspond2d` will modify the original point cloud data, resulting in minor
 * discrepancies between the point clouds generated with and without `correspond2d` enabled. This modification may
 * reduce the accuracy of the point cloud data.
 */

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

        // Get and save depth image
        RVC::DepthMap dp = x1.GetDepthMap();
        std::string depth_addr = save_directory + "depth.tiff";
        std::cout << "save depth image to file: " << depth_addr << std::endl;
        dp.SaveDepthMap(depth_addr.c_str());
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