
/**
 * This example is to demonstrate the usage of Cluster Filter and Reflection Filter. Cluster Filter is used in almost
 * all scenarios. It is employed to remove small clusters of outliers. Reflection Filter is designed to eliminate large
 * areas of point cloud noise caused by reflections, which are difficult to remove with Cluster Filter alone.
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

    /**
     * The Cluster Filter removes floating points and isolated clusters from the point cloud.
     * The 'noise_removal_distance' parameter indicates the maximum distance between points that are considered to be
     * in the same class. Points that are farther apart than this value will be classified into a new class.
     * The 'noise_removal_point_number' parameter specifies that if the number of points within the same cluster is less
     * than this value, they will be considered as outliers and removed.
     */

    RVC::X1::CaptureOptions cap_opt;
    x1.LoadCaptureOptionParameters(cap_opt);
    cap_opt.noise_removal_distance = 0.5;  // unit: mm
    cap_opt.noise_removal_point_number = 40;

    /**
     * 'reflection_filter_threshold' used to remove large-area erroneous point clouds caused by reflections.
     * The higher this value is set, the more points will be removed by the reflection filter.
     * Setting it too large may remove data from thin and pointy objects.
     * range:[0, 30]
     */
    cap_opt.reflection_filter_threshold = 10;

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

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