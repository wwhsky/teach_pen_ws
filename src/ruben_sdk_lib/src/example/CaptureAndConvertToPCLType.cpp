#include <RVC/RVC.h>

#include <iostream>

#include "ruben_sdk_lib/IO/SavePointMap.h"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

static pcl::PointCloud<pcl::PointXYZ>::Ptr PointMap2CloudPoint(RVC::PointMap &pm) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->height = pm.GetSize().height;
    cloud->width = pm.GetSize().width;
    cloud->is_dense = false;
    cloud->resize(cloud->height * cloud->width);
    const unsigned int pm_sz = cloud->height * cloud->width;
    const double *pm_data = pm.GetPointDataConstPtr();
    for (int i = 0; i < pm_sz; i++, pm_data += 3) {
        cloud->points[i].x = pm_data[0];
        cloud->points[i].y = pm_data[1];
        cloud->points[i].z = pm_data[2];
    }
    return cloud;
}

int main(int argc, char **argv) {
    // Initialize RVC system
    RVC::SystemInit();

    // Scan all RVC Camera devices
    RVC::Device devices[10];
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);

    // Find whether any RVC Camera is connected or not
    if (actual_size == 0) {
        std::cout << "Can not find any RVC Camera!" << std::endl;
        RVC::SystemShutdown();
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
        return -1;
    }

    // Capture a point map and a image (default can be x1.Capture();)
    if (x1.Capture() == true) {
        std::cout << "RVC Camera capture successed!" << std::endl;
        // Get point map data (m)
        RVC::PointMap pm = x1.GetPointMap();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = PointMap2CloudPoint(pm);

        const double *pm_data = pm.GetPointDataConstPtr();

        const size_t n_pts = pm.GetSize().width * pm.GetSize().height;
        const size_t step = n_pts / 10000;  // print 10000 point

        for (size_t i = 0; i < n_pts; i += step) {
            printf("index: %d RVC::PointMap: (%.6f, %.6f, %.6f), pcd::PointCloud: (%.6f, %.6f, %.6f)\n", i,
                   pm_data[i * 3], pm_data[i * 3 + 1], pm_data[i * 3 + 2], cloud->points[i].x, cloud->points[i].y,
                   cloud->points[i].z);
        }

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