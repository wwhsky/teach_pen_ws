#include <RVC/RVC.h>

#include <iostream>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"

int main(int argc, char *argv[]) {
    // Initialize RVC system.
    RVC::SystemInit();

    // Find Device
    RVC::Device devices[10];
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);
    if (actual_size == 0) {
        std::cout << "Can not find any Camera!" << std::endl;
        return -1;
    }
    if(devices[0].IsFirmwareMatch() == false){
        std::cout << "device firmware mismatch, Please use RVCManager to upgrade the firmware" << std::endl;
        RVC::SystemShutdown();
        return -1;
    }
    
    RVC::Device device = devices[0];

    RVC::DeviceInfo info;
    device.GetDeviceInfo(&info);

    // Create and Open
    RVC::X1 x1 = RVC::X1::Create(device);
    x1.Open();
    if (!x1.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X1." << std::endl;
        RVC::X1::Destroy(x1);
        RVC::SystemShutdown();
        return -1;
    }

    // Capture Options
    RVC::X1::CaptureOptions options;
    x1.LoadCaptureOptionParameters(options);
    bool ret = x1.Capture(options);

    if (ret == false) {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        x1.Close();
        RVC::X1::Destroy(x1);
        RVC::SystemShutdown();
        return -1;
    }

    // Data Process
    std::string dir = "./Data/";
    MakeDirectories(dir);
    dir += info.sn;
    MakeDirectories(dir);
    dir += "/x1";
    MakeDirectories(dir);

    auto pointcloud_helper = RVC::utils::PointMap(x1.GetPointMap());
    auto image_helper = RVC::utils::Image(x1.GetImage());

    pointcloud_helper.pm.Save((dir + "/pointcloud.ply").c_str(), RVC::PointMapUnit::Millimeter, true);
    pointcloud_helper.pm.Save((dir + "/pointcloud_color.ply").c_str(), RVC::PointMapUnit::Millimeter, true,
                              image_helper.img);
    image_helper.img.SaveImage((dir + "/image.png").c_str());

    // convert uint manually
    for (int i = 0; i < pointcloud_helper.GetSize().rows; ++i) {
        for (int j = 0; j < pointcloud_helper.GetSize().cols; ++j) {
            auto sp = pointcloud_helper.At(i, j);
            pointcloud_helper.Set(i, j, { sp.x * 1000, sp.y * 1000, sp.z * 1000 });
        }
    }

    // Close RVC Camera.
    x1.Close();
    RVC::X1::Destroy(x1);
    RVC::SystemShutdown();

    return 0;
}
