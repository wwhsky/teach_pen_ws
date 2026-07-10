#include <RVC/RVC.h>

#include <iostream>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"

int main(int argc, char *argv[]) {
    // Initialize RVC system.
    RVC::SystemInit();

	// Find Device
    RVC::Device device;

	// Method 1,Find device by index.
    {

        RVC::Device devices[10];
        size_t actual_size = 0;
        SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);
        if (actual_size == 0) {
            std::cout << "Can not find any Camera!" << std::endl;
            return -1;
        }
        device = devices[0];
    }

	// Method 2,Find device by sn
	// This is the most recommended method when you have multiple cameras.
    {
        //device = RVC::SystemFindDevice("12345678");
    }
    
    if(device.IsFirmwareMatch() == false){
        std::cout << "device firmware mismatch, Please use RVCManager to upgrade the firmware" << std::endl;
        RVC::SystemShutdown();
        return -1;
    }
    
	// Create and Open
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
        std::cout << "Device support extra camera" << std::endl;
        camera_id = RVC::CameraID_Extra;
    }

	// Capture
    // Capture using the internal parameters of the camera.
    // We suggest that you adjust the parameters in RVC Manager first.
    bool ret = x2.Capture();
    if(ret == false)
    {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        x2.Close();
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return -1;
    }

	// Data Process
    std::string dir = "./data/";
    MakeDirectories(dir);
    dir += info.sn;
    MakeDirectories(dir);
    dir += "/x2";
    MakeDirectories(dir);

    RVC::PointMap pointcloud = x2.GetPointMap();
    RVC::Image image = x2.GetImage(camera_id);
    pointcloud.Save((dir + "/pointcloud.ply").c_str(), RVC::PointMapUnit::Millimeter, true);
    pointcloud.Save((dir + "/pointcloud_color.ply").c_str(),RVC::PointMapUnit::Millimeter,true,image);
    image.SaveImage((dir + "/image.png").c_str());


    // Close RVC Camera.
    x2.Close();
    RVC::X2::Destroy(x2);
    RVC::SystemShutdown();

    return 0;
}
