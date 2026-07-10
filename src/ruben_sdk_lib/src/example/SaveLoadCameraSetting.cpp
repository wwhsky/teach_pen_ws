// Copyright (c) RVBUST, Inc - All rights reserved.

#include <RVC/RVC.h>
#include <fstream>
#include <iostream>
#include <ruben_sdk_lib/IO/FileIO.h>
#include <ruben_sdk_lib/IO/SavePointMap.h>

static void UseX1() {
    RVC::SystemInit();
    RVC::Device devices[10];
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);

    if (actual_size == 0) {
        std::cout << "Can not find any Camera!" << std::endl;
        return;
    }
    if(devices[0].IsFirmwareMatch() == false){
        std::cout << "device firmware mismatch, Please use RVCManager to upgrade the firmware" << std::endl;
        RVC::SystemShutdown();
        return;
    }
    // Create a RVC Camera and choose use left side camera
    RVC::X1 x1 = RVC::X1::Create(devices[0], RVC::CameraID_Left);

    // Open RVC Camera
    x1.Open();

    if (!x1.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X1." << std::endl;
        RVC::X1::Destroy(x1);
        RVC::SystemShutdown();
        return;
    }
    
    bool ret = x1.SaveSettingToFile("d://test.json");
	if(!ret)
	{
        std::cout << "RVC Camera failed to SaveSettingToFile !" << std::endl;
        RVC::X1::Destroy(x1);
        RVC::SystemShutdown();
        return;
	}

    ret = x1.LoadSettingFromFile("d://test.json");
    if (!ret)
    {
        std::cout << "RVC Camera failed to LoadSettingFromFile !" << std::endl;
        RVC::X1::Destroy(x1);
        RVC::SystemShutdown();
        return;
    }

    RVC::X1::CaptureOptions stored_opt_1;
    x1.LoadCaptureOptionParameters(stored_opt_1);

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    // Capture a point map and a image with default setting.
    if (x1.Capture(stored_opt_1) == true) {
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
    }
    else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "RVC Camera capture failed!" << std::endl;
    }

	
    x1.Close();
    RVC::X1::Destroy(x1);

    RVC::SystemShutdown();
    return;
}

static void UseX2() {
    RVC::SystemInit();
    // Scan all RVC Camera devices
    RVC::Device devices[10];
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);

    // Find whether any Camera is connected or not.
    if (actual_size == 0) {
        std::cout << "Can not find any Camera!" << std::endl;
        return;
    }

    // Create a RVC Camera.
    RVC::X2 x2 = RVC::X2::Create(devices[0]);

    // Open RVC Camera.
    x2.Open();

    // Test RVC Camera is opened or not.
    if (!x2.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X2." << std::endl;
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return;
    }
	
    bool ret = x2.SaveSettingToFile("/home/kdc/code/ruben_sdk_ws/test.json");
    if (!ret)
    {
        std::cout << "RVC Camera failed to SaveSettingToFile !" << std::endl;
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return;
    }

    ret = x2.LoadSettingFromFile("/home/kdc/code/ruben_sdk_ws/test.json");
    if (!ret)
    {
        std::cout << "RVC Camera failed to LoadSettingFromFile !" << std::endl;
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return;
    }

    x2.Close();
    RVC::X2::Destroy(x2);

    RVC::SystemShutdown();
    return;
}

int main(int argc, char *argv[]) {
    UseX1();
    UseX2();
    return 0;
}
