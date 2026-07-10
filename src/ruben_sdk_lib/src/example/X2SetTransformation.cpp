#include <RVC/RVC.h>

#include <iostream>

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
    RVC::CameraID camera_id = RVC::CameraID_Left;
    if (info.support_extra) {
        camera_id = RVC::CameraID_Extra;
    }

    // Set capture parameters
    RVC::X2::CaptureOptions cap_opt;
    x2.LoadCaptureOptionParameters(cap_opt);
    cap_opt.exposure_time_2d = 20;
    cap_opt.exposure_time_3d = 20;

    double transformation[16] = {0.9981883352,
                                 0.0502378577,
                                 0.0331089875,
                                 -0.0472612266,
                                 0.0505828392,
                                 -0.9986731004,
                                 -0.0096651386,
                                 0.076352919,
                                 0.0325794993,
                                 0.0113223752,
                                 -0.999405013,
                                 0.9951857984,
                                 0.,
                                 0.,
                                 0.,
                                 1.

    };

    RVC::X2::CustomTransformOptions custom_trans_opt;
    if (!info.support_extra) {
        custom_trans_opt.coordinate_select = RVC::X2::CustomTransformOptions::CoordinateSelect_CameraLeft;
    } else {
        custom_trans_opt.coordinate_select = RVC::X2::CustomTransformOptions::CoordinateSelect_CameraExtra;
    }
    for (size_t i = 0; i < 16; i++) {
        custom_trans_opt.transform[i] = transformation[i];
    }
    bool ret = x2.SetCustomTransformation(custom_trans_opt);

    if (ret) {
        std::cout << "set transformation success" << std::endl;
    } else {
        std::cout << "set transformation failed! custom transformation will not be used!" << std::endl;
    }

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    // Capture a point map and a image with custom setting.
    if (x2.Capture(cap_opt) == true) {
        std::cout << "RVC Camera capture successed!" << std::endl;

        // Get point map data (m).
        RVC::PointMap pm = x2.GetPointMap();
        std::string pm_addr = save_directory + "test.ply";
        std::cout << "save point map to file: " << pm_addr << std::endl;
        pm.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true);
        // Get image data.
        RVC::Image img = x2.GetImage(camera_id);
        std::string img_addr = save_directory + "test.png";
        std::cout << "save image to file: " << img_addr << std::endl;
        img.SaveImage(img_addr.c_str());
    } else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "RVC Camera capture failed!" << std::endl;
    }

    // Close RVC Camera.
    x2.Close();

    // Destroy RVC Camera.
    RVC::X2::Destroy(x2);

    // Shutdown RVC System.
    RVC::SystemShutdown();

    return 0;
}
