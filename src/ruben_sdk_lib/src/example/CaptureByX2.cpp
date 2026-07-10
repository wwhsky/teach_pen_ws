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
    // Transform point map's coordinate to left/right/extra camera
    cap_opt.transform_to_camera = camera_id;
    // Set 2d and 3d exposure time. Capture with white light, range [11, 100]ms, others [3, 100]ms.
    cap_opt.exposure_time_2d = 20;
    cap_opt.exposure_time_3d = 20;
    // Set 2d and 3d gain. the default value is 0. The gain value of each series cameras is different, you can call
    // function GetGainRange() to get specific range.
    cap_opt.gain_2d = 0;
    cap_opt.gain_3d = 0;
    // Set 2d and 3d gamma. the default value is 1. The gamma value of each series cameras is different, you can call
    // function GetGammaRange() to get specific range.
    cap_opt.gamma_2d = 1;
    cap_opt.gamma_3d = 1;
    // range in [0, 10]. the default value is 3. The contrast of point less than this value will be treat * as invalid
    // point and be removed.
    cap_opt.light_contrast_threshold = 2;
    // Set projector color. the default value is RVC::ProjectorColor_Blue.
    cap_opt.projector_color = RVC::ProjectorColor_Blue;

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    // Capture a point map and a image with default setting.
    if (x2.Capture(cap_opt) == true) {
        // Get point map data (m).
        RVC::PointMap pm = x2.GetPointMap();
        std::string pm_addr = save_directory + "test.ply";
        std::cout << "save point map to file: " << pm_addr << std::endl;
        pm.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true);

        // Get image data. choose left or right side. the point map is map to left image.
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
