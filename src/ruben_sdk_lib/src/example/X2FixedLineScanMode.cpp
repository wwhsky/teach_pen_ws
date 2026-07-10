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
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not "
                     "occupied and supports X2."
                  << std::endl;
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return -1;
    }
    RVC::DeviceInfo info;
    device.GetDeviceInfo(&info);
    if ((info.support_capture_mode & RVC::CaptureMode_FixedLineScan) == false) {
        std::cout << "The camera does not support FixedLineScan Mode!" << std::endl;
        x2.Close();
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return -1;
    }

    RVC::X2::CaptureOptions cap_opt;
    x2.LoadCaptureOptionParameters(cap_opt);
    // options.capture_mode = CaptureMode_AntiInterReflection;
    // linescan
    cap_opt.capture_mode = RVC::CaptureMode_FixedLineScan;
    // single line
    cap_opt.line_scanner_exposure_time_us = 300;
    cap_opt.projector_brightness = 100;
    cap_opt.gain_3d = 0;
    cap_opt.line_scanner_laser_position = 65536 / 2;
    cap_opt.line_scanner_min_distance = 400;
    cap_opt.line_scanner_max_distance = 1000;
    cap_opt.line_scanner_brightness_threshold = 5;

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    bool ret = x2.StartFixedLineScan(cap_opt);
    if (ret == false) {
        std::cout << "Start Fixed Line Scan failed!" << std::endl;
        x2.Close();
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return -1;
    }
    int total_capture_num = 1000;
    int save_interval_num = 100;
    for (int i = 0; i <= total_capture_num; i++) {
        // RVC::PointMap pm = x2.GetFixedLineScanPointMap();
        RVC::PointMap pm;
        if (x2.GetFixedLineScanPointMap(pm)) {
            if (i % save_interval_num == 0) {
                std::string pm_addr = save_directory + "test_" + std::to_string(i) + ".ply";
                std::cout << "save point map to file: " << pm_addr << std::endl;
                pm.Save(pm_addr.c_str(), RVC::PointMapUnit::Millimeter, true);
            }
        }
    }
    x2.StopFixedLineScan();

    // Close RVC Camera.
    x2.Close();

    // Destroy RVC Camera.
    RVC::X2::Destroy(x2);

    // Shutdown RVC System.
    RVC::SystemShutdown();

    return 0;
}