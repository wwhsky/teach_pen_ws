// Copyright (c) RVBUST, Inc - All rights reserved.

#include <RVC/RVC.h>

#include <fstream>
#include <iostream>

static void PrintCaptureOptions(RVC::X1::CaptureOptions &cap_opt) {
    std::cout << "x1.calc_normal:" << cap_opt.calc_normal << std::endl;
    std::cout << "x1.transform_to_camera:" << cap_opt.transform_to_camera << std::endl;
    std::cout << "x1.filter_range:" << cap_opt.filter_range << std::endl;
    std::cout << "x1.phase_filter_range:" << cap_opt.phase_filter_range << std::endl;
    std::cout << "x1.projector_brightness:" << cap_opt.projector_brightness << std::endl;
    std::cout << "x1.exposure_time_2d:" << cap_opt.exposure_time_2d << std::endl;
    std::cout << "x1.exposure_time_3d:" << cap_opt.exposure_time_3d << std::endl;
    std::cout << "x1.gain_2d:" << cap_opt.gain_2d << std::endl;
    std::cout << "x1.gain_3d:" << cap_opt.gain_3d << std::endl;
    std::cout << "x1.hdr_exposure_times:" << cap_opt.hdr_exposure_times << std::endl;
    std::cout << "x1.hdr_exposuretime_content[0]:" << cap_opt.hdr_exposuretime_content[0] << std::endl;
    std::cout << "x1.hdr_exposuretime_content[1]:" << cap_opt.hdr_exposuretime_content[1] << std::endl;
    std::cout << "x1.hdr_exposuretime_content[2]:" << cap_opt.hdr_exposuretime_content[2] << std::endl;
    std::cout << "x1.calc_normal_radius:" << cap_opt.calc_normal_radius << std::endl;
    std::cout << "x1.gamma_2d:" << cap_opt.gamma_2d << std::endl;
    std::cout << "x1.gamma_3d:" << cap_opt.gamma_3d << std::endl;
    std::cout << "x1.use_projector_capturing_2d_image:" << cap_opt.use_projector_capturing_2d_image << std::endl;
    std::cout << "x1.downsample_distance:" << cap_opt.downsample_distance << std::endl;
    std::cout << "x1.truncate_z_max:" << cap_opt.truncate_z_max << std::endl;
    std::cout << "x1.truncate_z_min:" << cap_opt.truncate_z_min << std::endl;
    std::cout << "x1.scan_times:" << cap_opt.scan_times << std::endl;
    std::cout << "x1.reflection_filter_threshold:" << cap_opt.reflection_filter_threshold << std::endl;
    std::cout << "x1.smooth_sigma:" << cap_opt.smooth_sigma << std::endl;
    std::cout << "x1.enable_2d_in_capture:" << cap_opt.enable_2d_in_capture << std::endl;
}

static void PrintCaptureOptions(RVC::X2::CaptureOptions &cap_opt) {
    std::cout << "x2.transform_to_camera:" << cap_opt.transform_to_camera << std::endl;
    std::cout << "x2.projector_brightness:" << cap_opt.projector_brightness << std::endl;
    std::cout << "x2.calc_normal:" << cap_opt.calc_normal << std::endl;
    std::cout << "x2.calc_normal_radius:" << cap_opt.calc_normal_radius << std::endl;
    std::cout << "x2.light_contrast_threshold:" << cap_opt.light_contrast_threshold << std::endl;
    std::cout << "x2.exposure_time_2d:" << cap_opt.exposure_time_2d << std::endl;
    std::cout << "x2.exposure_time_3d:" << cap_opt.exposure_time_3d << std::endl;
    std::cout << "x2.gain_2d:" << cap_opt.gain_2d << std::endl;
    std::cout << "x2.gain_3d:" << cap_opt.gain_3d << std::endl;
    std::cout << "x2.hdr_exposure_times:" << cap_opt.hdr_exposure_times << std::endl;
    std::cout << "x2.hdr_exposuretime_content[0]:" << cap_opt.hdr_exposuretime_content[0] << std::endl;
    std::cout << "x2.hdr_exposuretime_content[1]:" << cap_opt.hdr_exposuretime_content[1] << std::endl;
    std::cout << "x2.hdr_exposuretime_content[2]:" << cap_opt.hdr_exposuretime_content[2] << std::endl;
    std::cout << "x2.gamma_2d:" << cap_opt.gamma_2d << std::endl;
    std::cout << "x2.gamma_3d:" << cap_opt.gamma_3d << std::endl;
    std::cout << "x2.projector_color:" << cap_opt.projector_color << std::endl;
    std::cout << "x2.use_projector_capturing_2d_image:" << cap_opt.use_projector_capturing_2d_image << std::endl;
    std::cout << "x2.downsample_distance:" << cap_opt.downsample_distance << std::endl;
    std::cout << "x2.scan_times:" << cap_opt.scan_times << std::endl;
    std::cout << "x2.line_scanner_exposure_time_us:" << cap_opt.line_scanner_exposure_time_us << std::endl;
    std::cout << "x2.line_scanner_max_distance:" << cap_opt.line_scanner_max_distance << std::endl;
    std::cout << "x2.line_scanner_min_distance:" << cap_opt.line_scanner_min_distance << std::endl;
    std::cout << "x2.line_scanner_scan_time_ms:" << cap_opt.line_scanner_scan_time_ms << std::endl;
    std::cout << "x2.line_scanner_laser_position:" << cap_opt.line_scanner_laser_position << std::endl;
    std::cout << "x2.line_scanner_brightness_threshold:" << cap_opt.line_scanner_brightness_threshold << std::endl;
    std::cout << "x2.correspond2d:" << cap_opt.correspond2d << std::endl;
    std::cout << "x2.reflection_filter_threshold:" << cap_opt.reflection_filter_threshold << std::endl;
    std::cout << "x2.smooth_sigma:" << cap_opt.smooth_sigma << std::endl;
    std::cout << "x2.enable_2d_in_capture:" << cap_opt.enable_2d_in_capture << std::endl;
    std::cout << "x2.pointcloud_completion:" << cap_opt.pointcloud_completion << std::endl;
}

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
    // Set capture parameters
    RVC::X1::CaptureOptions cap_opt;
    // Transform point map's coordinate to camera
    cap_opt.transform_to_camera = true;
    // Set noise points filter range (0~30)
    cap_opt.filter_range = 0;
    // Set projector brightness (1~240)
    cap_opt.projector_brightness = 200;
    // Set camera exposure time (3~100) ms
    cap_opt.exposure_time_2d = 20;
    cap_opt.exposure_time_3d = 20;

    // Set HDR exposure times [0, 2, 3]. 0 presents not use hdr. 2 and 3 presents hdr times.
    cap_opt.hdr_exposure_times = 3;
    cap_opt.hdr_exposuretime_content[0] = 3;
    cap_opt.hdr_exposuretime_content[1] = 6;
    cap_opt.hdr_exposuretime_content[2] = 12;
    x1.SaveCaptureOptionParameters(cap_opt);

    RVC::X1::CaptureOptions stored_opt_1;
    x1.LoadCaptureOptionParameters(stored_opt_1);
    PrintCaptureOptions(stored_opt_1);

    x1.Close();
    RVC::X1::Destroy(x1);

    RVC::SystemShutdown();
    return;
}

static void UseX2() {
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
        return;
    }
    if(devices[0].IsFirmwareMatch() == false){
        std::cout << "device firmware mismatch, Please use RVCManager to upgrade the firmware" << std::endl;
        RVC::SystemShutdown();
        return;
    }
    // Create and open RVC Camera.
    RVC::Device device = devices[0];
    RVC::X2 x2 = RVC::X2::Create(device);
    x2.Open();
    if (!x2.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X2." << std::endl;
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return;
    }

    RVC::DeviceInfo info;
    device.GetDeviceInfo(&info);
    RVC::CameraID camera_id = RVC::CameraID_Left;
    if (info.support_extra) {
        camera_id = RVC::CameraID_Extra;
    }
    // Set capture parameters
    RVC::X2::CaptureOptions cap_opt;
    // Transform point map's coordinate to camera
    cap_opt.transform_to_camera = camera_id;
    // Set light contrast threshold range (0~10). the default value is 3.
    cap_opt.light_contrast_threshold = 3;
    // Set projector brightness (1~240)
    cap_opt.projector_brightness = 200;
    // Set camera exposure time (3~100) ms
    cap_opt.exposure_time_2d = 20;
    cap_opt.exposure_time_3d = 20;
    // Set projector color
    cap_opt.projector_color = RVC::ProjectorColor_Blue;
    x2.SaveCaptureOptionParameters(cap_opt);

    RVC::X2::CaptureOptions stored_opt_1;
    x2.LoadCaptureOptionParameters(stored_opt_1);
    PrintCaptureOptions(stored_opt_1);

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
