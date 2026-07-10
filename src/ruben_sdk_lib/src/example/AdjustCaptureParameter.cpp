#include <RVC/RVC.h>

#include <algorithm>
#include <iostream>
#include <string>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"

void Help(char **argv) {
    // clang-format off
    printf(" \
    usage: %s\n\
        [ -h, --help ] show help \n\
        [ --calc_normal value ] \n\
                        Whether to calculate the normal of 3D points. 0 means calculate 3D points without normal. 1 means calculate 3D points with normal., default value = 0\n\
        [ --calc_normal_radius value] \n\
                        Calculate 3D points normal radius in pixel, value > 0. default value = 5\n\
        [ --transform_to_camera value] \n\
                        The point map is calculated in reference plane coordinate. 0 means calculate point map in calibration board coordinate. 1 means transform point map to camera coordinate., default value = 0\n\
        [ --filter_range value] \n\
                        Control noise removal, [0, 30]. the larger value the stronger denoising. default value = 0\n\
        [ --phase_filter_range value] \n\
                        Control phase filter to remove noisy points by phase difference, [0, 40]. the larger value the stronger denoising. default value = 0\n\
        [ --projector_brightness value] \n\
                        Control projector brightness [1, 240]. the larger value the brighter of projector projection. default value = 240\n\
        [ --exposure_time_2d value] \n\
                        2D image exposure time. Capture with white light, range  [11, 100]ms, others [3, 100]ms. default value = 11\n\
        [ --exposure_time_3d value] \n\
                        3D point map exposure time. Capture with white light, range  [11, 100]ms, others [3, 100]ms. default value = 11\n\
        [ --gain_2d value] \n\
                        2D image gain. The 2D gain value of each series cameras is different, you can call function GetGainRange() to get specific range. default value = 0\n\
        [ --gain_3d value] \n\
                        3D point map gain.  The 3D gain value of each series cameras is different, you can call function GetGainRange() to get specific range. default value = 0\n\
        [ --gamma_2d value] \n\
                        2D image gamma. The 2D gamma value of each series cameras is different, you can call function GetGammaRange() to get specific range. default value = 1\n\
        [ --gamma_3d value] \n\
                        3D point map gamma.  The 3D gamma value of each series cameras is different, you can call function GetGammaRange() to get specific range. default value = 1\n\
        [ --white_balance_times value] \n\
                        Number of images used for automatic calculation the white balance parameters (0, 20). we commend 10 times for usual case. This function only supports color camera to optimize 2D color image quality.\n\
        [ --bandwidth_ratio value] \n\
                        Camera bandwidth ratio [0.3, 1]. \n\
        [ --use_projector_capturing_2d_image value] \n\
                        Whether open projector when capture 2d image. 1 means open. 0 means not open. Only gray camera setting this option work. default value = 1\n\
        [ --smoothness value ] \n\
                        Smoothness level. 0: Off, 1: weak, 2: normal, 3: strong. default value = 0\n\
        [ --downsample_distance value] \n\
                        downsample distance, unit: m. default value = -1. \n\
        [ --capture_mode value] \n\
                        control capture mode. 0: CaptureMode_Fast, 1: CaptureMode_Normal. default value = 1\n\
    example: \n\
        %s --exposure_time_2d 20 --exposure_time_3d 20 \n\
    \n", argv[0], argv[0]);
    // clang-format on
}

bool CmdOptionExists(const int argc, char **argv, const std::string &option) {
    char **begin = argv, **end = argv + argc;
    return std::find(begin, end, option) != end;
}

const char *GetCmdOption(const int argc, char **argv, const std::string &option) {
    char **begin = argv, **end = argv + argc;
    char **itr = std::find(begin, end, option);
    if (itr != end && ++itr != end) {
        return *itr;
    }
    return 0;
}

RVC::X1::CaptureOptions ParseCaptureOptions(const int argc, char **argv) {
    RVC::X1::CaptureOptions cap_opt;
    char **begin = argv, **end = argv + argc, **itr = begin;
    for (; itr != end; itr++) {
        const std::string name = std::string(*itr);
        if (name == "--calc_normal") {
            itr++;
            cap_opt.calc_normal = std::stoi(*itr);
        }
        if (name == "--calc_normal_radius") {
            itr++;
            cap_opt.calc_normal_radius = std::stoi(*itr);
        }
        if (name == "--transform_to_camera") {
            itr++;
            cap_opt.transform_to_camera = std::stoi(*itr);
        }
        if (name == "--filter_range") {
            itr++;
            cap_opt.filter_range = std::stoi(*itr);
        }
        if (name == "--phase_filter_range") {
            itr++;
            cap_opt.phase_filter_range = std::stoi(*itr);
        }
        if (name == "--projector_brightness") {
            itr++;
            cap_opt.projector_brightness = std::stoi(*itr);
        }
        if (name == "--exposure_time_2d") {
            itr++;
            cap_opt.exposure_time_2d = std::stoi(*itr);
        }
        if (name == "--exposure_time_3d") {
            itr++;
            cap_opt.exposure_time_3d = std::stoi(*itr);
        }
        if (name == "--gain_2d") {
            itr++;
            cap_opt.gain_2d = std::stof(*itr);
        }
        if (name == "--gain_3d") {
            itr++;
            cap_opt.gain_3d = std::stof(*itr);
        }
        if (name == "--gamma_2d") {
            itr++;
            cap_opt.gamma_2d = std::stof(*itr);
        }
        if (name == "--gamma_3d") {
            itr++;
            cap_opt.gamma_3d = std::stof(*itr);
        }
        if (name == "--use_projector_capturing_2d_image") {
            itr++;
            cap_opt.use_projector_capturing_2d_image = std::stoi(*itr);
        }
        if (name == "--smoothness") {
            itr++;
            cap_opt.smoothness = RVC::SmoothnessLevel(std::stoi(*itr));
        }
        if (name == "--downsample_distance") {
            itr++;
            cap_opt.downsample_distance = std::stof(*itr);
        }
        if (name == "--capture_mode") {
            itr++;
            cap_opt.capture_mode = RVC::CaptureMode(std::stoi(*itr));
        }
    }
    return cap_opt;
}

int main(int argc, char **argv) {
    if (CmdOptionExists(argc, argv, "--help") || CmdOptionExists(argc, argv, "-h")) {
        Help(argv);
        exit(0);
    }
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
    // parse capture parameters
    RVC::X1::CaptureOptions cap_opt;
    x1.LoadCaptureOptionParameters(cap_opt);
    cap_opt = ParseCaptureOptions(argc, argv);

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    // For color camera, it needs only once auto white balance before the first capture when the light condition of
    // scene has no big change. white_balace_times is how many images used for automatic calculate the white balance
    // parameters and we commend 10 times for usual case.
    if (CmdOptionExists(argc, argv, "--white_balance_times")) {
        int white_balance_times = std::stoi(GetCmdOption(argc, argv, "--white_balance_times"));
        bool ret = x1.AutoWhiteBalance(white_balance_times);
        if (ret) {
            printf("Success AutoWhiteBalance\n");
        } else {
            printf("Failed AutoWhiteBalance\n");
        }
    }

    // Set camera bandwidth [0.3, 1]
    if (CmdOptionExists(argc, argv, "--bandwidth_ratio")) {
        float bandwidth_ratio = std::stof(GetCmdOption(argc, argv, "--bandwidth_ratio"));
        bool ret = x1.SetBandwidth(bandwidth_ratio);
        if (ret) {
            printf("Success SetBandwidth\n");
        } else {
            printf("Failed SetBandwidth\n");
        }
    }

    // Capture a point map and a image
    if (x1.Capture(cap_opt) == true) {
        std::cout << "RVC Camera capture successed!" << std::endl;
        // Get point map data (m)
        RVC::PointMap pm = x1.GetPointMap();
        std::string pm_addr = save_directory + "test.ply";
        std::cout << "save point map to file: " << pm_addr << std::endl;
        pm.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true);

        // Get image data
        RVC::Image img = x1.GetImage();
        std::string img_addr = save_directory + "test.png";
        std::cout << "save image to file: " << img_addr << std::endl;
        img.SaveImage(img_addr.c_str());
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
