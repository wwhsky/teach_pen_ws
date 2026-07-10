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
    std::cout << info.name << "-" << info.sn << std::endl;

    // Capture Options
    RVC::X2::CaptureOptions options;
    x2.LoadCaptureOptionParameters(options);
    std::cout << "Print capture options---" << std::endl;
    {
        /**
         * P and I series supports the following modes:
         * CaptureMode_Normal
         * CaptureMode_Fast
         * CaptureMode_AntiInterReflection
         *
         * G series supports the following modes:
         * CaptureMode_Ultra
         * CaptureMode_AntiInterReflection
         *
         * M series supports the following modes:
         * CaptureMode_Ultra
         * CaptureMode_AntiInterReflection
         * CaptureMode_SwingLineScan
         * CaptureMode_FixedLineScan
         *
         * Note:The CaptureMode_Robust has been deprecated. You can achieve the same effect by setting the
         * 'scan_times' parameter.
         */

        std::string captureMode = options.capture_mode == RVC::CaptureMode_Normal  ? "CaptureMode_Normal"
                                  : options.capture_mode == RVC::CaptureMode_Fast  ? "CaptureMode_Fast"
                                  : options.capture_mode == RVC::CaptureMode_Ultra ? "CaptureMode_Ultra"
                                  : options.capture_mode == RVC::CaptureMode_AntiInterReflection
                                      ? "CaptureMode_AntiInterReflection"
                                  : options.capture_mode == RVC::CaptureMode_SwingLineScan ? "CaptureMode_SwingLineScan"
                                  : options.capture_mode == RVC::CaptureMode_FixedLineScan ? "CaptureMode_FixedLineScan"
                                                                                           : "Error";

        std::cout << "[capture_mode] = {" << captureMode << "}" << std::endl;

        // Parameters related to 2D Capture
        std::cout << "[exposure_time_2d] = {" << options.exposure_time_2d << "}" << std::endl;
        std::cout << "[gain_2d] = {" << options.gain_2d << "}" << std::endl;
        std::cout << "[gamma_2d] = {" << options.gamma_2d << " }" << std::endl;
        std::cout << "[use_projector_capturing_2d_image] = {" << options.use_projector_capturing_2d_image << "}"
                  << std::endl;

        // Parameters related to 3D Capture
        std::cout << "[enable_2d_in_capture] = {" << options.enable_2d_in_capture << "}" << std::endl;
        std::cout << "[exposure_time_3d] = {" << options.exposure_time_3d << "}" << std::endl;
        std::cout << "[gain_3d] = {" << options.gain_3d << "}" << std::endl;
        std::cout << "[gamma_3d] = {" << options.gamma_3d << "}" << std::endl;
        std::cout << "[light_contrast_threshold] = {" << options.light_contrast_threshold << "}" << std::endl;
        std::cout << "[projector_brightness] = {" << options.projector_brightness << "}" << std::endl;
        // Note:Only the G and M series support the 'scan_times' parameter.
        std::cout << "[scan_times] = {" << options.scan_times << "}" << std::endl;

        // Note:Only M series support the 'line_scanner_*' and 'correspond2d' parameters.
        std::cout << "[line_scanner_exposure_time_us] = {" << options.line_scanner_exposure_time_us << "}" << std::endl;
        std::cout << "[line_scanner_max_distance] = {" << options.line_scanner_max_distance << "}" << std::endl;
        std::cout << "[line_scanner_min_distance] = {" << options.line_scanner_min_distance << "}" << std::endl;
        std::cout << "[line_scanner_scan_time_ms] = {" << options.line_scanner_scan_time_ms << "}" << std::endl;
        std::cout << "[line_scanner_laser_position] = {" << options.line_scanner_laser_position << "}" << std::endl;
        std::cout << "[line_scanner_brightness_threshold] = {" << options.line_scanner_brightness_threshold << "}" << std::endl;
        std::cout << "[correspond2d] = {" << options.correspond2d << "}" << std::endl;

        /**
         * Currently supported HDR modes include:
         * CaptureMode_Normal
         * CaptureMode_Fast
         * CaptureMode_Ultra
         * CaptureMode_AntiInterReflection
         */
        std::cout << "[hdr_exposure_times] = {" << options.hdr_exposure_times << "}" << std::endl;
        std::cout << "[hdr_exposuretime_content] = {" << options.hdr_exposuretime_content[0] << ","
                  << options.hdr_exposuretime_content[1] << "," << options.hdr_exposuretime_content[2] << "}"
                  << std::endl;
        std::cout << "[hdr_gain_3d] = {" << options.hdr_gain_3d[0] << "," << options.hdr_gain_3d[1] << ","
                  << options.hdr_gain_3d[2] << "}" << std::endl;
        std::cout << "[hdr_projector_brightness] = {" << options.hdr_projector_brightness[0] << ","
                  << options.hdr_projector_brightness[1] << "," << options.hdr_projector_brightness[2] << "}"
                  << std::endl;
        // Note:Only the G and M series support the 'hdr_scan_times' parameter.
        std::cout << "[hdr_scan_times] = {" << options.hdr_scan_times[0] << "," << options.hdr_scan_times[1] << ","
                  << options.hdr_scan_times[2] << "}" << std::endl;

        // Parameters related to post-processing
        std::cout << "[confidence_threshold] = {" << options.confidence_threshold << "}" << std::endl;
        std::cout << "[use_auto_noise_removal] = {" << options.use_auto_noise_removal << "}" << std::endl;
        std::cout << "[noise_removal_distance] = {" << options.noise_removal_distance << "}" << std::endl;
        std::cout << "[noise_removal_point_number] = {" << options.noise_removal_point_number << "}" << std::endl;
        std::cout << "[reflection_filter_threshold] = {" << options.reflection_filter_threshold << "}" << std::endl;
        std::cout << "[smooth_sigma] = {" << options.smooth_sigma << "}" << std::endl;
        std::cout << "[downsample_distance] = {" << options.downsample_distance << "}" << std::endl;
        std::cout << "[calc_normal] = {" << options.calc_normal << "}" << std::endl;
        std::cout << "[calc_normal_radius] = {" << options.calc_normal_radius << "}" << std::endl;

        std::cout << "ROI(x,y,w,h)--[roi] = {" << options.roi.x << "," << options.roi.y << "," << options.roi.width
                  << "," << options.roi.height << "}" << std::endl;
        std::cout << "[pointcloud_completion] = {" << options.pointcloud_completion << "}" << std::endl;
    }

    bool ret;
    // Method 1, Use Default Options and Modify.
    {
        // options = RVC::X2::CaptureOptions();
        // todo:modify options
        // ret = x2.Capture(options);
    }

    // Method 2 , Load Options From Camera and Modify.
    {
        // x2.LoadCaptureOptionParameters(options);
        // todo:modify options
        // ret = x2.Capture(options);
    }

    // *****
    // Method 3 , Using the internal parameters of the camera.
    {
        // We suggest adjusting the camera parameters By RVC Manager.
        // Then, when we use SDK, we directly use the parameters inside the camera.
        ret = x2.Capture();
    }

    // Method 4 , Load Options From File and Capture.
    {
        // If you need to capture multiple scenes, their parameters are different.
        // You can save these parameters as different configuration files  By RVC Manager and then import the files
        // before capture. ret = x2.LoadSettingFromFile("file.json"); ret = x2.Capture();
    }

    if (ret == false) {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        x2.Close();
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return -1;
    }

    // Data Process
    std::string dir = "./Data/";
    MakeDirectories(dir);
    dir += info.sn;
    MakeDirectories(dir);
    dir += "/x2";
    MakeDirectories(dir);

    RVC::PointMap pointcloud = x2.GetPointMap();
    RVC::Image image = x2.GetImage(camera_id);
    pointcloud.Save((dir + "/pointcloud.ply").c_str(), RVC::PointMapUnit::Millimeter, true);
    pointcloud.Save((dir + "/pointcloud_color.ply").c_str(), RVC::PointMapUnit::Millimeter, true, image);
    image.SaveImage((dir + "/image.png").c_str());

    // Close RVC Camera.
    x2.Close();
    RVC::X2::Destroy(x2);
    RVC::SystemShutdown();

    return 0;
}
