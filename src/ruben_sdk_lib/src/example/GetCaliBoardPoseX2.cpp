#include <RVC/RVC.h>

#include <iostream>
#include <opencv2/imgcodecs.hpp>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"
#include "ruben_sdk_lib/Utils/CaliBoardUtils.h"

void Help() {
    printf(
        "usage: ./GetCaliBoardPoseX2 exposure_time_ms cicle_center_distance_m circle_radius_m\n\ncircle "
        "center distance(circle step) and circle radius can be found in calibration board\n\n");
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        Help();
        exit(0);
    }
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

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    // set capture option parameter
    RVC::X2::CaptureOptions cap_opts;
    x2.LoadCaptureOptionParameters(cap_opts);
    cap_opts.transform_to_camera = camera_id;
    cap_opts.exposure_time_2d = std::atoi(argv[1]);
    cap_opts.exposure_time_3d = std::atoi(argv[1]);
    cap_opts.use_projector_capturing_2d_image = true;

    // set calibration board detection parameter
    float extrinsic_matrix[16], intrinsic_matrix[9], distortion[5];
    x2.GetExtrinsicMatrix(camera_id, extrinsic_matrix);
    x2.GetIntrinsicParameters(camera_id, intrinsic_matrix, distortion);
    // pixel distance in Z plane where Z equals to calibration board position Z.
    const double pixel_dist_m = extrinsic_matrix[11] / intrinsic_matrix[0];
    const double circle_radius_m = std::atof(argv[3]), circle_radius_pixel = circle_radius_m / pixel_dist_m;

    const int calib_board_size[2] = {4, 11};            // {points_per_row, points_per_colum}
    const double center_distance = std::atof(argv[2]);  // unit: m

    cv::SimpleBlobDetector::Params blob_detector_param;
    blob_detector_param.filterByColor = true;
    blob_detector_param.blobColor = 255;
    blob_detector_param.filterByArea = true;
    blob_detector_param.minArea = circle_radius_pixel * circle_radius_pixel * 3.14159 / 3;  // unit: pixel
    blob_detector_param.maxArea = circle_radius_pixel * circle_radius_pixel * 3.14159 * 3;  // unit: pixel
    blob_detector_param.filterByCircularity = true;
    blob_detector_param.minCircularity = 0.5;

    printf("area threshold pixel: %f, %f\n", blob_detector_param.minArea, blob_detector_param.maxArea);

    // Capture a point map and a image with default setting.
    if (x2.Capture(cap_opts) == true) {
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
        double R_in_camera[9], T_in_camera[3];
        cv::Mat drawed_image;
        bool found = GetCaliBoardPose(pm, img, center_distance, calib_board_size, blob_detector_param, R_in_camera,
                                      T_in_camera, &drawed_image);
        cv::imwrite(save_directory + "drawedImage.png", drawed_image);
        if (!found) {
            printf(
                "Get pose failed. please adjust parameter according to ./Data/drawedImage.png. if calibboard in 2d "
                "image is detected, then adjust exposure time of 3d, otherwise, adjust blobdetector parameter\n");
        } else {
            PrintHomogeneousMatrix(R_in_camera, T_in_camera);
            // transform point cloud from camera coordinate to calibBoard coordinate
            double R_in_calib[9], T_in_calib[3];
            RigidTransformInv(R_in_camera, T_in_camera, R_in_calib, T_in_calib);
            RigidTransformPointMap(pm, R_in_calib, T_in_calib);
            pm_addr = save_directory + "pointmap_in_calib.ply";
            std::cout << "save transformed point map to file: " << pm_addr << std::endl;
            pm.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true);

        }
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