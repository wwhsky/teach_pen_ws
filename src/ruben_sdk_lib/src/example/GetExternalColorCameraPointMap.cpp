#include <RVC/RVC.h>
#include <RVC/experimental/ExternalColorCamera.h>

#include <iostream>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"

// This example shows the process of acquiring the point cloud of an external camera. Note that this program cannot be
// run directly. The part that acquires the image of the external camera needs to be modified.
int main(int argc, char *argv[]) {
    // Prepare work: Users need to calibrate the external camera themselves. 
    // The quality of calibration directly affects the quality of final result.
    float external_camera_intrinsic_matrix[9] = {2400, 0, 1000, 0, 2400, 750, 0, 0, 1};         // Needs to be modified.
    float external_camera_camera_distortion[5] = {-0.099f, 0.218f, 0.00037f, 0.0001f, -0.26f};  // Needs to be modified.
    int external_camera_image_width = 0;                                                        // Needs to be modified.
    int external_camera_image_height = 0;                                                       // Needs to be modified.

    // 1. Initialize
    RVC::SystemInit();
    RVC::Device devices[10];
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);
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
    RVC::X2 x2 = RVC::X2::Create(devices[0]);
    x2.Open();
    if (!x2.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X2." << std::endl;
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return -1;
    }
    RVC::DeviceInfo info;
    devices[0].GetDeviceInfo(&info);
    if (info.support_extra == true) {
        std::cout << "The camera has already support color camera!" << std::endl;
        x2.Close();
        RVC::X2::Destroy(x2);
        RVC::SystemShutdown();
        return -1;
    }
    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    // 2. Capture three point maps with 4*11 calibration board to get external camera extrinsic matrix.
    float external_camera_extrinsic_matrix[16];
    {
        // 2.1 capture first point map
        std::cout << "Put the calibration board at the nearest working distance and take first frame.\nPress Enter to "
                     "continue..."
                  << std::endl;
        std::cin.get();
        x2.Capture();
        RVC::PointMap internal_camera_point_map0 = x2.GetPointMap().Clone();
        internal_camera_point_map0.Clone();
        RVC::Image internal_camera_image0 = x2.GetImage(RVC::CameraID_Left).Clone();
        unsigned char *external_camera_image_data0 = nullptr;  // Get external image. Needs to be modified.

        std::string pm_addr = save_directory + "calibration_internal0.ply";
        internal_camera_point_map0.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true, internal_camera_image0);
        std::string img_addr = save_directory + "calibration_internal0.png";
        internal_camera_image0.SaveImage(img_addr.c_str());

        // 2.2 Capture second point map
        std::cout << "Put the calibration board at the middle working distance and take second frame.\nPress Enter to "
                     "continue..."
                  << std::endl;
        std::cin.get();
        x2.Capture();
        RVC::PointMap internal_camera_point_map1 = x2.GetPointMap().Clone();
        RVC::Image internal_camera_image1 = x2.GetImage(RVC::CameraID_Left).Clone();
        unsigned char *external_camera_image_data1 = nullptr;  // Get external image. Needs to be modified.

        pm_addr = save_directory + "calibration_internal1.ply";
        internal_camera_point_map1.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true, internal_camera_image1);
        img_addr = save_directory + "calibration_internal1.png";
        internal_camera_image1.SaveImage(img_addr.c_str());

        // 2.3 Capture third point map
        std::cout << "Put the calibration board at the furthest working distance and take third frame.\nPress Enter to "
                     "continue..."
                  << std::endl;
        std::cin.get();
        x2.Capture();
        RVC::PointMap internal_camera_point_map2 = x2.GetPointMap().Clone();
        RVC::Image internal_camera_image2 = x2.GetImage(RVC::CameraID_Left).Clone();
        unsigned char *external_camera_image_data2 = nullptr;  // Get external image. Needs to be modified.

        pm_addr = save_directory + "calibration_internal2.ply";
        internal_camera_point_map2.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true, internal_camera_image2);
        img_addr = save_directory + "calibration_internal2.png";
        internal_camera_image2.SaveImage(img_addr.c_str());
        
        // 2.4 Get external camera extrinsic matrix
        double error;
        int error_code = RVC::GetExternalCameraExtrinsicMatrix(
            internal_camera_image0, internal_camera_point_map0, external_camera_image_data0, internal_camera_image1,
            internal_camera_point_map1, external_camera_image_data1, internal_camera_image2, internal_camera_point_map2,
            external_camera_image_data2, external_camera_image_width, external_camera_image_height,
            external_camera_intrinsic_matrix, external_camera_camera_distortion,
            external_camera_extrinsic_matrix, error);
        if (error_code != 0) {
            std::cout << "Get external camera extrinsic matrix failed! Error code: " << error_code << std::endl;
            x2.Close();
            RVC::X2::Destroy(x2);
            RVC::SystemShutdown();
            return -1;
        }

        // print extrinsic matrix and error
        std::cout << "External camera extrinsic matrix: " << std::endl;
        for (int i = 0; i < 16; i++) {
            std::cout << external_camera_extrinsic_matrix[i];
            if (i != 15) {
                std::cout << ", ";
            }
        }
        std::cout << std::endl;
        std::cout << "Reprojection error(pixel): " << error << std::endl;
    }

    // 3. After calibration, we can capture a point map with RVC camera and the corresponding image by external camera and get external camera point map.
    bool capture_success = x2.Capture(); 
    RVC::PointMap internal_camera_point_map = x2.GetPointMap();
    RVC::Image internal_camera_image = x2.GetImage(RVC::CameraID_Left);

    std::string pm_addr = save_directory + "scene_internal.ply";
    internal_camera_point_map.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true, internal_camera_image);
    std::string img_addr = save_directory + "scene_internal.png";
    internal_camera_image.SaveImage(img_addr.c_str());

    RVC::PointMap external_camera_point_map = RVC::GetExternalCameraPointMap(
        internal_camera_point_map, external_camera_image_width, external_camera_image_height,
        external_camera_intrinsic_matrix, external_camera_camera_distortion, external_camera_extrinsic_matrix);

    std::string pm_addr2 = save_directory + "scene_external.ply";
    external_camera_point_map.Save(pm_addr2.c_str(), RVC::PointMapUnit::Meter, true);

    x2.Close();
    RVC::X2::Destroy(x2);
    RVC::SystemShutdown();
    return 0;
}
