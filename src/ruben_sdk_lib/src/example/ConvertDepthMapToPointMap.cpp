/**
 * This program illustrates the process of transforming a depth map into a point cloud.
 *
 * Important: When the capture mode is set to `CaptureMode_SwingLineScan` and `correspond2d` is disabled (set to false),
 * the retrieval of a depth map is not feasible. If you require a depth map in the swinglinescan mode, you should enable
 * `correspond2d` (set it to true). However, be advised that enabling `correspond2d` will adjust the original point
 * cloud data. Moreover, while the original point cloud computation uses the sub-pixel center of the laser line, the
 * conversion from depth map to point cloud operates on integer-level pixels, which can lead to minor inaccuracies,with
 * a maximum error not exceeding the point spacing.
 */

#include <RVC/RVC.h>

#include <iostream>
#include <opencv2/calib3d.hpp>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"

static RVC::PointMap ConvertDepthMapToPointMap(RVC::DepthMap &dp, const float intrinsicMatrix[9],
                                               const float distortion[5], RVC::ROI &roi) {
    RVC::Size sz = dp.GetSize();
    const int width = sz.width, height = sz.height, image_size = width * height;
    RVC::PointMap pm;
    pm = RVC::PointMap::Create(RVC::PointMapType::PointsOnly, sz);
    std::vector<cv::Point2f> pts(width * height);
    cv::Point2f *pts_ptr = pts.data();
    for (size_t r = roi.y; r < roi.y + height; r++) {
        for (size_t c = roi.x; c < roi.x + width; c++, pts_ptr++) {
            pts_ptr->x = c;
            pts_ptr->y = r;
        }
    }
    const float distortion_cv[5] = {distortion[0], distortion[1], distortion[3], distortion[4], distortion[2]};
    std::vector<cv::Point2f> undistorted_pts;
    cv::undistortPoints(pts, undistorted_pts, cv::Mat(3, 3, CV_32FC1, const_cast<float *>(intrinsicMatrix)),
                        cv::Mat(1, 5, CV_32FC1, const_cast<float *>(distortion_cv)));
    double *pm_ptr = pm.GetPointDataPtr();
    pts_ptr = undistorted_pts.data();
    const double *z = dp.GetDataPtr();
    for (size_t i = 0; i < image_size; i++, pts_ptr++, pm_ptr += 3, z++) {
        pm_ptr[0] = pts_ptr->x * z[0];
        pm_ptr[1] = pts_ptr->y * z[0];
        pm_ptr[2] = z[0];
    }
    return pm;
}

static bool IsPointMapEqual(RVC::PointMap &pm0, RVC::PointMap &pm1, double &maxDiff, const double tol = 1.0e-6) {
    bool rtf;
    const RVC::Size sz0 = pm0.GetSize(), sz1 = pm1.GetSize();
    rtf = sz0.width == sz1.width && sz0.height == sz1.height;
    maxDiff = -1;
    if (!rtf) {
        return rtf;
    }
    const double *data_ptr0 = pm0.GetPointDataPtr(), *data_ptr1 = pm1.GetPointDataPtr();
    const int data_size = sz0.width * sz0.height * 3;
    double sub_abs;
    bool data0_is_valid, data1_is_valid;
    for (size_t i = 0; i < data_size && rtf; i++) {
        data0_is_valid = data_ptr0[i] == data_ptr0[i];
        data1_is_valid = data_ptr1[i] == data_ptr1[i];
        if (data0_is_valid != data1_is_valid) {
            rtf = false;
        } else if (data0_is_valid) {
            sub_abs = fabs(data_ptr0[i] - data_ptr1[i]);
            if (sub_abs > maxDiff) {
                maxDiff = sub_abs;
            }
        }
    }
    rtf = rtf && maxDiff < tol;
    return rtf;
}

int main(int argc, char *argv[]) {
    // Initialize RVC system.
    RVC::SystemInit();

    // Scan all USB RVC Camera devices.
    RVC::Device devices[10];
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);

    // Find whether any RVC Camera is connected or not.
    if (actual_size == 0) {
        std::cout << "Can not find any USB RVC Camera!" << std::endl;
        return -1;
    }
    if(devices[0].IsFirmwareMatch() == false){
        std::cout << "device firmware mismatch, Please use RVCManager to upgrade the firmware" << std::endl;
        RVC::SystemShutdown();
        return -1;
    }
    // Create a RVC Camera and choose use left side camera.
    RVC::X1 x1 = RVC::X1::Create(devices[0], RVC::CameraID_Left);

    // Open RVC Camera.
    x1.Open();

    // Test RVC Camera is opened or not.
    if (!x1.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X1." << std::endl;
        RVC::X1::Destroy(x1);
        RVC::SystemShutdown();
        return 1;
    }

    RVC::X1::CaptureOptions cap_opts;
    x1.LoadCaptureOptionParameters(cap_opts);
    // We need camera coordinate system data to perform the correct conversion
    cap_opts.transform_to_camera = true;
    RVC::X1::CustomTransformOptions custom_transform_options;
    custom_transform_options.coordinate_select =
        RVC::X1::CustomTransformOptions::CoordinateSelect::CoordinateSelect_Disabled;
    x1.SetCustomTransformation(custom_transform_options);

    const std::string save_directory = "./Data/";
    MakeDirectories(save_directory);

    // Capture a point map and a image with default setting.
    if (x1.Capture(cap_opts) == true) {
        RVC::PointMap pm = x1.GetPointMap();
        std::string pm_addr = save_directory + "test.ply";
        std::cout << "save point map to file: " << pm_addr << std::endl;
        pm.Save(pm_addr.c_str(), RVC::PointMapUnit::Meter, true);

        RVC::DepthMap dp = x1.GetDepthMap();
        float intrinsic_matrix[9], distortion[5];
        x1.GetIntrinsicParameters(intrinsic_matrix, distortion);
        RVC::PointMap convert_pm = ConvertDepthMapToPointMap(dp, intrinsic_matrix, distortion, cap_opts.roi);
        double max_diff;
        bool rtf = IsPointMapEqual(pm, convert_pm, max_diff, 1.0e-6);  // compare accuracy: um
        printf("pointmap is %s, max_diff (mm): %f\n", rtf ? "equal" : "not equal", max_diff * 1000);
    } else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "RVC Camera capture failed!" << std::endl;
    }

    // Close RVC Camera.
    x1.Close();

    // Destroy RVC Camera.
    RVC::X1::Destroy(x1);

    // Shutdown RVC System.
    RVC::SystemShutdown();

    return 0;
}