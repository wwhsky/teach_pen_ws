#include <ruben_sdk_lib/IO/FileIO.h>
#include <RVC/RVC.h>
#include <RVC/experimental/PointCloudCompensator.h>

#include <iostream>
#include <tuple>

class CameraHelper {
public:
    CameraHelper() { RVC::SystemInit(); }
    ~CameraHelper() {
        if (m_x2.IsOpen()) {
            m_x2.Close();
        }
        if (m_x2.IsValid()) {
            RVC::X2::Destroy(m_x2);
        }
        RVC::SystemShutdown();
    }
    bool Open() {
        // List Devices
        RVC::Device devices[10];
        size_t actual_size = 0;
        SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);
        if (actual_size == 0) {
            return false;
        }

        if(devices[0].IsFirmwareMatch() == false){
            std::cout << "device firmware mismatch, Please use RVCManager to upgrade the firmware" << std::endl;
            RVC::SystemShutdown();
            return -1;
        }
        // Create and open camera
        m_x2 = RVC::X2::Create(devices[0]);
        bool is_open_success = m_x2.Open();
        if (!is_open_success) {
            std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X2." << std::endl;
            RVC::X2::Destroy(m_x2);
            return false;
        }
        RVC::DeviceInfo info;
        devices[0].GetDeviceInfo(&info);
        camera_id = RVC::CameraID_Left;
        if (info.support_extra) {
            camera_id = RVC::CameraID_Extra;
        }
        // Load capture options and set the coordinate system to the camera coordinate system, because the
        // Compensator_ROIs requires that the point cloud must be under the camera coordinate system.
        bool is_success_load_capture_options = m_x2.LoadCaptureOptionParameters(m_opts);
        m_opts.transform_to_camera = camera_id;
        RVC::X2::CustomTransformOptions custom_transform_opt;
        custom_transform_opt.coordinate_select = RVC::X2::CustomTransformOptions::CoordinateSelect_Disabled;
        bool is_success_set_transformation = m_x2.SetCustomTransformation(custom_transform_opt);
        return is_success_load_capture_options && is_success_set_transformation;
    }
    std::pair<RVC::PointMap, RVC::Image> Capture() {
        bool is_capture_success = m_x2.Capture(m_opts);
        if (!is_capture_success) {
            return {RVC::PointMap(), RVC::Image()};
        }
        return {m_x2.GetPointMap(), m_x2.GetImage(camera_id)};
    }

private:
    RVC::X2 m_x2;
    RVC::X2::CaptureOptions m_opts;
    RVC::CameraID camera_id;
};

int CompensateForX2() {
    CameraHelper camera_helper;
    bool is_open_success = camera_helper.Open();
    if (!is_open_success) {
        std::cout << "RVC Camera fails to open!" << std::endl;
        return -1;
    }

    RVC::Compensator_FixedMarkers compensator;

    // 1. Capture a reference point map in static scene and initialize compensator.
    RVC::PointMap pm;
    RVC::Image image;
    std::tie(pm, image) = camera_helper.Capture();
    int ret = compensator.Initialize(pm, image, 0); // 0 for calibration board, 1 for concentric circles
    if (ret != 0) {
        std::cout << "Initialize fails! Error code: " << ret << "." << std::endl;
        return -1;
    }
    std::cout << "Initialize successfully!" << std::endl;

    std::string dir = "./Data/";
    MakeDirectories(dir);
    // 2. Capture the fixed markers and update compensation model.
    // It is recommended to update the model within every 50 captures.
    // In this sample program, there are only 10 frames in between as a demonstration.
    int intervalBetweenCompensation = 10;  
    std::tie(pm, image) = camera_helper.Capture();
    pm.Save((dir + "/pointcloud_0.ply").c_str(), RVC::PointMapUnit::Meter, true, image);
    image.SaveImage((dir + "/image_0.png").c_str());
    while (intervalBetweenCompensation--) {
        camera_helper.Capture();
    }
    std::tie(pm, image) = camera_helper.Capture();
    double drift_distance = 0;
    ret = compensator.Update(pm, image, drift_distance);
    if (ret != 0) {
        std::cout << "Update compensation model fails! Error code: " << ret << "." << std::endl;
        return -1;
    }
    std::cout << "Current drift distance: " << drift_distance << std::endl;
    std::cout << "Update compensation model successfully!" << std::endl;

    // 3. After update compensation model, we can compensate for the point cloud.
    std::tie(pm, image) = camera_helper.Capture();
    pm.Save((dir + "/pointcloud_n_before_compensate.ply").c_str(), RVC::PointMapUnit::Meter, true, image);
    image.SaveImage((dir + "/image_n.png").c_str());
    compensator.Apply(pm);
    pm.Save((dir + "/pointcloud_n_after_compensate.ply").c_str(), RVC::PointMapUnit::Meter, true, image);
    std::cout << "Compensate pm successfully!" << std::endl;

    return 0;
}

int main(int argc, char *argv[]) {
    /*
     * Use case: Compensate for the point cloud drift caused by changes of camera or ambient temperature.
     *
     * Process:
     * 1. Capture the fixed markers to initialize;
     * 2. Keep the camera in the same position as it was initialized and capture the same fixed markers to update the
     * model;
     * 3. Continue to use the camera to take n frames, and for each frame we can apply the updated model to compensate;
     * 4. Repeat the process of 2 and 3.
     * It is recommended that n should be within 50. And the first step is recommended to be done after
     * hand-eye calibration.
     */
    CompensateForX2();

    return 0;
}