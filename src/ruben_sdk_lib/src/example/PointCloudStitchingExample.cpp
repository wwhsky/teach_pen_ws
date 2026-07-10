#include <RVC/RVC.h>
#include <RVC/experimental/PointCloudStitching.h>
#include <iostream>
#include <fstream>
#include <array>
#ifdef _WIN32
#include <direct.h>
#include <io.h>
#define MKDIR(path) _mkdir(path)
#else
#include <sys/stat.h>
#include <unistd.h>
#define MKDIR(path) mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif

void MakeDirectories(const std::string& directories) {
    MKDIR(directories.c_str());
}

void InversionRt(const double* R, const double* t, double* R_inv, double* t_inv) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_inv[i * 3 + j] = R[j * 3 + i];
        }
    }
    for (int i = 0; i < 3; i++) {
        t_inv[i] = 0;
        for (int j = 0; j < 3; j++) {
            t_inv[i] -= R_inv[i * 3 + j] * t[j];
        }
    }
}

void StitchOnline() {
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
    
    RVC::Device device = devices[0];
    // or find device by SN: 
    // RVC::Device device = RVC::SystemFindDevice("G2GMY20B005");

    RVC::X2 x2 = RVC::X2::Create(device);
    x2.Open();
    if (!x2.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected "
                     "and make sure it is not occupied and supports X2!"
                  << std::endl;
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
    std::string saveFolder = +"./output/";
    MakeDirectories(saveFolder);

    x2.Capture();
    RVC::PointMap point_map0 = x2.GetPointMap();
    RVC::Image image0 = x2.GetImage(camera_id);
    point_map0.Save((saveFolder + "/point_map0.ply").c_str(), RVC::PointMapUnit::Millimeter, true, image0);
    image0.SaveImage((saveFolder + "/image0.png").c_str());

    std::vector<RVC::PointMap> point_maps;
    std::vector<RVC::Image> images;
    point_maps.push_back(point_map0.Clone());
    images.push_back(image0.Clone());

    const int total_stitch_count = 3;
    std::vector<std::array<double, 9>> Rs(total_stitch_count);
    std::vector<std::array<double, 3>> ts(total_stitch_count);

    for (int count = 1; count < total_stitch_count; count++) {
        std::cout << "Move camera and take next frame. Press Enter to continue..." << std::endl;
        std::cin.get();
        x2.Capture();
        RVC::PointMap point_map = x2.GetPointMap();
        RVC::Image image = x2.GetImage(camera_id);
        point_map.Save((saveFolder + "/point_map" + std::to_string(count) + ".ply").c_str(),
                       RVC::PointMapUnit::Millimeter, true, image);
        image.SaveImage((saveFolder + "/image" + std::to_string(count) + ".png").c_str());
        RVC::CodedCircleMarkerType type; 
        // **** Notice: need to be changed according to the actual type. ****
        type.N = 15;
        type.r1_to_r0_ratio = 4.0f / 1.5f;
        type.r2_to_r0_ratio = 6.0f / 1.5f;
        int ret = RVC::GetTwoCameraTransformByCodedCircleMarker(point_maps.back(), images.back(), point_map, image,
                                                                type, Rs[count].data(), ts[count].data());
        if (ret != 0) {
            std::cout << "Failed to get two camera transform by coded circle marker, ret: " << ret << std::endl;
            x2.Close();
            RVC::X2::Destroy(x2);
            RVC::SystemShutdown();
            return;
        }
        RVC::TransformPointCloud(Rs[count].data(), ts[count].data(), point_map);
        point_map.Save((saveFolder + "/point_map" + std::to_string(count) + "_transformed.ply").c_str(),
                       RVC::PointMapUnit::Millimeter, true, image);

        point_maps.push_back(point_map.Clone());
        images.push_back(image.Clone());
    }

    // The above is to convert all point clouds to the first camera coordinate system.
    // Option: the following is to convert all point clouds to the last camera coordinate system.
    {
        std::array<double, 9> R_inv;
        std::array<double, 3> t_inv;
        InversionRt(Rs.back().data(), ts.back().data(), R_inv.data(), t_inv.data());
        for (int i = 0; i < (int)point_maps.size() - 1; i++) {
            RVC::TransformPointCloud(R_inv.data(), t_inv.data(), point_maps[i]);
            point_maps[i].Save((saveFolder + "/point_map" + std::to_string(i) + "_transformed_to_last.ply").c_str(),
                               RVC::PointMapUnit::Millimeter, true, images[i]);
        }
    }

    // return resources to RVC system
    for (int i = 0; i < point_maps.size(); i++) {
        RVC::PointMap::Destroy(point_maps[i]);
        RVC::Image::Destroy(images[i]);
    }
    x2.Close();
    RVC::X2::Destroy(x2);
    RVC::SystemShutdown();
    return;
}

// This function shows how to stitch three point clouds into the coordinate system of the first point cloud.
void StitchOffline(const std::string& folder, const std::vector<std::string>& ply_names,
                   const std::vector<std::string>& image_names) {
    // Convert point_map1 to the coordinate system of the point_map0
    RVC::Image image0 = RVC::Image::CreateFromFile((folder + image_names[0]).c_str());
    RVC::Image image1 = RVC::Image::CreateFromFile((folder + image_names[1]).c_str());

    RVC::PointMap point_map0 = RVC::PointMap::CreateFromFile((folder + ply_names[0]).c_str(), image0.GetSize(),
                                                             RVC::PointMapUnit::Enum::Meter);
    RVC::PointMap point_map1 = RVC::PointMap::CreateFromFile((folder + ply_names[1]).c_str(), image1.GetSize(),
                                                             RVC::PointMapUnit::Enum::Meter);

    if (!image0.IsValid() || !image1.IsValid() || !point_map0.IsValid() || !point_map1.IsValid()) {
        std::cout << "Failed to load image or point cloud" << std::endl;
        return;
    }

    double R1[9];
    double t1[3];
    RVC::CodedCircleMarkerType type; // Need to be changed according to the actual type
    type.N = 15;
    type.r1_to_r0_ratio = 4.0f / 1.5f;
    type.r2_to_r0_ratio = 6.0f / 1.5f;
    int ret = RVC::GetTwoCameraTransformByCodedCircleMarker(point_map0, image0, point_map1, image1, type, R1, t1);
    if (ret != 0) {
        std::cout << "Failed to get two camera transform by coded circle marker, ret: " << ret << std::endl;
        return;
    }
    RVC::TransformPointCloud(R1, t1, point_map1);

    std::string save_folder = folder + "output/";
    MakeDirectories(save_folder);
    point_map1.Save((save_folder + "output1.ply").c_str(), RVC::PointMapUnit::Enum::Meter, true, image1);

    if (ply_names.size() < 3 || image_names.size() < 3) {
        return;
    }
    // Convert point_map2 to the coordinate system of the point_map0
    RVC::Image image2 = RVC::Image::CreateFromFile((folder + image_names[2]).c_str());
    RVC::PointMap point_map2 = RVC::PointMap::CreateFromFile((folder + ply_names[2]).c_str(), image2.GetSize(),
                                                             RVC::PointMapUnit::Enum::Meter);

    double R2[9];
    double t2[3];
    ret = RVC::GetTwoCameraTransformByCodedCircleMarker(point_map1, image1, point_map2, image2, type, R2, t2);
    if (ret != 0) {
        std::cout << "Failed to get two camera transform by coded circle marker, ret: " << ret << std::endl;
        return;
    }

    RVC::TransformPointCloud(R2, t2, point_map2);
    point_map2.Save((save_folder + "output2.ply").c_str(), RVC::PointMapUnit::Enum::Meter, true, image2);

    // If use CreateFromFile(), you should call Destroy() to release the resource to RVC system.
    RVC::PointMap::Destroy(point_map0);
    RVC::PointMap::Destroy(point_map1);
    RVC::PointMap::Destroy(point_map2);
    RVC::Image::Destroy(image0);
    RVC::Image::Destroy(image1);
    RVC::Image::Destroy(image2);

    return;
}

// User guild: RVCSDK/docs/PointCloudStitchingManual.pdf
/*
This example shows how to stitch three point clouds into the coordinate system of the first point cloud.
If you have more than three point clouds, the process is the same.
To generate coded circle marker pattern, see: Examples/Python/Utils/GenerateCodedCircle.py
*/
int main() {
    StitchOnline();
    return 0;

    std::string folder = "./Data/";
    std::vector<std::string> plyNames = {"0.ply", "1.ply", "2.ply"};
    std::vector<std::string> imageNames = {"0.png", "1.png", "2.png"};
    StitchOffline(folder, plyNames, imageNames);
    return 0;
}