#include "ruben_sdk_lib/IO/SavePointMap.h"

#include <fstream>
#include <iostream>
#include <sstream>

int SavePlyFile(const char *save_address, RVC::PointMap &pm, const bool saveNormal) {
    if (!pm.IsValid()) {
        std::cout << "point map is invalid!" << std::endl;
        return 1;
    }
    std::ofstream file;
    double *normal_data = saveNormal ? pm.GetNormalDataPtr() : nullptr;
    file.open(save_address);
    std::stringstream ss;

    ss << "ply"
       << "\n";
    ss << "format ascii 1.0"
       << "\n";
    ss << "comment Created by Rvbust, Inc"
       << "\n";
    RVC::Size pm_size = pm.GetSize();
    int pm_num = pm_size.rows * pm_size.cols;
    ss << "element vertex " << pm_num << "\n";
    ss << "property float x"
       << "\n";
    ss << "property float y"
       << "\n";
    ss << "property float z"
       << "\n";
    if (normal_data != nullptr) {
        ss << "property float nx"
           << "\n";
        ss << "property float ny"
           << "\n";
        ss << "property float nz"
           << "\n";
    }
    ss << "end_header"
       << "\n";
    double *pm_data = pm.GetPointDataPtr();

    for (int i = 0; i < pm_num; i++) {
        ss << (float)*pm_data << " " << (float)*(pm_data + 1) << " " << (float)*(pm_data + 2);
        if (normal_data != nullptr) {
            ss << " " << (float)*normal_data << " " << (float)*(normal_data + 1) << " " << (float)*(normal_data + 2);
            normal_data += 3;
        }
        ss << "\n";
        pm_data = pm_data + 3;
    }
    file << ss.str();

    file.close();

    return 0;
}

int SavePlyFile(const char *save_address, const double *xyzs, const unsigned int nPts) {
    std::ofstream file;
    file.open(save_address);
    std::stringstream ss;

    ss << "ply"
       << "\n";
    ss << "format ascii 1.0"
       << "\n";
    ss << "comment Created by Rvbust, Inc"
       << "\n";
    int pm_num = nPts;
    ss << "element vertex " << pm_num << "\n";
    ss << "property float x"
       << "\n";
    ss << "property float y"
       << "\n";
    ss << "property float z"
       << "\n";
    ss << "end_header"
       << "\n";

    const double *pm_data = xyzs;
    for (int i = 0; i < pm_num; i++) {
        ss << (float)*pm_data << " " << (float)*(pm_data + 1) << " " << (float)*(pm_data + 2) << "\n";
        pm_data = pm_data + 3;
    }
    file << ss.str();
    file.close();

    return 0;
}

int SavePlyFile(const char *save_address, RVC::PointMap &pm, RVC::Image &image) {
    RVC::Size sz = pm.GetSize();
    const unsigned int w = sz.width, h = sz.height;
    std::ofstream file;
    file.open(save_address);
    std::stringstream ss;

    ss << "ply"
       << "\n";
    ss << "format ascii 1.0"
       << "\n";
    ss << "comment Created by Rvbust, Inc"
       << "\n";
    unsigned int pm_num = w * h;
    ss << "element vertex " << pm_num << "\n";
    ss << "property float x"
       << "\n";
    ss << "property float y"
       << "\n";
    ss << "property float z"
       << "\n";
    ss << "property uchar red"
       << "\n";
    ss << "property uchar green"
       << "\n";
    ss << "property uchar blue"
       << "\n";
    ss << "end_header"
       << "\n";
    const double *pm_data = pm.GetPointDataPtr();
    const unsigned char *img_data = image.GetDataPtr();
    RVC::ImageType::Enum image_type = image.GetType();
    const unsigned int pixel_sz = RVC::ImageType::GetPixelSize(image_type);
    for (int i = 0; i < pm_num; i++) {
        ss << (float)*pm_data << " " << (float)*(pm_data + 1) << " " << (float)*(pm_data + 2) << " ";
        switch (image_type) {
        case RVC::ImageType::Mono8: {
            ss << int(img_data[0]) << " " << int(img_data[0]) << " " << int(img_data[0]) << "\n";
            img_data++;
            break;
        }
        case RVC::ImageType::RGB8: {
            ss << int(img_data[0]) << " " << int(img_data[1]) << " " << int(img_data[2]) << "\n";
            img_data += 3;
            break;
        }
        case RVC::ImageType::BGR8: {
            ss << int(img_data[2]) << " " << int(img_data[1]) << " " << int(img_data[0]) << "\n";
            img_data += 3;
            break;
        }
        default:
            break;
        }
        pm_data = pm_data + 3;
    }
    file << ss.str();
    file.close();

    return 0;
}