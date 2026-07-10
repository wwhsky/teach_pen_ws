#include <RVC/RVC.h>
#include <RVC/experimental/MarkerDetection.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <ruben_sdk_lib/IO/FileIO.h>

#define USE_X1
#ifdef USE_X1
#define XX X1
#else
#define XX X2
#endif

using namespace RVC;

std::string GetCurrentTimeStr() {
    auto now = std::chrono::system_clock::now();
    std::time_t nowTimeT = std::chrono::system_clock::to_time_t(now);
    std::tm* nowTm = std::localtime(&nowTimeT);
    std::ostringstream oss;
    oss << std::put_time(nowTm, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

cv::Mat ConvertRVCImageToCVMat(const RVC::Image& img) {
	cv::Mat cvImage;
	if (img.GetType() == ImageType::Mono8) {
		cv::Mat cvImageTemp = cv::Mat(img.GetSize().height, img.GetSize().width, CV_8UC1, (void*)img.GetDataConstPtr());
		cv::cvtColor(cvImageTemp, cvImage, cv::COLOR_GRAY2BGR);
	}
	else if (img.GetType() == ImageType::RGB8 || img.GetType() == ImageType::BGR8) {
		cvImage = cv::Mat(img.GetSize().height, img.GetSize().width, CV_8UC3,
			(void*)img.GetDataConstPtr());
	}
	else {
		std::cout << "Unsupported image type!" << std::endl;
	}
	return cvImage;
}

void ShowCodedCircleMarkerResult(const RVC::Image& img, int cvWaitTime) {
    RVC::CodedCircleMarkerType type;
    // **** Notice: need to be changed according to the actual type. ****
    type.N = 15;
    type.r1_to_r0_ratio = 4.0f / 1.5f;
    type.r2_to_r0_ratio = 6.0f / 1.5f;
    int markerNums;
    RVC::CodedCircleMarker markers[1000];
    RVC::DetectCodedCircleMarker(img, type, &markerNums, markers);

    cv::Mat cvImage = ConvertRVCImageToCVMat(img);
    if (cvImage.empty()) {
		std::cout << "Convert RVC Image to CV Mat failed!" << std::endl;
		return;
	}

    for (int i = 0; i < markerNums; i++) {
		cv::circle(cvImage, cv::Point(markers[i].x, markers[i].y), 0, cv::Scalar(0, 0, 255), -1);
		cv::putText(cvImage, std::to_string(markers[i].code), cv::Point(markers[i].x, markers[i].y),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
	}
	std::cout << "markerNums: " << markerNums << std::endl;
    cv::namedWindow("image");
    cv::moveWindow("image", 50, 25);
    int newHeight = 900;
    int newWidth = (int)((double)cvImage.cols / cvImage.rows * newHeight);
    cv::resize(cvImage, cvImage, cv::Size(newWidth, newHeight));
	cv::imshow("image", cvImage);
	cv::waitKey(cvWaitTime);
}

void CodedCircleMarkerDetectOffline(const std::string& folder, const std::string& imageName){
    cv::Mat img = cv::imread(folder + "/" + imageName, cv::IMREAD_GRAYSCALE);
    RVC::Image rvcImg = RVC::Image::Create(RVC::ImageType::Mono8, RVC::Size(img.cols, img.rows), img.data, false);
    ShowCodedCircleMarkerResult(rvcImg, 0);
}

void CodedCircleMarkerDetectOnline() {
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
#ifdef USE_X1
    XX camera = XX::Create(devices[0], RVC::CameraID_Left);
#else
    XX camera = XX::Create(devices[0]);
#endif
    camera.Open();
    if (!camera.IsOpen()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected "
					 "and make sure it is not occupied!" << std::endl;
        RVC::XX::Destroy(camera);
        RVC::SystemShutdown();
        return;
    }

    RVC::DeviceInfo info;
    devices[0].GetDeviceInfo(&info);
#ifndef USE_X1
    RVC::CameraID camera_id = RVC::CameraID_Left;
    if (info.support_extra) {
        camera_id = RVC::CameraID_Extra;
    }
#endif

    // RVC::XX::CaptureOptions cap_opt;
    // cap_opt.exposure_time_2d = 20;
    bool save = false;
    int saveIndex = 0;
    int maxSaveCount = 1000;

    std::string folder = "./DetectMarkersData/";
    MakeDirectories(folder);
    folder = folder + std::string(info.sn) + "/";
    MakeDirectories(folder);
    folder += GetCurrentTimeStr() + "/";
    MakeDirectories(folder);

    // while not press space
    while (cv::waitKey(10) != 32) {
#ifdef USE_X1
        camera.Capture2D();
        RVC::Image img = camera.GetImage();
#else
        camera.Capture2D(camera_id);
        RVC::Image img = camera.GetImage(camera_id);
#endif

        if (save) {
            std::string imagePath = folder + std::to_string(saveIndex) + ".png";
            saveIndex++;
            saveIndex %= maxSaveCount;
            img.SaveImage(imagePath.c_str());
        }

        ShowCodedCircleMarkerResult(img, 1);
    }

    camera.Close();
    XX::Destroy(camera);
    SystemShutdown();
    return;
}

// To generate coded circle marker pattern, see: Examples/Python/Utils/GenerateCodedCircle.py
int main(int argc, char *argv[]) {
    CodedCircleMarkerDetectOnline();
    return 0;
    
    std::string folder = "D:/";
    std::string imageName = "0.png";
    CodedCircleMarkerDetectOffline(folder, imageName);
    return 0;
}
