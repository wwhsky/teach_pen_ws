#include <RVC/RVC.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include "ruben_sdk_lib/IO/FileIO.h"
#include "ruben_sdk_lib/IO/SavePointMap.h"
struct Point {
    Point() {}
    Point(const float x, const float y, const float z) : x(x), y(y), z(z) {}
    float x;
    float y;
    float z;
};

class RobotControler {
public:
    RobotControler() {}

    void move() {
        // move your robot
        std::cout << "Move from (" << currentPos.x << ", " << currentPos.y << ", " << currentPos.z << ") to ("
                  << nextPos.x << ", " << nextPos.y << ", " << nextPos.z << ")" << std::endl;
        // ...
        currentPos = nextPos;
    }

    void setNextPos(const Point& pos) { nextPos = pos; }

private:
    Point currentPos{0, 0, 0};
    Point nextPos;
};

static RobotControler robot;
static int moveCounter = 0;
std::string dir = "./Data/";

static void X1CollectNotify(RVC::X1::CollectionCallBackInfo info, RVC::X1::CaptureOptions opts, RVC::UserPtr ctx) {
    RobotControler* robot_ptr = reinterpret_cast<RobotControler*>(ctx);

    // Move
    robot_ptr->move();
    info.image.SaveImage((dir + "/image_" + std::to_string(moveCounter) + ".png").c_str());
}

static void X1CalculateNotify(RVC::X1::CalculationCallBackInfo info, RVC::X1::CaptureOptions opts, RVC::UserPtr ctx) {
    RobotControler* robot_ptr = reinterpret_cast<RobotControler*>(ctx);

    std::cout << "Calculation Finished!" << std::endl;
}

int main(int argc, char* argv[]) {
    // Initialize RVC system.
    RVC::SystemInit();

    // Find Device
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
    RVC::Device device = devices[0];

    RVC::DeviceInfo info;
    device.GetDeviceInfo(&info);

    // Create and Open
    RVC::X1 x1 = RVC::X1::Create(device);
    if (!x1.Open()) {
        std::cout << "Failed to open camera! Please check whether the camera is connected and make sure it is not occupied and supports X1." << std::endl;
        RVC::X1::Destroy(x1);
        RVC::SystemShutdown();
        return -1;
    }

    // Set CallBack
    if (!x1.SetCollectionCallBack(X1CollectNotify, &robot) || !x1.SetCalculationCallBack(X1CalculateNotify, &robot)) {
        std::cout << "Failed to set X1 CallBack!" << std::endl;
        RVC::X1::Destroy(x1);
        RVC::SystemShutdown();
        return -1;
    }

    // Data Process Dir
    MakeDirectories(dir);
    dir += info.sn;
    MakeDirectories(dir);
    dir += "/x1";
    MakeDirectories(dir);

    // Set Position vector
    std::vector<Point> points;
    points.emplace_back(1.1, 2.3, 5.6);
    points.emplace_back(1.8, 6.3, 0.6);
    points.emplace_back(1.9, 3.7, 3.6);
    points.emplace_back(2.22, 2.3, 5.6);
    points.emplace_back(0.0, 0.0, 0.0);

    // Start to capture
    for (int i = 0; i < points.size(); ++i) {
        robot.setNextPos(points[i]);
        bool ret = x1.Capture();

        if (ret == false) {
            std::cout << RVC::GetLastErrorMessage() << std::endl;
            x1.Close();
            RVC::X1::Destroy(x1);
            RVC::SystemShutdown();
            return -1;
        }

        RVC::PointMap pm = x1.GetPointMap();
        RVC::Image img = x1.GetImage();
        pm.Save((dir + "/pointcloud_" + std::to_string(i) + ".ply").c_str(), RVC::PointMapUnit::Millimeter, true);
        pm.Save((dir + "/pointcloud_color_" + std::to_string(i) + ".ply").c_str(), RVC::PointMapUnit::Millimeter, true,
                img);
    }

    // Close RVC Camera.
    x1.Close();
    RVC::X1::Destroy(x1);
    RVC::SystemShutdown();

    return 0;
}
