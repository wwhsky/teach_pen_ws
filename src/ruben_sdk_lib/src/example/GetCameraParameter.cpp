#include <RVC/RVC.h>
#include <iostream>

int main(int argc, char *argv[]) {
    // Initialize RVC system.
    RVC::SystemInit();

    // Scan all RVC Camera devices.
    RVC::Device devices[10];
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::All);

    // Find whether any RVC Camera is connected or not.
    if (actual_size == 0) {
        std::cout << "Can not find any RVC Camera!" << std::endl;
        return -1;
    }
    if(devices[0].IsFirmwareMatch() == false){
        std::cout << "device firmware mismatch, Please use RVCManager to upgrade the firmware" << std::endl;
        RVC::SystemShutdown();
        return -1;
    }
    // Create a RVC Camera and choose use left side camera.
    RVC::X1 x1 = RVC::X1::Create(devices[0], RVC::CameraID_Left);
    x1.Open();

    float T_extrinsic[16];
    float params_intrinsic[9];
    float distortion[5];
    bool ret = x1.GetExtrinsicMatrix(T_extrinsic);
    ret &= x1.GetIntrinsicParameters(params_intrinsic, distortion);
    if (ret) {
        std::cout << "camera Extrinsic Matrix: \n";
        for (int i = 0; i < 16; i++) {
            std::cout << T_extrinsic[i] << ", ";
        }
        std::cout << "\ncamera intrinsic parameter: \n";
        for (int i = 0; i < 9; i++) {
            std::cout << params_intrinsic[i] << ", ";
        }
        std::cout << "\ncamera distortion: \n";
        for (int i = 0; i < 5; i++) {
            std::cout << distortion[i] << ", ";
        }
        std::cout << "\n\n";
    } else {
        std::cout << RVC::GetLastErrorMessage() << std::endl;
        std::cout << "RVC camera is not valid !!!\n";
    }

    // Close RVC Camera.
    x1.Close();

    // Destroy RVC Camera.
    RVC::X1::Destroy(x1);

    // Shutdown RVC System.
    RVC::SystemShutdown();

    return 0;
}