// Copyright (c) RVBUST, Inc - All rights reserved.

#include <RVC/RVC.h>
#include <iostream>

using namespace RVC;

int main(int argc, char* argv[]) {

    // Initialize RVC system.
    RVC::SystemInit();

	// Find Device
    RVC::Device device;

	// Method 1,Find device by index.
    RVC::Device devices[20];
    size_t actual_size = 0;
    SystemListDevices(devices, 20, &actual_size, RVC::SystemListDeviceType::All);
    if (actual_size == 0) {
        std::cout << "Can not find any Camera!" << std::endl;
        return -1;
    }
    RVC::DeviceInfo info;
    for(int i = 0; i < actual_size; i++){
        devices[i].GetDeviceInfo(&info);
        std::cout << "Device " << i << " : " << std::endl;
        std::cout << "        SN : " << info.sn << std::endl;
        std::cout << "        Model : " << info.name << std::endl;
    }
    std::cout << "Please select a device : ";

    int number;
    std::cin >> number;
    if(number < 0 || number >= actual_size) {
        std::cout << "Can not find any Camera!" << std::endl;
        return -1;
    }


    bool ret = devices[number].UpgradeFirmware("./rvc_fw_v1.5.0_Laser_20250529.rvbin", "./m2600.data");
    if(ret){
        std::cout << "Upgrade Firmware OK!!" << std::endl;
    }else{
        std::cout << "Upgrade Firmware error!!" << std::endl;
    }

    return 0;
}
