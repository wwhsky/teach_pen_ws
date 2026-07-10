相机采集模式：
enum CaptureMode {
    CaptureMode_Fast = 1 << 0,
    CaptureMode_Normal = 1 << 1,
    CaptureMode_Ultra = 1 << 2,
    CaptureMode_Robust = 1 << 3,  // [deprecated] CaptureMode_Robust is deprecated! Use scan_times instead.
    CaptureMode_AntiInterReflection = 1 << 4,
    CaptureMode_SwingLineScan = 1 << 5,
    CaptureMode_FixedLineScan = 1 << 6,
    CaptureMode_LineArrayShift = 1 << 7,
};

注意：
安装SDK后若需要使用PCL库，而PCL库依赖高版本的libusb，但是RVC自带低版本的libusb，会导致冲突，需要在运行前先手动运行：
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0

ruben_sdk_server_node：相机SDK封装的ROS节点，可直接获取彩图和点云，客户端获取的时候同步进行数据的显示。

ruben_sdk_client_test：相机ROS服务节点的客户端测试，可直接搬运使用。

暂时不可以直接使用固定线扫功能。
