# Teach Pen Workspace

这个工作空间目前主要包含四部分：

- `vive_tracker_ros2`：读取 Vive Tracker 位姿和按键，并发布 ROS2 topic / TF。项目主线位姿使用 SteamVR/OpenVR；`libsurvive` 保留用于按键读取、实验和对比。
- `calibration`：用于标定 `steamvr_base`、`tracker_frame`、`teaching_pen_tip`、`welding_torch_tip`、`workpiece_frame` 等坐标关系。
- `teach_pen`：项目自己的业务包，包含示教路径采集、路径回放、焊缝感知和 JAKA 轨迹执行节点。
- `jaka_ros2`：JAKA 机械臂的描述、MoveIt 配置、仿真和实机接口。

## Vive Tracker 追踪

`vive_tracker_ros2` 现在有两条追踪路线：

```text
位姿主线：Tracker -> USB Dongle -> SteamVR/OpenVR -> ROS2 Pose/TF
辅助路线：Tracker -> USB Dongle -> libsurvive -> ROS2 Pose/Joy/TF
```

实际测试下来，SteamVR/OpenVR 的位姿解算效果比 `libsurvive` 更稳定，因此
示教和标定主流程优先使用 SteamVR/OpenVR。`libsurvive` 的价值主要在于它能
直接从 Watchman/Dongle 数据里读到 Tracker 按键，也能用于查看基站可见性和
做算法对比。

注意：当前 `vive_tracker.launch.py` 启动的是 `libsurvive` 节点；如果要走
主线 SteamVR/OpenVR 位姿，需要直接运行 `vive_tracker_node`，见下面
“SteamVR / OpenVR 主线”小节。

### 两种方案对比

项目里保留 SteamVR/OpenVR 和 `libsurvive` 两套方案，是因为它们各自解决的
问题不完全一样。

SteamVR/OpenVR 方案依赖 SteamVR 运行时，使用前必须安装并打开 SteamVR。
在没有头显的情况下，还需要配置 null driver，让 SteamVR 允许只用 Tracker
和 Lighthouse 基站运行。它的缺点是系统更重、依赖闭源运行时，并且 Tracker
按键在无头显/无控制器绑定场景下不一定容易读出来。

但从目前实测看，SteamVR 的位姿解算和融合效果更好。Tracker 静止时更稳，
移动后停下来的收敛也更自然，因此当前示教笔采点、VR 到机器人标定、轨迹复现
这条主流程优先使用 SteamVR/OpenVR 输出的位姿。

`libsurvive` 是开源项目，可以绕过 SteamVR 直接访问 Dongle 和 Lighthouse
数据。它的优点是依赖更轻，不需要下载或运行 SteamVR/OpenVR；同时能读到更多
底层数据，例如 Tracker 按键、基站可见性、光学观测数量等。这些数据对调试
示教笔按键、观察遮挡、对比不同追踪算法很有价值。

`libsurvive` 的问题是当前实测位姿不如 SteamVR 稳：静止时可能出现漂移，移动
一段后再停下来可能出现过冲或回弹，推测与它的光学/IMU 融合算法、基站协同
或滤波参数有关。后续可以继续调 `use_raw_observation`、`use_kalman`、
`poser`、`pose_filter_alpha` 等参数，但在没有验证到更稳之前，不把它作为
主位姿来源。

因此当前取舍是：

```text
位姿：优先 SteamVR/OpenVR
按键/原始观测/可见性调试：优先 libsurvive
算法对比：两者都保留
```

### libsurvive 辅助路线

只有在需要 `libsurvive` 按键、可见性或对比测试时，才需要构建工作空间里的
`third_party/libsurvive/build-local`：

```bash
sudo apt install build-essential cmake libusb-1.0-0-dev libjson-c-dev libeigen3-dev

cd ~/code/teach_pen_ws
git clone https://github.com/cntools/libsurvive.git third_party/libsurvive
touch third_party/libsurvive/COLCON_IGNORE
cmake -S third_party/libsurvive \
  -B third_party/libsurvive/build-local \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSE_OPENBLAS=OFF \
  -DUSE_OPENCV=OFF \
  -DBUILD_GATT_SUPPORT=OFF
cmake --build third_party/libsurvive/build-local --parallel

colcon build --packages-select vive_tracker_ros2
source install/setup.bash
```

如果普通用户无法访问 USB 设备，需要安装 udev 规则：

```bash
sudo cp third_party/libsurvive/useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Lighthouse 基站配置

本项目使用两个 Lighthouse 基站时，需要让 Vive Tracker 同时处在两个基站的
视野内。Tracker 虽然在单基站下也可能得到位姿，但遮挡、姿态约束和稳定性会
明显变差；示教笔采点时应尽量保证 Tracker 的传感器窗口能被两个基站同时看到。

对于 Lighthouse 1.0 基站，双基站模式建议设置为：

```text
一个基站：B
另一个基站：C
```

这种 B/C 模式依赖两个基站之间能够互相看到对方，用于完成光学同步。如果两个
基站之间存在遮挡、无法互相可见，可以使用一根 3.5mm 音频线作为同步线连接
两个基站，此时通常将基站设置为 A/B 模式进行有线同步。

本项目约定使用 Lighthouse 原始追踪空间作为 `steamvr_base`。实际安装时，
基站的位置和朝向不需要与机器人坐标系一致，后续会通过标定得到：

```text
robot_base -> steamvr_base
```

因此基站的作用是提供稳定追踪，机器人坐标系下的真实含义由本项目标定结果决定。

### 启动 libsurvive 节点

第一次使用或移动基站后，可以强制重新标定 Lighthouse 相对关系：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 launch vive_tracker_ros2 vive_tracker.launch.py force_calibrate:=true
```

保持 Tracker 静止，并让它同时被两个基站看到。日志出现 `MPFIT success` 后，
后续正常启动可以不再加 `force_calibrate`：

```bash
ros2 launch vive_tracker_ros2 vive_tracker.launch.py
```

默认会发布：

```text
/vive_tracker/pose
/vive_tracker/<serial>/pose
/vive_tracker/buttons
/vive_tracker/<serial>/buttons
/vive_tracker/visibility
/tf: steamvr_base -> tracker_frame
```

`/vive_tracker/buttons` 是 `sensor_msgs/msg/Joy`：

```text
buttons[0] = trigger
buttons[1] = grip
buttons[2] = thumb / trackpad
buttons[3] = menu

axes[0] = trackpad x
axes[1] = trackpad y
axes[2] = trigger raw
```

`/vive_tracker/visibility` 用于观察基站可见性：

```text
[visible_lighthouses, total_measurements, lh0_x, lh0_y, lh1_x, lh1_y]
```

常用检查命令：

```bash
ros2 topic echo /vive_tracker/pose
ros2 topic echo /vive_tracker/buttons
ros2 topic echo /vive_tracker/visibility
ros2 run tf2_ros tf2_echo steamvr_base tracker_frame
```

如果想减少 IMU 融合影响、只看光学解算结果，可以尝试：

```bash
ros2 launch vive_tracker_ros2 vive_tracker.launch.py use_raw_observation:=true
```

这种模式更接近纯 Lighthouse 光学观测，更新会更不连续；没有有效光学解时会停。

### SteamVR / OpenVR 主线

SteamVR/OpenVR 是当前项目推荐的 Vive Tracker 位姿来源。它的按键读取在无头显
场景下不够可靠，但位姿解算更稳定，适合作为示教笔采点和标定的主输入。

默认情况下 SteamVR 需要检测到 HMD 头显才能正常进入追踪状态。本项目只使用
Vive Tracker 和 Lighthouse 基站，不使用头显，因此需要修改 SteamVR 配置，
让 SteamVR 允许无头显运行。

配置文件位置：

```text
~/.local/share/Steam/config/steamvr.vrsettings
```

在该文件中加入或修改 `steamvr` 配置段：

```json
{
  "steamvr": {
    "requireHmd": false,
    "forcedDriver": "null",
    "activateMultipleDrivers": true,
    "enableHomeApp": false,
    "showMirrorView": false
  }
}
```

如果文件中已经有其他配置项，不要直接覆盖整个文件，而是把上面的键合并到
已有的 `"steamvr"` 对象中，保证最终文件仍然是合法 JSON。

常用字段含义：

```text
requireHmd: false
    不强制要求头显。

forcedDriver: "null"
    使用 SteamVR 的 null HMD 驱动模拟头显。

activateMultipleDrivers: true
    允许 null driver 和 Lighthouse / Tracker 相关驱动同时工作。

enableHomeApp: false
    不启动 SteamVR Home，减少额外负载。

showMirrorView: false
    不显示 VR 镜像窗口，避免无头显场景下弹出无用画面。
```

配置完成后重启 SteamVR。随后将 Vive Tracker 的无线接收器插入电脑，
长按 Tracker 的配对键进入配对状态。配对成功后，SteamVR 小窗口中应能看到
Tracker 图标；基站可见且追踪正常时，OpenVR 节点才能读到有效位姿。

### tracking universe

OpenVR 读取位姿时需要指定 tracking universe，也就是 SteamVR 输出位姿所使用
的坐标空间。本项目节点支持通过 `tracking_universe` 参数选择。

常见模式：

```text
raw
    原始追踪空间。未经过 SteamVR Room Setup 的坐标变换，也不依赖房间原点、
    地面高度或站立中心。本项目使用该模式，然后用自己的标定流程将
    steamvr_base 对齐到 robot_base。

standing / sitstand
    经过 SteamVR Room Setup 后的用户空间。通常包含地面、站立中心、房间方向
    等人为标定结果，适合 VR 应用，但对本项目来说会引入额外的 SteamVR 房间
    标定依赖。

seated
    以坐姿/重置后的 seated zero pose 为参考的空间，主要用于坐姿 VR 场景。
```

本项目推荐使用：

```text
tracking_universe:=raw
```

也就是使用未经过 SteamVR 房间标定的原始数据。这样 SteamVR 只负责 Lighthouse
追踪，机器人坐标系关系由本项目自己的标定模块维护。

OpenVR 节点直接运行示例：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 run vive_tracker_ros2 vive_tracker_node --ros-args \
  -p tracking_universe:=raw
```

如果日志中出现：

```text
pose_valid=false
tracking_result=Calibrating_OutOfRange
```

通常表示 Tracker 已连接，但当前没有被基站稳定追踪。需要检查基站供电、
基站模式、Tracker 是否在基站视野内，以及 SteamVR 中设备图标是否正常。

## 标定模块

当前标定包是 `calibration`，主要维护这些 TF：

```text
robot_base
├── robot_flange
│   └── welding_torch_tip
├── steamvr_base
│   └── tracker_frame
│       └── teaching_pen_tip
└── workpiece_frame
```

运行时发布静态标定 TF：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 run calibration publish_calibration_tf --ros-args \
  -p files:="[config/calibration/teaching_pen_tip.yaml,config/calibration/welding_torch_tip.yaml,config/calibration/vr_to_robot.yaml,config/calibration/workpiece.yaml]"
```

`publish_calibration_tf` 支持两种 YAML 格式：字段直接在顶层，或者在
`calibration_result` 下。当前默认发布用文件是：

```text
config/calibration/teaching_pen_tip.yaml
config/calibration/welding_torch_tip.yaml
config/calibration/vr_to_robot.yaml
config/calibration/workpiece.yaml
```

### 联合标定

当前主线推荐使用联合标定：

```bash
ros2 run calibration calibrate_tool_vr_joint --ros-args \
  -p output_file:=config/calibration/tool_vr_joint_calibration.yaml
```

它同时估计：

```text
robot_flange -> welding_torch_tip
robot_base   -> steamvr_base
```

这个脚本的输出 `tool_vr_joint_calibration.yaml` 是完整记录文件，里面包含原始
样本和两个嵌套结果。它不是 `publish_calibration_tf` 默认直接读取的文件。
标定完成后，需要把其中结果拆到发布用 YAML：

```text
tool_transform       -> config/calibration/welding_torch_tip.yaml
vr_to_robot_transform -> config/calibration/vr_to_robot.yaml
```

如果已经有稳定结果，后续日常运行只需要加载发布用 YAML；只有更换 Tracker
安装、移动基站、调整工具或重新定义机器人基座时才需要重新标定。

### 其他标定工具

```text
calibrate_tool
    传统 pivot 工具标定。默认求 robot_flange -> welding_torch_tip。

calibrate_vr_to_robot
    旧的 VR 到机器人标定流程。适合两个 tip 已知时，用成对点求
    robot_base -> steamvr_base。

calibrate_workpiece
    工件三点法标定。采集原点、+X 点、XY 平面点，输出
    robot_base -> workpiece_frame。
```

## teach_pen 与 MoveIt 流程

这一部分是项目从“示教笔采点”到“机械臂复现轨迹”的主流程。核心思想是：

```text
示教笔采集 teaching_pen_tip 路径
-> 标定得到 robot_flange -> welding_torch_tip
-> replay_path_node 将 tip 路径换算成 Link6 / robot_flange 目标
-> MoveIt 做 IK、碰撞检查、路径规划和时间参数化
-> MoveIt 将 joint trajectory 发给当前执行端
-> 执行端决定 RViz fake、Gazebo 还是真机运动
```

MoveIt 本身主要负责规划，不直接决定“后面是真机械臂还是仿真机械臂”。在
JAKA ZU5 配置里，MoveIt 最终会把轨迹发到：

```text
/jaka_zu5_controller/follow_joint_trajectory
```

这个接口是 `control_msgs/action/FollowJointTrajectory`。谁提供这个 action
server，谁就是当前执行端。

### 运行模式

| 模式 | 启动方式 | 执行端 | 用途 |
|---|---|---|---|
| RViz fake | `ros2 launch jaka_zu5_moveit_config demo.launch.py use_rviz_sim:=true` | fake ros2_control | 快速检查 MoveIt 规划和 RViz 显示 |
| Gazebo | `ros2 launch teach_pen gazebo.launch.py model:=zu5` | Gazebo ros2_control | 检查物理仿真、工件和环境关系 |
| 实机主线 | `ros2 launch teach_pen real_robot.launch.py ip:=<机械臂IP> model:=zu5` | `jaka_trajectory_executor_node` | 项目自己的 JAKA SDK 执行链路 |
| JAKA demo | `ros2 launch jaka_planner moveit_server.launch.py ip:=<机械臂IP> model:=zu5` | 官方 `moveit_server` | 对照官方 demo，不作为主线 |

项目自己的 Gazebo 入口是：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 launch teach_pen gazebo.launch.py model:=zu5
```

它会启动 `publish_calibration_tf`、JAKA MoveIt 配置里的 Gazebo 子 launch 和
RViz。Gazebo 子 launch 内部负责启动 Ignition Gazebo、`robot_state_publisher`、
`move_group`、生成机器人模型，并启动 `jaka_zu5_controller` 和
`joint_state_broadcaster`。不要再额外启动一份 `move_group.launch.py`，否则会出现
两个同名 `/move_group`，执行轨迹时容易互相干扰。

`moveit_controllers.yaml` 中配置的是 MoveIt 要找哪个 action：

```text
src/jaka_ros2/src/jaka_zu5_moveit_config/config/moveit_controllers.yaml
```

当前 `moveit_manage_controllers=false`，所以 MoveIt 不负责帮你启动 controller。
运行前必须已经有一个执行端提供 `/jaka_zu5_controller/follow_joint_trajectory`。
实机模式下不要同时启动 `use_rviz_sim:=true`，否则 fake controller 可能和真机
executor 抢 `/joint_states` 或 trajectory action。

### teach_pen 节点分工

| 节点 | 输入 | 输出 | 作用 |
|---|---|---|---|
| `collect_path_node` | TF: `robot_base <- teaching_pen_tip` | `config/paths/*.yaml` | 用示教笔采集 tip 路径 |
| `replay_path_node` | 路径 YAML、`welding_torch_tip.yaml`、MoveIt | MoveIt 规划/执行请求 | 将 tip 路径转换为法兰目标并复现 |
| `jaka_trajectory_executor_node` | `FollowJointTrajectory` action | JAKA SDK `servo_j`、`/joint_states` | 项目自己的实机执行端 |
| `seam_perception_node` | ROI 点云或点云文件 | `/seam_tracking/measured_path` | 从点云中提取焊缝线 |

### 路径采集

采集节点查询 TF 中 `base_frame <- tip_frame` 的位姿并保存成 YAML。默认采的是
示教笔笔尖：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 run teach_pen collect_path_node --ros-args \
  -p base_frame:=robot_base \
  -p tip_frame:=teaching_pen_tip \
  -p output_file:=config/paths/demo_path.yaml
```

键盘控制：

```text
Space  采样一个点
r      开关连续采样
u      撤销最后一个点
s      保存 YAML
q      退出节点
```

路径 YAML 的基本格式：

```yaml
base_frame: robot_base
tip_frame: teaching_pen_tip
sample_count: 3

samples:
  - index: 0
    stamp_sec: 0.0
    translation: [0.35, 0.00, 0.30]
    rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

`translation` 单位为米，`rotation_xyzw` 为四元数，顺序是 `x, y, z, w`。
注意：这里保存的是 tip 位姿，不是 MoveIt 直接执行的 `Link6` 位姿。

### 路径回放

`replay_path_node` 读取路径 YAML 后，会加载工具标定：

```text
robot_flange -> welding_torch_tip
```

然后把每个 `base -> tip` 样本换算成 `base -> Link6/flange` 目标。这样 MoveIt
规划的是机械臂真实末端 link，焊枪尖会尽量复现示教笔尖采集到的路径。

运行示例：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 run teach_pen replay_path_node --ros-args \
  -p model:=zu5 \
  -p input_file:=config/paths/demo_path.yaml \
  -p tool_calibration_file:=config/calibration/welding_torch_tip.yaml \
  -p execute:=true \
  -p path_mode:=cartesian
```

`use_sim_time` 默认为 `false`，实机运行时不需要设置。若在 Gazebo 中单独运行
`replay_path_node`，需要让它使用仿真时间，否则会因为 `/joint_states` 的时间戳
来自 `/clock` 而无法获取当前状态：

```bash
ros2 run teach_pen replay_path_node --ros-args \
  -p use_sim_time:=true
```

当前回放流程：

```text
初始化 MoveGroupInterface
-> 读取 samples
-> 读取 tool_calibration_file
-> 添加 table collision
-> 规划并执行 home start
-> 用 seeded IK 移动到第一个采样点
-> cartesian 模式：computeCartesianPath + 时间参数化
   joint 模式：逐点 IK + 普通关节规划
-> 发布 /display_planned_path 供 RViz 预览
-> execute 发送给当前 FollowJointTrajectory action server
-> 成功后 home end，失败后尝试 home after failure
```

常用参数：

```text
model                    default: zu5
pose_reference_frame     default: robot_base
end_effector_link        default: Link6
tool_calibration_file    default: config/calibration/welding_torch_tip.yaml
path_mode                cartesian / joint
cartesian_eef_step       default: 0.001
cartesian_min_fraction   default: 1.0
velocity_scaling         default: 0.05
acceleration_scaling     default: 0.05
execute                  default: true，false 时只规划和预览
use_sim_time             default: false，Gazebo 单独回放时设为 true
enable_table_collision   default: true
table_z                  default: -0.015
```

`velocity_scaling` 和 `acceleration_scaling` 会传给 MoveIt 的轨迹时间参数化，
含义是按关节速度/加速度限制的比例执行。比如：

```bash
ros2 run teach_pen replay_path_node --ros-args \
  -p velocity_scaling:=0.10 \
  -p acceleration_scaling:=0.10
```

如果规划失败，节点会打印 IK 失败位置、关节越界、碰撞对等诊断信息。常见原因是
路径超出工作空间、工具标定不准、焊枪姿态不可达，或者工具/桌面碰撞。

### 实机执行端

项目自己的实机链路由 `jaka_trajectory_executor_node` 提供。它做三件事：

```text
连接 JAKA SDK
发布 /joint_states
提供 /jaka_zu5_controller/follow_joint_trajectory
```

推荐用一条 launch 启动实机环境：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 launch teach_pen real_robot.launch.py ip:=<机械臂IP> model:=zu5
```

这个 launch 默认会同时启动：

```text
jaka_trajectory_executor_node
publish_calibration_tf
static_virtual_joint_tfs
robot_state_publisher
move_group
RViz
```

常用参数：

```text
start_executor        default: true
start_calibration_tf  default: true
start_static_tf       default: true
start_rsp             default: true
start_move_group      default: true
start_rviz            default: true
auto_power_on         default: true
auto_enable           default: true
```

如果只想单独启动执行端：

```bash
ros2 run teach_pen jaka_trajectory_executor_node --ros-args \
  -p ip:=<机械臂IP> \
  -p model:=zu5 \
  -p auto_power_on:=true \
  -p auto_enable:=true
```

执行端会把 MoveIt 发来的轨迹按 `servo_period` 重采样，再按
`servo_send_period` 调用 JAKA SDK `servo_j`。常用参数：

```text
servo_period           default: 0.008
servo_send_period      default: 0.004
servo_step_num         default: 1
servo_queue_control    default: true
servo_queue_low        default: 3
servo_queue_high       default: 10
reach_tolerance_deg    default: 0.2
```

检查当前接的是谁：

```bash
ros2 action list | grep follow_joint_trajectory
ros2 node list
ros2 topic echo /joint_states --once
```

大致判断：

```text
jaka_trajectory_executor_node    项目自己的实机执行端
moveit_server                    JAKA 官方 demo 执行端
controller_manager               fake ros2_control 或 Gazebo 执行端
```

### 焊缝感知

`seam_perception_node` 用于处理 ROI 点云并提取焊缝线。当前实现是两平面 RANSAC
拟合角焊缝交线，并根据交线附近真实点云支持范围裁剪起点和终点。

topic 输入示例：

```bash
ros2 run teach_pen seam_perception_node --ros-args \
  -p input_cloud_topic:=/seam_camera/roi_points
```

文件测试示例：

```bash
ros2 run teach_pen seam_perception_node --ros-args \
  -p input_file:=/path/to/cloud.ply \
  -p input_file_frame:=seam_camera_frame
```

输出：

```text
/seam_tracking/measured_path
/seam_tracking/debug_cloud
```

## ZU5 末端工具模型

本次将相机、相机支架和焊枪的组合模型接入了 JAKA ZU5 的 URDF，使
RViz 和 MoveIt 能够显示工具并进行碰撞检查。

### 模型文件

模型位于：

```text
src/jaka_ros2/src/jaka_description/meshes/jaka_zu5_meshes
```

当前实际使用：

```text
Assembly_binary.stl
AssemblySimplified_binary.stl
```

- `Assembly_binary.stl`：较完整的外观模型，用于 `<visual>`。
- `AssemblySimplified_binary.stl`：简化模型，用于 `<collision>`。

原始 CAD 导出的 STL 尺寸单位为毫米，而 URDF 使用米，因此两个 mesh
都设置了：

```xml
scale="0.001 0.001 0.001"
```

RViz 只能可靠读取二进制 STL，所以保留了原始 STL，同时生成了对应的
`*_binary.stl` 文件供 URDF 使用。

### URDF 结构

当前工具结构保持为：

```text
link0
└── robot_base

Link6
└── robot_flange
    └── tool_assembly
```

定义位置：

```text
src/jaka_ros2/src/jaka_description/urdf/jaka_zu5.urdf
```

为了保留 JAKA 原始 link 名称，同时让项目代码使用含义更清楚的坐标系，
URDF 中增加了两个零偏置语义 frame：

```text
link0 -> robot_base
Link6 -> robot_flange
```

它们分别与 JAKA 的 `link0` 和 `Link6` 完全重合。`tool_assembly` 通过
固定关节连接到 `robot_flange`：

```xml
<joint name="robot_flange_to_tool_assembly" type="fixed">
  <origin xyz="0 0 0" rpy="0 0 0" />
  <parent link="robot_flange" />
  <child link="tool_assembly" />
</joint>
```

当前组合 STL 已经包含相机、支架和焊枪，所以没有再为这些实体分别建立
可视化 link。`camera_frame` 和焊枪 TCP 属于功能坐标系，后续可以根据
手眼标定和工具标定结果单独通过 TF 发布，不要求拆分组合 STL。

`Link6` 本身不是由 URDF 节点主动测量出来的。运行时：

```text
/joint_states
-> robot_state_publisher
-> 根据 URDF 正运动学计算
-> 发布 Link1 ... Link6 以及固定工具关节的 TF
```

可以使用下面的命令检查：

```bash
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo Link5 Link6
ros2 run tf2_ros tf2_echo link0 robot_base
ros2 run tf2_ros tf2_echo Link6 robot_flange
ros2 run tf2_ros tf2_echo robot_flange tool_assembly
```

### MoveIt 碰撞配置

工具的碰撞模型会参与 MoveIt/FCL 的碰撞检查。URDF 中存在
`<collision>` 只表示“这是碰撞几何”，并不表示当前已经发生碰撞。

`robot_flange` 是没有碰撞几何的语义 frame，工具在物理上仍然安装于
`Link6` 法兰处。为了允许安装面附近的合理几何重叠，`Link6` 与
`tool_assembly` 之间的碰撞继续在 SRDF 中忽略：

```xml
<disable_collisions link1="Link6" link2="tool_assembly" reason="Adjacent"/>
```

配置位置：

```text
src/jaka_ros2/src/jaka_zu5_moveit_config/config/jaka_zu5.srdf
```

如果日志提示工具与 `Link3`、`Link4` 等非相邻连杆碰撞，不应直接加入
忽略列表。应优先检查：

1. `robot_flange_to_tool_assembly` 的 `xyz` 和 `rpy` 是否符合 CAD 装配坐标。
2. STL 原点和坐标轴是否与 `Link6` 坐标系一致。
3. STL 是否正确使用毫米到米的缩放。
4. 简化碰撞模型是否包含了过大的包络或无关几何。
5. 当前机械臂起始姿态是否确实使工具碰到了本体。

必要时可以暂时注释工具的 `<collision>`，只用于判断规划失败是否由工具
碰撞模型引起；确认原因后应恢复碰撞检查。

### 修改后的构建流程

修改 URDF、mesh 或 SRDF 后，需要重新构建对应包并重新启动 MoveIt：

```bash
cd ~/code/teach_pen_ws
colcon build --packages-select jaka_description jaka_zu5_moveit_config
source install/setup.bash
ros2 launch jaka_zu5_moveit_config demo.launch.py use_rviz_sim:=true
```

仅在 RViz 中重新加载显示，通常不会使已经运行的
`robot_state_publisher` 和 `move_group` 自动读取新模型，最稳妥的做法是
结束原 launch 后重新启动。

### 当前进展与下一步

已经完成：

- 组合工具 STL 接入 ZU5 URDF。
- 外观模型和简化碰撞模型分开使用。
- 完成毫米到米的模型缩放。
- 添加 `link0 -> robot_base` 和 `Link6 -> robot_flange` 语义坐标系。
- 添加 `robot_flange -> tool_assembly` 固定关节。
- 在 SRDF 中忽略相邻的 `Link6` 与 `tool_assembly` 碰撞。
- 在 RViz/MoveIt 中完成基本显示和规划验证。

下一步：

- 从 CAD 确认组合 STL 相对于 `robot_flange` 的准确安装变换。
- 复核或按现场重新执行焊枪 TCP / VR 联合标定，更新 `welding_torch_tip.yaml` 和 `vr_to_robot.yaml`。
- 完成相机手眼标定，发布相机坐标系 TF。
- 用多组机械臂姿态验证工具不会与本体发生错误碰撞。
- 将焊缝路径转换为焊枪 TCP 路径，并在 MoveIt 中进行笛卡尔规划。
