#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

struct PathSample
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Transform transform;
};

class ReplayPathNode : public rclcpp::Node
{
public:
    explicit ReplayPathNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("replay_path_node", options)
    {
        m_model = declare_parameter<std::string>("model", "zu5");
        m_planning_group = "jaka_" + m_model;
        m_input_file = declare_parameter<std::string>("input_file", "config/paths/demo_path.yaml");
    }

    bool initializeMoveIt()
    {
        // 创建 MoveGroupInterface 对象时，内部会使用传入的node创建或者链接MoveIt的Action客户端，TF监听，规划和执行结果回调等
        // shared_from_this() 把共享指针交给interface。共享指针指共享了对象所有权以及对象的生命周期，例如node和another都指向ReplayPathNode,
        // 只有node和another都被销毁后，ReplayPathNode才会被销毁
        m_move_group = std::make_unique<
            moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(),
                m_planning_group
            );
        const auto current_state = m_move_group->getCurrentState(10.0);
        if (!current_state) {
            RCLCPP_ERROR(get_logger(), "无法获取当前机器人关节状态，请检查 /joint_states 和 use_sim_time");
            return false;
        }

        m_joint_model_group = current_state->getJointModelGroup(m_planning_group);
        if (m_joint_model_group == nullptr) {
            RCLCPP_ERROR(get_logger(), "找不到规划组: %s", m_planning_group.c_str());
            return false;
        }

        return true;
    }

    bool loadSamples()
    {
        YAML::Node root;
        try {
            std::ifstream input(m_input_file);
            if (!input.is_open()) {
                RCLCPP_ERROR(get_logger(), "failed to open input file: %s", m_input_file.c_str());
                return false;
            }
            root = YAML::Load(input);
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(get_logger(), "failed to load %s: %s", m_input_file.c_str(), ex.what());
            return false;
        }

        if (!root["samples"]) {
            RCLCPP_ERROR(get_logger(), "no samples found in %s", m_input_file.c_str());
            return false;
        }

        for (const auto & item : root["samples"]) {
            PathSample sample;
            const double stamp_sec = item["stamp_sec"].as<double>();
            const auto stamp_nanoseconds = static_cast<int64_t>(stamp_sec * 1e9);
            sample.stamp = rclcpp::Time(stamp_nanoseconds, get_clock()->get_clock_type());

            const auto translation = item["translation"].as<std::vector<double>>();
            if (translation.size() != 3) {
                RCLCPP_ERROR(get_logger(), "translation must have 3 values");
                return false;
            }

            const auto rotation = item["rotation_xyzw"].as<std::vector<double>>();
            if (rotation.size() != 4) {
                RCLCPP_ERROR(get_logger(), "rotation_xyzw must have 4 values");
                return false;
            }

            sample.transform.translation.x = translation[0];
            sample.transform.translation.y = translation[1];
            sample.transform.translation.z = translation[2];

            sample.transform.rotation.x = rotation[0];
            sample.transform.rotation.y = rotation[1];
            sample.transform.rotation.z = rotation[2];
            sample.transform.rotation.w = rotation[3];
            
            m_samples.push_back(sample);
        }

        RCLCPP_INFO(get_logger(), "loaded %zu samples from %s", m_samples.size(), m_input_file.c_str());
        return true;
    }
    void replayPath()
    {
        if (m_samples.empty()) {
            RCLCPP_ERROR(this->get_logger(), "采样点为空");
            return;
        }
        // 首先移动到轨迹开始点
        geometry_msgs::msg::Pose pose;
        const auto & first_sample = m_samples[0];
        pose.position.x = first_sample.transform.translation.x;
        pose.position.y = first_sample.transform.translation.y;
        pose.position.z = first_sample.transform.translation.z;
        pose.orientation = first_sample.transform.rotation;
        m_move_group->setPoseTarget(pose);
        const auto move_result = m_move_group->move();
        if (move_result != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "运动到初始采样点失败");
            return;
        }

        std::vector<geometry_msgs::msg::Pose> waypoints;
        for (const auto & sample : m_samples) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = sample.transform.translation.x;
            pose.position.y = sample.transform.translation.y;
            pose.position.z = sample.transform.translation.z;
            pose.orientation = sample.transform.rotation;
            waypoints.push_back(pose);
        }
        moveit_msgs::msg::RobotTrajectory trajectory; // trajectory 用于存储关节轨迹
        // moveit 对每个waypoint之间插值，对每个插值点计算逆运动学，最终将关节空间轨迹存储在trajectory中
        // fraction表示成功规划的点数占总点数的比例
        double fraction = m_move_group->computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
        if (fraction < 1.0) {
            RCLCPP_ERROR(this->get_logger(), "路径规划失败，只有%.1f%%的点成功规划", fraction * 100.0);
            return;
        }

        // 时间参数化需要使用moveit轨迹对象，创建当前规划组的轨迹并且使用trajectory初始化
        robot_trajectory::RobotTrajectory robot_trajectory(
            m_move_group->getRobotModel(),
            m_planning_group);
        robot_trajectory.setRobotTrajectoryMsg(
            *m_move_group->getCurrentState(),
            trajectory);
        
        // 使用time parameterization对象计算时间参数化
        trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
        const bool time_success = time_parameterization.computeTimeStamps(
            robot_trajectory,
            0.05,
            0.05);
        if (!time_success) {
            RCLCPP_ERROR(this->get_logger(), "轨迹时间参数化失败");
            return;
        }

        // 重新根据轨迹对象转回轨迹消息
        robot_trajectory.getRobotTrajectoryMsg(trajectory);

        // 定义plan并且准备执行
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        // move用于设定好target,execute用于执行规划好的plan
        const auto execute_result = m_move_group->execute(plan);
        if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "轨迹执行失败");
        }
    }

private:
    std::string m_model;
    std::string m_planning_group;
    std::string m_input_file;

    std::vector<PathSample> m_samples;
    std::unique_ptr<
        moveit::planning_interface::MoveGroupInterface> m_move_group;
    const moveit::core::JointModelGroup* m_joint_model_group;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    auto node = std::make_shared<ReplayPathNode>(options);

    // ROS2会在Node中使用CallbackGroup保存ROS对象（subscriptions,timers等）的回调
    // executor从Node的CallbackGroup中收集这些实体，等待其变为ready并执行它们的回调。
    // spin()内部会等待ROS事件，找到已经就绪的ROS实体，执行其回调，随后继续等待
    // rclcpp::spin(node) 为使用executor的简化写法，但是会阻塞，因此无法调用后续的函数
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    if (!node->initializeMoveIt()) {
        executor.cancel();
        if (spin_thread.joinable()) {
            spin_thread.join();
        }
        rclcpp::shutdown();
        return 1;
    }

    if (!node->loadSamples()) {
        executor.cancel();
        if (spin_thread.joinable()) {
            spin_thread.join();
        }
        rclcpp::shutdown();
        return 1;
    }

    node->replayPath();

    executor.cancel();

    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}
