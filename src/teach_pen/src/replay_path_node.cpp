#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/moveit_msgs/msg/display_robot_state.hpp"  
#include "moveit_msgs/moveit_msgs/msg/display_trajectory.hpp"   
#include "moveit_msgs/moveit_msgs/msg/attached_collision_object.hpp"  
#include "moveit_msgs/moveit_msgs/msg/collision_object.hpp"  
#include <moveit_msgs/moveit_msgs/msg/joint_limits.hpp>
#include <moveit/robot_state/robot_state.h>
#include "std_srvs/srv/empty.hpp"
#include <yaml-cpp/yaml.h>


struct PathSample
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Transform transform;
};

class ReplayPathNode : public rclcpp::Node
{
public:
    ReplayPathNode()
    : Node("replay_path_node")
    {
        m_model = declare_parameter<std::string>("model", "zu5");
        m_planning_group = "jaka_" + m_model;
        m_input_file = declare_parameter<std::string>("input_file", "config/paths/demo_path.yaml");
    }

    void initializeMoveIt()
    {
        // 创建 MoveGroupInterface 对象时，内部会使用传入的node创建或者链接MoveIt的Action客户端，TF监听，规划和执行结果回调等
        // shared_from_this() 把共享指针交给interface。共享指针指共享了对象所有权以及对象的生命周期，例如node和another都指向ReplayPathNode,
        // 只有node和another都被销毁后，ReplayPathNode才会被销毁
        m_move_group = std::make_unique<
            moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(),
                m_planning_group
            );
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        m_joint_model_group = m_move_group->getCurrentState()->getJointModelGroup(m_planning_group);
    }

    void loadSamples()
    {
        YAML::Node root;
        try {
            std::ifstream input(m_input_file);
            if (!input.is_open()) {
                RCLCPP_ERROR(get_logger(), "failed to open input file: %s", m_input_file.c_str());
                return;
            }
            root = YAML::Load(input);
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(get_logger(), "failed to load %s: %s", m_input_file.c_str(), ex.what());
            return;
        }

        if (!root["samples"]) {
            RCLCPP_ERROR(get_logger(), "no samples found in %s", m_input_file.c_str());
            return;
        }

        for (const auto & item : root["samples"]) {
            PathSample sample;
            sample.stamp = rclcpp::Time(
                item["stamp_sec"].as<int32_t>(),
                0,
                get_clock()->get_clock_type());

            const auto translation = item["translation"].as<std::vector<double>>();
            const auto rotation = item["rotation_xyzw"].as<std::vector<double>>();

            sample.transform.translation.x = translation[0];
            sample.transform.translation.y = translation[1];
            sample.transform.translation.z = translation[2];

            sample.transform.rotation.x = rotation[0];
            sample.transform.rotation.y = rotation[1];
            sample.transform.rotation.z = rotation[2];
            sample.transform.rotation.w = rotation[3];
            
            m_samples.push_back(sample);
        }
    }
    void replayPath()
    {
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
            RCLCPP_ERROR(this->get_logger(), "路径规划失败，只有%.1f%的点成功规划", fraction);
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        // move用于设定好target,execute用于执行规划好的plan
        const auto result = m_move_group->execute(plan);
        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
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
    auto node = std::make_shared<ReplayPathNode>();

    // ROS2会在Node中使用CallbackGroup保存ROS对象（subscriptions,timers等）的回调
    // executor从Node的CallbackGroup中收集这些实体，等待其变为ready并执行它们的回调。
    // spin()内部会等待ROS事件，找到已经就绪的ROS实体，执行其回调，随后继续等待
    // rclcpp::spin(node) 为使用executor的简化写法，但是会阻塞，因此无法调用后续的函数
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    node->initializeMoveIt();
    node->loadSamples();
    node->replayPath();

    executor.cancel();

    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}