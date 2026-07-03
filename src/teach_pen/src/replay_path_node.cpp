#include <fstream>
#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

struct PathSample
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Transform T_base_tip;
};

class ReplayPathNode;

std::mutex g_replay_node_mutex;
std::weak_ptr<ReplayPathNode> g_replay_node;
std::atomic_bool g_sigint_seen{false};

// Transform naming convention: T_A_B means the pose of frame B in frame A.
// It also transforms a point expressed in B into the same point expressed in A.
class ReplayPathNode : public rclcpp::Node
{
public:
    explicit ReplayPathNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("replay_path_node", options)
    {
        m_model = declare_parameter<std::string>("model", "zu5");
        m_planning_group = "jaka_" + m_model;
        m_input_file = declare_parameter<std::string>("input_file", "config/paths/demo_path.yaml");
        m_tool_parent_frame = declare_parameter<std::string>("tool_parent_frame", "robot_flange");
        m_tool_tip_frame = declare_parameter<std::string>("tool_tip_frame", "welding_torch_tip");
        m_tool_tf_timeout_sec = declare_parameter<double>("tool_tf_timeout_sec", 2.0);
        m_preview_delay_sec = declare_parameter<double>("preview_delay_sec", 0.0);
        m_execute = declare_parameter<bool>("execute", true);
        m_pose_reference_frame = declare_parameter<std::string>("pose_reference_frame", "robot_base");
        m_end_effector_link = declare_parameter<std::string>("end_effector_link", "Link6");
        m_planning_time = declare_parameter<double>("planning_time", 10.0);
        m_num_planning_attempts = declare_parameter<int>("num_planning_attempts", 10);
        m_goal_position_tolerance = declare_parameter<double>("goal_position_tolerance", 0.001);
        m_goal_orientation_tolerance = declare_parameter<double>("goal_orientation_tolerance", 0.01);
        m_cartesian_eef_step = declare_parameter<double>("cartesian_eef_step", 0.001);
        m_cartesian_min_fraction = declare_parameter<double>("cartesian_min_fraction", 1.0);
        m_path_mode = declare_parameter<std::string>("path_mode", "cartesian");
        m_velocity_scaling = declare_parameter<double>("velocity_scaling", 0.05);
        m_acceleration_scaling = declare_parameter<double>("acceleration_scaling", 0.05);
        if (!has_parameter("use_sim_time")) {
            declare_parameter<bool>("use_sim_time", false);
        }
        m_enable_table_collision = declare_parameter<bool>("enable_table_collision", true);
        m_table_frame = declare_parameter<std::string>("table_frame", "robot_base");
        // 移动桌面z以补偿安装法兰盘的厚度
        m_table_z = declare_parameter<double>("table_z", -0.015);
        m_table_size_x = declare_parameter<double>("table_size_x", 2.0);
        m_table_size_y = declare_parameter<double>("table_size_y", 2.0);
        m_table_thickness = declare_parameter<double>("table_thickness", 0.04);
        m_display_pub = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
            "/display_planned_path", 10);
        m_state_validity_client = create_client<moveit_msgs::srv::GetStateValidity>(
            "/check_state_validity");
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);
        declareKinematicsParameters();

        bool use_sim_time = false;
        get_parameter("use_sim_time", use_sim_time);
    }

    // 初始化 MoveIt 接口，包括 MoveGroupInterface、规划组和末端执行器链接
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

        m_move_group->setPoseReferenceFrame(m_pose_reference_frame);
        if (!m_end_effector_link.empty()) {
            if (!m_move_group->setEndEffectorLink(m_end_effector_link)) {
                RCLCPP_ERROR(
                    get_logger(), "无法设置末端 link 为 %s；请检查它是否属于 MoveIt 机器人模型",
                    m_end_effector_link.c_str());
                return false;
            }
        }
        m_move_group->setPlanningTime(m_planning_time);
        m_move_group->setNumPlanningAttempts(m_num_planning_attempts);
        m_move_group->setGoalPositionTolerance(m_goal_position_tolerance);
        m_move_group->setGoalOrientationTolerance(m_goal_orientation_tolerance);
        m_move_group->setMaxVelocityScalingFactor(m_velocity_scaling);
        m_move_group->setMaxAccelerationScalingFactor(m_acceleration_scaling);

        applyTableCollisionObject();
        return true;
    }

    void stopMotion()
    {
        if (m_move_group) {
            RCLCPP_WARN(get_logger(), "Stopping MoveIt execution");
            m_move_group->stop();
        }
    }

    // 设置桌面碰撞体，防止焊枪与桌面碰撞
    void applyTableCollisionObject()
    {
        if (!m_enable_table_collision) {
            return;
        }

        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = m_table_frame;
        table.id = "table_z_minus_limit";

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions.resize(3);
        box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = m_table_size_x;
        box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = m_table_size_y;
        box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = m_table_thickness;

        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;
        table_pose.position.z = m_table_z - m_table_thickness * 0.5;

        table.primitives.push_back(box);
        table.primitive_poses.push_back(table_pose);
        table.operation = moveit_msgs::msg::CollisionObject::ADD;

        // 一个move_group节点中主要维护一份PlanningScene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_interface.applyCollisionObjects({table});
        RCLCPP_INFO(
            get_logger(),
            "added table collision object: frame=%s top_z=%.4f size=[%.3f %.3f %.3f]",
            m_table_frame.c_str(), m_table_z, m_table_size_x, m_table_size_y, m_table_thickness);
    }

    // 定义IK求解器相关参数
    void declareKinematicsParameters()
    {
        const std::string prefix = "robot_description_kinematics." + m_planning_group + ".";
        declare_parameter<std::string>(
            prefix + "kinematics_solver",
            "kdl_kinematics_plugin/KDLKinematicsPlugin");
        declare_parameter<double>(
            prefix + "kinematics_solver_search_resolution",
            0.005);
        declare_parameter<double>(
            prefix + "kinematics_solver_timeout",
            0.05);
    }

    // 加载采样点
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

            sample.T_base_tip.translation.x = translation[0];
            sample.T_base_tip.translation.y = translation[1];
            sample.T_base_tip.translation.z = translation[2];

            sample.T_base_tip.rotation.x = rotation[0];
            sample.T_base_tip.rotation.y = rotation[1];
            sample.T_base_tip.rotation.z = rotation[2];
            sample.T_base_tip.rotation.w = rotation[3];
            
            m_samples.push_back(sample);
        }

        RCLCPP_INFO(get_logger(), "loaded %zu samples from %s", m_samples.size(), m_input_file.c_str());
        return true;
    }

    // 从 TF 读取工具标定
    bool lookupToolTransform()
    {
        geometry_msgs::msg::TransformStamped T_tool_parent_tip_msg;
        try {
            T_tool_parent_tip_msg = m_tf_buffer->lookupTransform(
                m_tool_parent_frame,
                m_tool_tip_frame,
                tf2::TimePointZero,
                tf2::durationFromSec(m_tool_tf_timeout_sec));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(
                get_logger(),
                "failed to lookup tool transform %s -> %s: %s",
                m_tool_parent_frame.c_str(),
                m_tool_tip_frame.c_str(),
                ex.what());
            return false;
        }

        const auto & translation = T_tool_parent_tip_msg.transform.translation;
        const auto & rotation = T_tool_parent_tip_msg.transform.rotation;

        tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
        q.normalize();
        m_T_tool_parent_tip = tf2::Transform(
            q,
            tf2::Vector3(translation.x, translation.y, translation.z));
        m_T_tip_tool_parent = m_T_tool_parent_tip.inverse();

        RCLCPP_INFO(
            get_logger(),
            "loaded tool transform from TF %s -> %s; replaying tip path as tool parent targets",
            m_tool_parent_frame.c_str(),
            m_tool_tip_frame.c_str());
        return true;
    }

    // 根据采样点的位姿以及工具标定计算 tool parent 位姿
    geometry_msgs::msg::Pose sampleTipToToolParentPose(const PathSample & sample) const
    {
        tf2::Quaternion q(
            sample.T_base_tip.rotation.x,
            sample.T_base_tip.rotation.y,
            sample.T_base_tip.rotation.z,
            sample.T_base_tip.rotation.w);
        q.normalize();

        const tf2::Transform T_base_tip(
            q,
            tf2::Vector3(
                sample.T_base_tip.translation.x,
                sample.T_base_tip.translation.y,
                sample.T_base_tip.translation.z));
        const tf2::Transform T_base_tool_parent = T_base_tip * m_T_tip_tool_parent;

        geometry_msgs::msg::Pose pose;
        pose.position.x = T_base_tool_parent.getOrigin().x();
        pose.position.y = T_base_tool_parent.getOrigin().y();
        pose.position.z = T_base_tool_parent.getOrigin().z();

        const tf2::Quaternion tool_parent_rotation = T_base_tool_parent.getRotation().normalized();
        pose.orientation.x = tool_parent_rotation.x();
        pose.orientation.y = tool_parent_rotation.y();
        pose.orientation.z = tool_parent_rotation.z();
        pose.orientation.w = tool_parent_rotation.w();
        return pose;
    }

    // 发布 MoveIt 规划的轨迹到 /display_planned_path 以便在 RViz 中预览
    void publishDisplayTrajectory(const moveit_msgs::msg::RobotTrajectory & trajectory)
    {
        if (!trajectory.joint_trajectory.points.empty()) {
            const auto & duration = trajectory.joint_trajectory.points.back().time_from_start;
            RCLCPP_INFO(
                this->get_logger(),
                "trajectory duration: %.3f sec, points=%zu",
                rclcpp::Duration(duration).seconds(),
                trajectory.joint_trajectory.points.size());
        }

        moveit_msgs::msg::DisplayTrajectory display;
        const auto current_state = m_move_group->getCurrentState(2.0);
        if (current_state) {
            moveit::core::robotStateToRobotStateMsg(*current_state, display.trajectory_start);
        }
        display.trajectory.push_back(trajectory);
        m_display_pub->publish(display);

        RCLCPP_INFO(
            this->get_logger(), "published plan to /display_planned_path; preview %.2f sec",
            m_preview_delay_sec);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(m_preview_delay_sec)));
    }

    // 根据当前pose作为seed计算目标pose的IK
    bool computeSeededIkTarget(const geometry_msgs::msg::Pose & pose, moveit::core::RobotState & ik_state) const
    {
        const auto current_state = m_move_group->getCurrentState(2.0);
        if (!current_state) {
            RCLCPP_WARN(get_logger(), "无法获取当前状态，无法计算 seeded IK");
            return false;
        }

        ik_state = *current_state;
        const bool success = ik_state.setFromIK(
            m_joint_model_group,
            pose,
            m_move_group->getEndEffectorLink(),
            0.2);
        if (success) {
            unwrapTargetNearReference(*current_state, ik_state);
        }
        return success;
    }

    // 将目标关节状态解包到参考状态附近，以避免关节角度跳跃
    void unwrapTargetNearReference(
        const moveit::core::RobotState & reference_state,
        moveit::core::RobotState & target_state) const
    {
        constexpr double two_pi = 2.0 * M_PI;
        const auto & variable_names = m_joint_model_group->getVariableNames();
        // 获取参考和目标的关节角度以及机械臂的关节限位
        for (const auto & name : variable_names) {
            const double reference = reference_state.getVariablePosition(name);
            const double target = target_state.getVariablePosition(name);
            const auto & bounds = target_state.getRobotModel()->getVariableBounds(name);

            // 对于每个关节角，在目标角度的+-2个周期范围内寻找最接近参考的角度作为新的目标角度
            double best = target;
            double best_abs_delta = std::abs(target - reference);
            for (int offset = -2; offset <= 2; ++offset) {
                const double candidate = target + static_cast<double>(offset) * two_pi;
                if (bounds.position_bounded_ &&
                    (candidate < bounds.min_position_ || candidate > bounds.max_position_)) {
                    continue;
                }
                const double abs_delta = std::abs(candidate - reference);
                if (abs_delta < best_abs_delta) {
                    best = candidate;
                    best_abs_delta = abs_delta;
                }
            }
            target_state.setVariablePosition(name, best);
        }
    }

    // 打印关节角度的变化量
    void logJointDeltas(
        const moveit::core::RobotState & reference_state,
        const moveit::core::RobotState & target_state,
        const std::string & label) const
    {
        const auto & variable_names = m_joint_model_group->getVariableNames();
        std::string message = label + " joint deltas:";
        for (const auto & name : variable_names) {
            const double delta = target_state.getVariablePosition(name) -
                reference_state.getVariablePosition(name);
            message += " " + name + "=" + std::to_string(delta);
        }
        RCLCPP_INFO(get_logger(), "%s", message.c_str());
    }

    // 检查机器人状态在 MoveIt 场景中的有效性，包括关节限位和碰撞检测
    bool logStateValidity(
        const moveit::core::RobotState & state,
        const std::string & label) const
    {
        if (m_state_validity_client &&
            m_state_validity_client->wait_for_service(std::chrono::milliseconds(200))) { // 查询服务是否存在
            auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
            moveit::core::robotStateToRobotStateMsg(state, request->robot_state); // 把当前state转为ROS消息
            request->group_name = m_planning_group; // 指定规划组

            auto future = m_state_validity_client->async_send_request(request); // send并且等待，最多2s
            const auto status = future.wait_for(std::chrono::seconds(2));
            if (status == std::future_status::ready) {
                const auto response = future.get();
                // 检查碰撞
                if (response->valid) {
                    RCLCPP_INFO(get_logger(), "%s validity in move_group scene: ok", label.c_str());
                    return true;
                }

                RCLCPP_ERROR(get_logger(), "%s invalid in move_group scene", label.c_str());
                for (const auto & contact : response->contacts) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "  collision pair: %s <-> %s",
                        contact.contact_body_1.c_str(),
                        contact.contact_body_2.c_str());
                }
                return false;
            }

            RCLCPP_WARN(get_logger(), "%s validity service timed out; falling back to local scene", label.c_str());
        }

        // 超时后，使用本地场景进行碰撞检测
        // 检查关节限位
        bool valid = true;
        if (!state.satisfiesBounds(m_joint_model_group)) {
            RCLCPP_ERROR(get_logger(), "%s violates joint bounds", label.c_str());
            valid = false;
        }

        // 临时创建空的planningscene，使用当前state进行碰撞检测
        // 只包含机器人模型，仅能检查自碰撞
        planning_scene::PlanningScene planning_scene(m_move_group->getRobotModel());
        collision_detection::CollisionRequest request;
        collision_detection::CollisionResult result;
        request.contacts = true;
        request.max_contacts = 20;
        planning_scene.checkCollision(request, result, state);
        if (result.collision) {
            RCLCPP_ERROR(get_logger(), "%s is in collision", label.c_str());
            for (const auto & contact_entry : result.contacts) {
                RCLCPP_ERROR(
                    get_logger(),
                    "  collision pair: %s <-> %s contacts=%zu",
                    contact_entry.first.first.c_str(),
                    contact_entry.first.second.c_str(),
                    contact_entry.second.size());
            }
            valid = false;
        }

        if (valid) {
            RCLCPP_INFO(get_logger(), "%s validity: ok", label.c_str());
        }
        return valid;
    }

    // 规划并根据 execute 参数决定是否执行关节目标
    bool planAndMaybeExecuteJointTarget(
        const moveit::core::RobotState & target_state,
        const std::string & label)
    {
        const auto current_state = m_move_group->getCurrentState(2.0);
        if (current_state) {
            logJointDeltas(*current_state, target_state, label); // 打印关节角度变化两
            logStateValidity(*current_state, label + " start state"); // 打印当前状态的有效性(限位+碰撞)
            logStateValidity(target_state, label + " target state"); // 打印目标状态的有效性(限位+碰撞)
        }

        m_move_group->setStartStateToCurrentState();
        m_move_group->clearPoseTargets();
        m_move_group->setJointValueTarget(target_state);

        // 规划轨迹
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto plan_result = m_move_group->plan(plan);
        if (plan_result != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s 规划失败", label.c_str());
            return false;
        }

        // 发布轨迹到 /display_planned_path
        publishDisplayTrajectory(plan.trajectory_);
        if (!m_execute) {
            RCLCPP_INFO(this->get_logger(), "execute=false; only displayed %s plan", label.c_str());
            return true;
        }

        RCLCPP_INFO(this->get_logger(), "executing %s plan", label.c_str());
        const auto execute_result = m_move_group->execute(plan);
        if (execute_result != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s 执行失败", label.c_str());
            return false;
        }
        return true;
    }

    // 重现输入文件的采样点位姿（通过对于每个采样点直接计算 IK 并规划关节轨迹）
    bool replayJointWaypoints()
    {
        for (size_t sample_index = 1; sample_index < m_samples.size(); ++sample_index) {
            const auto tool_parent_pose = sampleTipToToolParentPose(m_samples[sample_index]);
            RCLCPP_INFO(
                this->get_logger(),
                "joint waypoint %zu xyz=[%.6f, %.6f, %.6f] qxyzw=[%.6f, %.6f, %.6f, %.6f]",
                sample_index,
                tool_parent_pose.position.x, tool_parent_pose.position.y, tool_parent_pose.position.z,
                tool_parent_pose.orientation.x, tool_parent_pose.orientation.y,
                tool_parent_pose.orientation.z, tool_parent_pose.orientation.w);

            const auto current_state = m_move_group->getCurrentState(2.0);
            if (!current_state) {
                RCLCPP_ERROR(this->get_logger(), "无法获取当前状态，无法规划 waypoint %zu", sample_index);
                return false;
            }
            moveit::core::RobotState target_state(*current_state);
            if (!computeSeededIkTarget(tool_parent_pose, target_state)) {
                RCLCPP_ERROR(this->get_logger(), "waypoint %zu IK 失败", sample_index);
                return false;
            }
            if (!planAndMaybeExecuteJointTarget(
                    target_state, "waypoint " + std::to_string(sample_index))) {
                return false;
            }
        }
        return true;
    }

    // 重现输入文件的采样点位姿（在采样点之间插值Cartesian Path后规划关节轨迹）
    bool replayCartesianWaypoints()
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        for (size_t sample_index = 1; sample_index < m_samples.size(); ++sample_index) {
            const auto & sample = m_samples[sample_index];
            auto tool_parent_pose = sampleTipToToolParentPose(sample);
            RCLCPP_INFO(
                this->get_logger(),
                "tool parent waypoint %zu xyz=[%.6f, %.6f, %.6f] qxyzw=[%.6f, %.6f, %.6f, %.6f]",
                sample_index,
                tool_parent_pose.position.x, tool_parent_pose.position.y, tool_parent_pose.position.z,
                tool_parent_pose.orientation.x, tool_parent_pose.orientation.y,
                tool_parent_pose.orientation.z, tool_parent_pose.orientation.w);
            waypoints.push_back(tool_parent_pose);
        }

        if (waypoints.empty()) {
            RCLCPP_INFO(this->get_logger(), "only one sample; initial target move completed");
            return true;
        }

        moveit_msgs::msg::RobotTrajectory trajectory;
        m_move_group->setStartStateToCurrentState();
        double fraction = m_move_group->computeCartesianPath(waypoints, m_cartesian_eef_step, 0.0, trajectory);
        if (fraction < m_cartesian_min_fraction) {
            RCLCPP_ERROR(
                this->get_logger(),
                "路径规划失败，只有%.1f%%的点成功规划；cartesian_eef_step=%.4f, required=%.1f%%",
                fraction * 100.0, m_cartesian_eef_step, m_cartesian_min_fraction * 100.0);
            logCartesianFractionLocation(waypoints, fraction);
            diagnoseCartesianFailure(waypoints);
            return false;
        } else if (fraction < 1.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "Cartesian path only reached %.1f%%; executing because cartesian_min_fraction=%.1f%%",
                fraction * 100.0, m_cartesian_min_fraction * 100.0);
        }

        return timeParameterizeDisplayAndExecute(trajectory);
    }

    tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose) const
    {
        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        q.normalize();
        return tf2::Transform(
            q,
            tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
    }

    geometry_msgs::msg::Pose transformToPose(const tf2::Transform & T) const
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = T.getOrigin().x();
        pose.position.y = T.getOrigin().y();
        pose.position.z = T.getOrigin().z();
        const auto q = T.getRotation().normalized();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        return pose;
    }

    // 在两个位姿之间进行线性插值，t为插值参数，范围为[0, 1]
    geometry_msgs::msg::Pose interpolatePose(
        const geometry_msgs::msg::Pose & start,
        const geometry_msgs::msg::Pose & goal,
        double t) const
    {
        const auto T_base_start = poseToTransform(start);
        const auto T_base_goal = poseToTransform(goal);
        const auto p = T_base_start.getOrigin().lerp(T_base_goal.getOrigin(), t);
        const auto q = T_base_start.getRotation().slerp(T_base_goal.getRotation(), t).normalized();
        return transformToPose(tf2::Transform(q, p));
    }

    void logCartesianFractionLocation(
        const std::vector<geometry_msgs::msg::Pose> & waypoints,
        double fraction) const
    {
        auto segment_start = m_move_group->getCurrentPose(m_move_group->getEndEffectorLink()).pose;
        std::vector<std::size_t> segment_steps;
        std::vector<double> segment_distances;
        std::size_t total_steps = 0;
        double total_distance = 0.0;

        for (const auto & waypoint : waypoints) {
            const auto T_base_segment_start = poseToTransform(segment_start);
            const auto T_base_waypoint = poseToTransform(waypoint);
            const double distance = T_base_segment_start.getOrigin().distance(T_base_waypoint.getOrigin());
            const std::size_t step_count = std::max<std::size_t>(
                1, static_cast<std::size_t>(std::ceil(distance / m_cartesian_eef_step)));

            segment_steps.push_back(step_count);
            segment_distances.push_back(distance);
            total_steps += step_count;
            total_distance += distance;
            segment_start = waypoint;
        }

        if (total_steps == 0 || waypoints.empty()) {
            return;
        }

        const auto completed_steps = static_cast<std::size_t>(
            std::floor(std::clamp(fraction, 0.0, 1.0) * static_cast<double>(total_steps)));
        std::size_t accumulated_steps = 0;
        double accumulated_distance = 0.0;
        for (std::size_t index = 0; index < segment_steps.size(); ++index) {
            const auto next_accumulated_steps = accumulated_steps + segment_steps[index];
            const auto segment_number = index + 1;
            if (completed_steps <= next_accumulated_steps) {
                const auto step_in_segment = completed_steps > accumulated_steps ?
                    completed_steps - accumulated_steps : 0;
                const double segment_fraction = segment_steps[index] > 0 ?
                    static_cast<double>(step_in_segment) / static_cast<double>(segment_steps[index]) : 0.0;
                const double approximate_distance =
                    accumulated_distance + segment_fraction * segment_distances[index];

                RCLCPP_ERROR(
                    get_logger(),
                    "Cartesian path stopped around waypoint segment %zu/%zu, step %zu/%zu, approx distance %.4f/%.4f m",
                    segment_number,
                    segment_steps.size(),
                    step_in_segment,
                    segment_steps[index],
                    approximate_distance,
                    total_distance);
                return;
            }
            accumulated_steps = next_accumulated_steps;
            accumulated_distance += segment_distances[index];
        }
    }

    void diagnoseCartesianFailure(const std::vector<geometry_msgs::msg::Pose> & waypoints) const
    {
        const auto current_state = m_move_group->getCurrentState(2.0);
        if (!current_state) {
            RCLCPP_WARN(get_logger(), "Cartesian diagnose skipped: cannot get current state");
            return;
        }

        planning_scene::PlanningScene planning_scene(m_move_group->getRobotModel());

        auto diagnostic_state = *current_state;
        auto segment_start = m_move_group->getCurrentPose(m_move_group->getEndEffectorLink()).pose;
        std::size_t global_step = 0;

        for (std::size_t waypoint_index = 0; waypoint_index < waypoints.size(); ++waypoint_index) {
            const auto & segment_goal = waypoints[waypoint_index];
            const auto T_base_segment_start = poseToTransform(segment_start);
            const auto T_base_segment_goal = poseToTransform(segment_goal);
            const double distance = T_base_segment_start.getOrigin().distance(T_base_segment_goal.getOrigin());
            const std::size_t step_count = std::max<std::size_t>(
                1, static_cast<std::size_t>(std::ceil(distance / m_cartesian_eef_step)));

            for (std::size_t step = 1; step <= step_count; ++step) {
                ++global_step;
                const double t = static_cast<double>(step) / static_cast<double>(step_count);
                const auto pose = interpolatePose(segment_start, segment_goal, t);

                auto ik_state = diagnostic_state;
                const bool ik_success = ik_state.setFromIK(
                    m_joint_model_group,
                    pose,
                    m_move_group->getEndEffectorLink(),
                    0.2);
                if (!ik_success) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Cartesian diagnose: IK failed at waypoint=%zu step=%zu/%zu global_step=%zu xyz=[%.6f, %.6f, %.6f] qxyzw=[%.6f, %.6f, %.6f, %.6f]",
                        waypoint_index + 1, step, step_count, global_step,
                        pose.position.x, pose.position.y, pose.position.z,
                        pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
                    return;
                }

                unwrapTargetNearReference(diagnostic_state, ik_state);
                if (!ik_state.satisfiesBounds(m_joint_model_group)) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Cartesian diagnose: joint bounds violated at waypoint=%zu step=%zu/%zu global_step=%zu",
                        waypoint_index + 1, step, step_count, global_step);
                    logJointDeltas(diagnostic_state, ik_state, "bounds failure");
                    return;
                }

                collision_detection::CollisionRequest request;
                collision_detection::CollisionResult result;
                request.contacts = true;
                request.max_contacts = 20;
                planning_scene.checkCollision(request, result, ik_state);
                if (result.collision) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Cartesian diagnose: collision at waypoint=%zu step=%zu/%zu global_step=%zu xyz=[%.6f, %.6f, %.6f]",
                        waypoint_index + 1, step, step_count, global_step,
                        pose.position.x, pose.position.y, pose.position.z);
                    for (const auto & contact_entry : result.contacts) {
                        RCLCPP_ERROR(
                            get_logger(),
                            "  collision pair: %s <-> %s contacts=%zu",
                            contact_entry.first.first.c_str(),
                            contact_entry.first.second.c_str(),
                            contact_entry.second.size());
                    }
                    return;
                }

                diagnostic_state = ik_state;
            }
            segment_start = segment_goal;
        }

        RCLCPP_WARN(
            get_logger(),
            "Cartesian diagnose found no IK/bounds/collision failure. computeCartesianPath may be failing due to jump filtering or internal IK discretization.");
    }

    bool timeParameterizeDisplayAndExecute(moveit_msgs::msg::RobotTrajectory & trajectory)
    {
        robot_trajectory::RobotTrajectory robot_trajectory(
            m_move_group->getRobotModel(),
            m_planning_group);
        robot_trajectory.setRobotTrajectoryMsg(
            *m_move_group->getCurrentState(),
            trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
        const bool time_success = time_parameterization.computeTimeStamps(
            robot_trajectory,
            m_velocity_scaling,
            m_acceleration_scaling);
        if (!time_success) {
            RCLCPP_ERROR(this->get_logger(), "轨迹时间参数化失败");
            return false;
        }

        robot_trajectory.getRobotTrajectoryMsg(trajectory);
        publishDisplayTrajectory(trajectory);
        if (!m_execute) {
            RCLCPP_INFO(this->get_logger(), "execute=false; only displayed cartesian path");
            return true;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        RCLCPP_INFO(this->get_logger(), "executing cartesian path");
        const auto execute_result = m_move_group->execute(plan);
        if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "轨迹执行失败");
            return false;
        }
        return true;
    }

    bool planAndMaybeExecuteHome(const std::string & label)
    {
        const auto current_state = m_move_group->getCurrentState(2.0);
        if (!current_state) {
            RCLCPP_ERROR(this->get_logger(), "无法获取当前状态，无法规划 %s", label.c_str());
            return false;
        }

        moveit::core::RobotState home_state(*current_state);
        const std::vector<double> home_joints = {
            1.74533, 2.0944, -2.26893, 3.14159, 1.5708, 3.14159};
        home_state.setJointGroupPositions(m_joint_model_group, home_joints);
        home_state.update();

        return planAndMaybeExecuteJointTarget(home_state, label);
    }

    void returnHomeAfterFailure(const std::string & reason)
    {
        RCLCPP_WARN(this->get_logger(), "%s，尝试回 home", reason.c_str());
        if (!planAndMaybeExecuteHome("home after failure")) {
            RCLCPP_ERROR(this->get_logger(), "失败后回 home 也失败");
        }
    }

    void replayPath()
    {
        if (m_samples.empty()) {
            RCLCPP_ERROR(this->get_logger(), "采样点为空");
            return;
        }

        if (!planAndMaybeExecuteHome("home start")) {
            return;
        }

        // 首先移动到轨迹开始点
        const auto & first_sample = m_samples[0];
        const geometry_msgs::msg::Pose pose = sampleTipToToolParentPose(first_sample);
        RCLCPP_INFO(
            this->get_logger(),
            "initial tool parent target xyz=[%.6f, %.6f, %.6f] qxyzw=[%.6f, %.6f, %.6f, %.6f]",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        const auto current_state = m_move_group->getCurrentState(2.0);
        if (!current_state) {
            RCLCPP_ERROR(this->get_logger(), "无法获取当前状态，无法规划到初始采样点");
            returnHomeAfterFailure("无法规划到初始采样点");
            return;
        }
        moveit::core::RobotState ik_target(*current_state);
        const bool ik_success = computeSeededIkTarget(pose, ik_target);
        RCLCPP_INFO(
            this->get_logger(), "initial target IK: %s",
            ik_success ? "success" : "failed");
        if (!ik_success) {
            RCLCPP_ERROR(this->get_logger(), "初始采样点 IK 失败");
            returnHomeAfterFailure("初始采样点 IK 失败");
            return;
        }
        if (!planAndMaybeExecuteJointTarget(ik_target, "initial target")) {
            returnHomeAfterFailure("initial target 失败");
            return;
        }

        bool path_success = false;
        if (m_path_mode == "joint") {
            path_success = replayJointWaypoints();
        } else if (m_path_mode == "cartesian") {
            path_success = replayCartesianWaypoints();
        } else {
            RCLCPP_ERROR(this->get_logger(), "unknown path_mode: %s", m_path_mode.c_str());
            returnHomeAfterFailure("path_mode 无效");
            return;
        }

        if (!path_success) {
            returnHomeAfterFailure("路径执行失败");
            return;
        }

        planAndMaybeExecuteHome("home end");
    }

private:
    std::string m_model;
    std::string m_planning_group;
    std::string m_input_file;
    std::string m_tool_parent_frame;
    std::string m_tool_tip_frame;
    double m_tool_tf_timeout_sec;
    double m_preview_delay_sec;
    bool m_execute;
    std::string m_pose_reference_frame;
    std::string m_end_effector_link;
    double m_planning_time;
    int m_num_planning_attempts;
    double m_goal_position_tolerance;
    double m_goal_orientation_tolerance;
    double m_cartesian_eef_step;
    double m_cartesian_min_fraction;
    std::string m_path_mode;
    double m_velocity_scaling;
    double m_acceleration_scaling;
    bool m_enable_table_collision;
    std::string m_table_frame;
    double m_table_z;
    double m_table_size_x;
    double m_table_size_y;
    double m_table_thickness;
    tf2::Transform m_T_tool_parent_tip;
    tf2::Transform m_T_tip_tool_parent;
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr m_display_pub;
    rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr m_state_validity_client;
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;

    std::vector<PathSample> m_samples;
    std::unique_ptr<
        moveit::planning_interface::MoveGroupInterface> m_move_group;
    const moveit::core::JointModelGroup* m_joint_model_group;
};

void handleReplaySigint(int)
{
    if (g_sigint_seen.exchange(true)) {
        std::_Exit(130);
    }

    std::shared_ptr<ReplayPathNode> node;
    {
        std::lock_guard<std::mutex> lock(g_replay_node_mutex);
        node = g_replay_node.lock();
    }
    if (node) {
        node->stopMotion();
    }
    rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReplayPathNode>();
    {
        std::lock_guard<std::mutex> lock(g_replay_node_mutex);
        g_replay_node = node;
    }
    std::signal(SIGINT, handleReplaySigint);

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

    if (!node->lookupToolTransform()) {
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
