#include <fstream>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
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
        m_tool_calibration_file = declare_parameter<std::string>(
            "tool_calibration_file", "config/calibration/welding_torch_tip.yaml");
        m_preview_delay_sec = declare_parameter<double>("preview_delay_sec", 1.0);
        m_execute = declare_parameter<bool>("execute", true);
        m_pose_reference_frame = declare_parameter<std::string>("pose_reference_frame", "robot_base");
        m_end_effector_link = declare_parameter<std::string>("end_effector_link", "Link6");
        m_planning_time = declare_parameter<double>("planning_time", 10.0);
        m_num_planning_attempts = declare_parameter<int>("num_planning_attempts", 10);
        m_goal_position_tolerance = declare_parameter<double>("goal_position_tolerance", 0.001);
        m_goal_orientation_tolerance = declare_parameter<double>("goal_orientation_tolerance", 0.01);
        m_cartesian_eef_step = declare_parameter<double>("cartesian_eef_step", 0.005);
        m_cartesian_min_fraction = declare_parameter<double>("cartesian_min_fraction", 1.0);
        m_path_mode = declare_parameter<std::string>("path_mode", "cartesian");
        m_enable_table_collision = declare_parameter<bool>("enable_table_collision", true);
        m_table_frame = declare_parameter<std::string>("table_frame", "robot_base");
        m_table_z = declare_parameter<double>("table_z", 0.0);
        m_table_size_x = declare_parameter<double>("table_size_x", 2.0);
        m_table_size_y = declare_parameter<double>("table_size_y", 2.0);
        m_table_thickness = declare_parameter<double>("table_thickness", 0.04);
        m_display_pub = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
            "/display_planned_path", 10);
        declareKinematicsParameters();
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

        applyTableCollisionObject();
        return true;
    }

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

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_interface.applyCollisionObjects({table});
        RCLCPP_INFO(
            get_logger(),
            "added table collision object: frame=%s top_z=%.4f size=[%.3f %.3f %.3f]",
            m_table_frame.c_str(), m_table_z, m_table_size_x, m_table_size_y, m_table_thickness);
    }

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

    bool loadToolTransform()
    {
        YAML::Node root;
        try {
            std::ifstream input(m_tool_calibration_file);
            if (!input.is_open()) {
                RCLCPP_ERROR(
                    get_logger(), "failed to open tool calibration file: %s",
                    m_tool_calibration_file.c_str());
                return false;
            }
            root = YAML::Load(input);
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(
                get_logger(), "failed to load %s: %s",
                m_tool_calibration_file.c_str(), ex.what());
            return false;
        }

        const YAML::Node transform = root["calibration_result"] ? root["calibration_result"] : root;
        if (!transform["translation"] || !transform["rotation_xyzw"]) {
            RCLCPP_ERROR(
                get_logger(), "%s missing translation or rotation_xyzw",
                m_tool_calibration_file.c_str());
            return false;
        }

        const auto translation = transform["translation"].as<std::vector<double>>();
        const auto rotation = transform["rotation_xyzw"].as<std::vector<double>>();
        if (translation.size() != 3 || rotation.size() != 4) {
            RCLCPP_ERROR(get_logger(), "tool calibration transform has invalid dimensions");
            return false;
        }

        tf2::Quaternion q(rotation[0], rotation[1], rotation[2], rotation[3]);
        q.normalize();
        m_flange_to_tip = tf2::Transform(
            q,
            tf2::Vector3(translation[0], translation[1], translation[2]));
        m_tip_to_flange = m_flange_to_tip.inverse();

        RCLCPP_INFO(
            get_logger(), "loaded tool transform from %s; replaying tip path as flange targets",
            m_tool_calibration_file.c_str());
        return true;
    }

    geometry_msgs::msg::Pose sampleTipToFlangePose(const PathSample & sample) const
    {
        tf2::Quaternion q(
            sample.transform.rotation.x,
            sample.transform.rotation.y,
            sample.transform.rotation.z,
            sample.transform.rotation.w);
        q.normalize();

        const tf2::Transform base_to_tip(
            q,
            tf2::Vector3(
                sample.transform.translation.x,
                sample.transform.translation.y,
                sample.transform.translation.z));
        const tf2::Transform base_to_flange = base_to_tip * m_tip_to_flange;

        geometry_msgs::msg::Pose pose;
        pose.position.x = base_to_flange.getOrigin().x();
        pose.position.y = base_to_flange.getOrigin().y();
        pose.position.z = base_to_flange.getOrigin().z();

        const tf2::Quaternion flange_rotation = base_to_flange.getRotation().normalized();
        pose.orientation.x = flange_rotation.x();
        pose.orientation.y = flange_rotation.y();
        pose.orientation.z = flange_rotation.z();
        pose.orientation.w = flange_rotation.w();
        return pose;
    }

    void publishDisplayTrajectory(const moveit_msgs::msg::RobotTrajectory & trajectory)
    {
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

    void unwrapTargetNearReference(
        const moveit::core::RobotState & reference_state,
        moveit::core::RobotState & target_state) const
    {
        constexpr double two_pi = 2.0 * M_PI;
        const auto & variable_names = m_joint_model_group->getVariableNames();
        for (const auto & name : variable_names) {
            const double reference = reference_state.getVariablePosition(name);
            const double target = target_state.getVariablePosition(name);
            const auto & bounds = target_state.getRobotModel()->getVariableBounds(name);

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

    bool planAndMaybeExecuteJointTarget(
        const moveit::core::RobotState & target_state,
        const std::string & label)
    {
        const auto current_state = m_move_group->getCurrentState(2.0);
        if (current_state) {
            logJointDeltas(*current_state, target_state, label);
        }

        m_move_group->setStartStateToCurrentState();
        m_move_group->clearPoseTargets();
        m_move_group->setJointValueTarget(target_state);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto plan_result = m_move_group->plan(plan);
        if (plan_result != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s 规划失败", label.c_str());
            return false;
        }

        publishDisplayTrajectory(plan.trajectory_);
        if (!m_execute) {
            RCLCPP_INFO(this->get_logger(), "execute=false; only displayed %s plan", label.c_str());
            return true;
        }

        const auto execute_result = m_move_group->execute(plan);
        if (execute_result != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s 执行失败", label.c_str());
            return false;
        }
        return true;
    }

    bool replayJointWaypoints()
    {
        for (size_t sample_index = 1; sample_index < m_samples.size(); ++sample_index) {
            const auto flange_pose = sampleTipToFlangePose(m_samples[sample_index]);
            RCLCPP_INFO(
                this->get_logger(),
                "joint waypoint %zu xyz=[%.6f, %.6f, %.6f] qxyzw=[%.6f, %.6f, %.6f, %.6f]",
                sample_index,
                flange_pose.position.x, flange_pose.position.y, flange_pose.position.z,
                flange_pose.orientation.x, flange_pose.orientation.y,
                flange_pose.orientation.z, flange_pose.orientation.w);

            const auto current_state = m_move_group->getCurrentState(2.0);
            if (!current_state) {
                RCLCPP_ERROR(this->get_logger(), "无法获取当前状态，无法规划 waypoint %zu", sample_index);
                return false;
            }
            moveit::core::RobotState target_state(*current_state);
            if (!computeSeededIkTarget(flange_pose, target_state)) {
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

    bool replayCartesianWaypoints()
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        for (size_t sample_index = 1; sample_index < m_samples.size(); ++sample_index) {
            const auto & sample = m_samples[sample_index];
            auto flange_pose = sampleTipToFlangePose(sample);
            RCLCPP_INFO(
                this->get_logger(),
                "flange waypoint %zu xyz=[%.6f, %.6f, %.6f] qxyzw=[%.6f, %.6f, %.6f, %.6f]",
                sample_index,
                flange_pose.position.x, flange_pose.position.y, flange_pose.position.z,
                flange_pose.orientation.x, flange_pose.orientation.y,
                flange_pose.orientation.z, flange_pose.orientation.w);
            waypoints.push_back(flange_pose);
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
            return false;
        } else if (fraction < 1.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "Cartesian path only reached %.1f%%; executing because cartesian_min_fraction=%.1f%%",
                fraction * 100.0, m_cartesian_min_fraction * 100.0);
        }

        return timeParameterizeDisplayAndExecute(trajectory);
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
            0.05,
            0.05);
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

        const auto execute_result = m_move_group->execute(plan);
        if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "轨迹执行失败");
            return false;
        }
        return true;
    }

    void replayPath()
    {
        if (m_samples.empty()) {
            RCLCPP_ERROR(this->get_logger(), "采样点为空");
            return;
        }
        // 首先移动到轨迹开始点
        const auto & first_sample = m_samples[0];
        const geometry_msgs::msg::Pose pose = sampleTipToFlangePose(first_sample);
        RCLCPP_INFO(
            this->get_logger(),
            "initial flange target xyz=[%.6f, %.6f, %.6f] qxyzw=[%.6f, %.6f, %.6f, %.6f]",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        const auto current_state = m_move_group->getCurrentState(2.0);
        if (!current_state) {
            RCLCPP_ERROR(this->get_logger(), "无法获取当前状态，无法规划到初始采样点");
            return;
        }
        moveit::core::RobotState ik_target(*current_state);
        const bool ik_success = computeSeededIkTarget(pose, ik_target);
        RCLCPP_INFO(
            this->get_logger(), "initial target IK: %s",
            ik_success ? "success" : "failed");
        if (!ik_success) {
            RCLCPP_ERROR(this->get_logger(), "初始采样点 IK 失败");
            return;
        }
        if (!planAndMaybeExecuteJointTarget(ik_target, "initial target")) {
            return;
        }

        if (m_path_mode == "joint") {
            replayJointWaypoints();
            return;
        }

        if (m_path_mode != "cartesian") {
            RCLCPP_ERROR(this->get_logger(), "unknown path_mode: %s", m_path_mode.c_str());
            return;
        }

        replayCartesianWaypoints();
    }

private:
    std::string m_model;
    std::string m_planning_group;
    std::string m_input_file;
    std::string m_tool_calibration_file;
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
    bool m_enable_table_collision;
    std::string m_table_frame;
    double m_table_z;
    double m_table_size_x;
    double m_table_size_y;
    double m_table_thickness;
    tf2::Transform m_flange_to_tip;
    tf2::Transform m_tip_to_flange;
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr m_display_pub;

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

    if (!node->loadToolTransform()) {
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
