#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "jaka_planner/JAKAZuRobot.h"

class JakaTrajectoryExecutorNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  JakaTrajectoryExecutorNode()
  : Node("jaka_trajectory_executor_node")
  {
    m_robot_ip = declare_parameter<std::string>("ip", "10.5.5.100");
    m_model = declare_parameter<std::string>("model", "zu5");
    m_publish_rate = declare_parameter<double>("joint_state_publish_rate", 125.0);
    m_servo_period = declare_parameter<double>("servo_period", 0.008);
    m_reach_tolerance_deg = declare_parameter<double>("reach_tolerance_deg", 0.2);
    m_auto_power_on = declare_parameter<bool>("auto_power_on", true);
    m_auto_enable = declare_parameter<bool>("auto_enable", true);

    m_joint_state_pub = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    connectRobot();

    const auto action_name = "/jaka_" + m_model + "_controller/follow_joint_trajectory";
    m_action_server = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      action_name,
      std::bind(&JakaTrajectoryExecutorNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JakaTrajectoryExecutorNode::handleCancel, this, std::placeholders::_1),
      std::bind(&JakaTrajectoryExecutorNode::handleAccepted, this, std::placeholders::_1));

    const auto publish_period = std::chrono::duration<double>(1.0 / m_publish_rate);
    m_joint_state_timer = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&JakaTrajectoryExecutorNode::publishJointStates, this));

    RCLCPP_INFO(get_logger(), "JAKA trajectory executor ready: %s", action_name.c_str());
  }

  ~JakaTrajectoryExecutorNode() override
  {
    m_robot.servo_move_enable(false);
  }

private:
  void connectRobot()
  {
    RCLCPP_INFO(get_logger(), "Connecting to JAKA robot: %s", m_robot_ip.c_str());
    const int login_result = m_robot.login_in(m_robot_ip.c_str(), false);
    if (login_result != 0) {
      RCLCPP_ERROR(get_logger(), "JAKA login failed: %d", login_result);
      return;
    }

    m_robot.servo_move_enable(false);
    m_robot.servo_move_use_joint_LPF(0.5);

    if (m_auto_power_on) {
      const int power_result = m_robot.power_on();
      RCLCPP_INFO(get_logger(), "JAKA power_on result: %d", power_result);
      rclcpp::sleep_for(std::chrono::seconds(8));
    }

    if (m_auto_enable) {
      const int enable_result = m_robot.enable_robot();
      RCLCPP_INFO(get_logger(), "JAKA enable_robot result: %d", enable_result);
      rclcpp::sleep_for(std::chrono::seconds(4));
    }
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    (void)uuid;
    const auto point_count = goal->trajectory.points.size();
    if (point_count == 0) {
      RCLCPP_ERROR(get_logger(), "Rejecting empty trajectory goal");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(), "Accepted trajectory goal with %zu points", point_count);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_WARN(get_logger(), "Cancel request received");
    m_robot.motion_abort();
    m_robot.servo_move_enable(false);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    std::thread([this, goal_handle]() {
      executeTrajectory(goal_handle);
    }).detach();
  }

  void executeTrajectory(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const auto & trajectory = goal->trajectory;
    const auto & points = trajectory.points;

    if (points.empty()) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      goal_handle->abort(result);
      return;
    }

    const int servo_enable_result = m_robot.servo_move_enable(true);
    if (servo_enable_result != 0) {
      RCLCPP_ERROR(get_logger(), "servo_move_enable(true) failed: %d", servo_enable_result);
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      goal_handle->abort(result);
      return;
    }

    double last_time = 0.0;
    JointValue target_joint{};

    for (std::size_t i = 0; i < points.size(); ++i) {
      if (goal_handle->is_canceling()) {
        m_robot.motion_abort();
        m_robot.servo_move_enable(false);
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        goal_handle->canceled(result);
        return;
      }

      const auto & point = points[i];
      if (point.positions.size() < 6) {
        RCLCPP_ERROR(get_logger(), "Trajectory point %zu has only %zu joint positions", i, point.positions.size());
        m_robot.servo_move_enable(false);
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        goal_handle->abort(result);
        return;
      }

      for (int joint_index = 0; joint_index < 6; ++joint_index) {
        target_joint.jVal[joint_index] = point.positions[static_cast<std::size_t>(joint_index)];
      }

      const double current_time =
        static_cast<double>(point.time_from_start.sec) +
        static_cast<double>(point.time_from_start.nanosec) * 1e-9;
      const double dt = std::max(current_time - last_time, m_servo_period);
      last_time = current_time;

      int step_num = static_cast<int>(std::round(dt / m_servo_period));
      step_num = std::max(step_num, 1);

      const int servo_result = m_robot.servo_j(&target_joint, MoveMode::ABS, step_num);
      if (servo_result != 0) {
        RCLCPP_ERROR(get_logger(), "servo_j failed at point %zu: %d", i, servo_result);
        m_robot.servo_move_enable(false);
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        goal_handle->abort(result);
        return;
      }
    }

    waitUntilReached(target_joint);
    m_robot.servo_move_enable(false);

    auto result = std::make_shared<FollowJointTrajectory::Result>();
    goal_handle->succeed(result);
  }

  void publishJointStates()
  {
    JointValue joint_position{};
    const int result = m_robot.get_joint_position(&joint_position);
    if (result != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "get_joint_position failed: %d", result);
      return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    for (int i = 0; i < 6; ++i) {
      msg.name.push_back("joint_" + std::to_string(i + 1));
      msg.position.push_back(joint_position.jVal[i]);
    }

    m_joint_state_pub->publish(msg);
  }

  bool isReached(const JointValue & target_joint)
  {
    JointValue current_joint{};
    const int result = m_robot.get_joint_position(&current_joint);
    if (result != 0) {
      RCLCPP_WARN(get_logger(), "get_joint_position failed while checking target: %d", result);
      return false;
    }

    constexpr double rad_to_deg = 180.0 / M_PI;
    for (int i = 0; i < 6; ++i) {
      const double current_deg = current_joint.jVal[i] * rad_to_deg;
      const double target_deg = target_joint.jVal[i] * rad_to_deg;
      if (std::abs(current_deg - target_deg) > m_reach_tolerance_deg) {
        return false;
      }
    }

    return true;
  }

  void waitUntilReached(const JointValue & target_joint)
  {
    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      if (isReached(target_joint)) {
        return;
      }
      rate.sleep();
    }
  }

  std::string m_robot_ip;
  std::string m_model;
  double m_publish_rate;
  double m_servo_period;
  double m_reach_tolerance_deg;
  bool m_auto_power_on;
  bool m_auto_enable;

  JAKAZuRobot m_robot;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;
  rclcpp::TimerBase::SharedPtr m_joint_state_timer;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr m_action_server;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JakaTrajectoryExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
