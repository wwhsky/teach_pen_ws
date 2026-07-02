#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "jaka_planner/JAKAZuRobot.h"

class JakaTrajectoryExecutorNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  JakaTrajectoryExecutorNode()
  : Node("jaka_trajectory_executor_node")
  {
    m_robot_ip = declare_parameter<std::string>("ip", "192.168.1.100");
    m_model = declare_parameter<std::string>("model", "zu5");
    m_publish_rate = declare_parameter<double>("joint_state_publish_rate", 125.0);
    m_servo_period = declare_parameter<double>("servo_period", 0.008);
    m_servo_send_period = declare_parameter<double>("servo_send_period", 0.004);
    m_servo_step_num = declare_parameter<int>("servo_step_num", 1);
    m_servo_queue_control = declare_parameter<bool>("servo_queue_control", true);
    m_servo_queue_low = declare_parameter<int>("servo_queue_low", 3);
    m_servo_queue_high = declare_parameter<int>("servo_queue_high", 10);
    m_servo_queue_hard_limit = declare_parameter<int>("servo_queue_hard_limit", 80);
    m_reach_tolerance_deg = declare_parameter<double>("reach_tolerance_deg", 0.2);
    m_auto_power_on = declare_parameter<bool>("auto_power_on", true);
    m_auto_enable = declare_parameter<bool>("auto_enable", true);
    m_debug_servo_timing = declare_parameter<bool>("debug_servo_timing", false);

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
    // m_robot.servo_move_use_joint_LPF(1);
    m_robot.servo_speed_foresight(10, 0.5);

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

    setEmptyPayload();
    logCurrentPayload();
  }

  void setEmptyPayload()
  {
    PayLoad payload{};
    payload.mass = 0.0;
    payload.centroid.x = 0.0;
    payload.centroid.y = 0.0;
    payload.centroid.z = 0.0;
    payload.payload_id = 0;

    const int result = m_robot.set_payload(&payload);
    RCLCPP_INFO(get_logger(), "JAKA set_payload empty result: %d", result);
  }

  void logCurrentPayload()
  {
    PayLoad payload{};
    const int result = m_robot.get_payload(&payload);
    if (result != 0) {
      RCLCPP_WARN(get_logger(), "JAKA get_payload failed: %d", result);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "JAKA current payload: mass=%.3f kg centroid=[%.3f, %.3f, %.3f] mm id=%d",
      payload.mass,
      payload.centroid.x,
      payload.centroid.y,
      payload.centroid.z,
      payload.payload_id);
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
    const auto points = resampleTrajectory(trajectory, m_servo_period);

    if (points.empty()) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      goal_handle->abort(result);
      return;
    }

    if (!ensureRobotReadyForExecution()) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      goal_handle->abort(result);
      return;
    }

    const int servo_enable_result = m_robot.servo_move_enable(true, false);
    if (servo_enable_result != 0) {
      RCLCPP_ERROR(get_logger(), "servo_move_enable(true) failed: %d", servo_enable_result);
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      goal_handle->abort(result);
      return;
    }

    JointValue target_joint{};
    const auto period_duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(m_servo_send_period));
    const unsigned int servo_step_num = static_cast<unsigned int>(std::max(1, m_servo_step_num));
    const double command_period = static_cast<double>(servo_step_num) * 0.008;
    RCLCPP_INFO(
      get_logger(),
      "servo command period %.4f sec, send period %.4f sec, sdk step_num=%u, queue_control=%s range=[%d,%d]",
      m_servo_period, m_servo_send_period, servo_step_num,
      m_servo_queue_control ? "true" : "false",
      m_servo_queue_low, m_servo_queue_high);
    auto next_send_time = std::chrono::steady_clock::now();
    auto previous_loop_start = std::chrono::steady_clock::now();
    double min_actual_period = std::numeric_limits<double>::infinity();
    double max_actual_period = 0.0;
    double sum_actual_period = 0.0;
    std::size_t period_sample_count = 0;
    std::size_t late_period_count = 0;
    int min_queue_num = std::numeric_limits<int>::max();
    int max_queue_num = 0;
    long long sum_queue_num = 0;
    std::size_t queue_sample_count = 0;

    for (std::size_t i = 0; i < points.size(); ++i) {
      std::this_thread::sleep_until(next_send_time);
      next_send_time += period_duration;

      const auto loop_start = std::chrono::steady_clock::now();
      if (i > 0) {
        const double actual_period =
          std::chrono::duration<double>(loop_start - previous_loop_start).count();
        min_actual_period = std::min(min_actual_period, actual_period);
        max_actual_period = std::max(max_actual_period, actual_period);
        sum_actual_period += actual_period;
        ++period_sample_count;

        if (actual_period > m_servo_send_period * 1.5) {
          ++late_period_count;
          if (m_debug_servo_timing) {
            RCLCPP_WARN(
              get_logger(),
              "servo timing late: point=%zu actual=%.6f sec expected=%.6f sec",
              i, actual_period, m_servo_send_period);
          }
        } else if (m_debug_servo_timing && i % 100 == 0) {
          RCLCPP_INFO(
            get_logger(),
            "servo timing: point=%zu actual=%.6f sec expected=%.6f sec",
            i, actual_period, m_servo_send_period);
        }
      }
      previous_loop_start = loop_start;

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

      int queue_num = 0;
      const int servo_result = m_robot.servo_j(&target_joint, MoveMode::ABS, servo_step_num, &queue_num);
      if (servo_result != 0) {
        RCLCPP_ERROR(get_logger(), "servo_j failed at point %zu: %d", i, servo_result);
        m_robot.servo_move_enable(false);
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        goal_handle->abort(result);
        return;
      }

      min_queue_num = std::min(min_queue_num, queue_num);
      max_queue_num = std::max(max_queue_num, queue_num);
      sum_queue_num += queue_num;
      ++queue_sample_count;

      if (m_debug_servo_timing && i % 100 == 0) {
        RCLCPP_INFO(get_logger(), "servo queue: point=%zu queue=%d", i, queue_num);
      }

      if (m_servo_queue_control) {
        if (queue_num >= m_servo_queue_hard_limit) {
          RCLCPP_WARN(
            get_logger(),
            "servo queue near limit: point=%zu queue=%d hard_limit=%d",
            i, queue_num, m_servo_queue_hard_limit);
        }

        if (queue_num > m_servo_queue_high) {
          const int excess_queue = std::max(1, queue_num - m_servo_queue_high);
          const double drain_sec = std::min(
            command_period,
            static_cast<double>(excess_queue) * m_servo_send_period);
          if (m_debug_servo_timing) {
            RCLCPP_INFO(
              get_logger(),
              "servo queue throttle: point=%zu queue=%d sleep=%.4f sec",
              i, queue_num, drain_sec);
          }
          std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(drain_sec)));
          next_send_time = std::chrono::steady_clock::now() + period_duration;
        }
      }
    }

    if (!waitUntilReached(target_joint, goal_handle)) {
      m_robot.motion_abort();
      m_robot.servo_move_enable(false);
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      goal_handle->canceled(result);
      return;
    }
    m_robot.servo_move_enable(false);

    if (m_debug_servo_timing && period_sample_count > 0) {
      RCLCPP_INFO(
        get_logger(),
        "servo timing summary: samples=%zu min=%.6f sec avg=%.6f sec max=%.6f sec late=%zu expected=%.6f sec",
        period_sample_count,
        min_actual_period,
        sum_actual_period / static_cast<double>(period_sample_count),
        max_actual_period,
        late_period_count,
        m_servo_send_period);
    }
    if (m_debug_servo_timing && queue_sample_count > 0) {
      RCLCPP_INFO(
        get_logger(),
        "servo queue summary: samples=%zu min=%d avg=%.2f max=%d target=[%d,%d]",
        queue_sample_count,
        min_queue_num,
        static_cast<double>(sum_queue_num) / static_cast<double>(queue_sample_count),
        max_queue_num,
        m_servo_queue_low,
        m_servo_queue_high);
    }

    auto result = std::make_shared<FollowJointTrajectory::Result>();
    goal_handle->succeed(result);
  }

  static double pointTimeSec(const trajectory_msgs::msg::JointTrajectoryPoint & point)
  {
    return static_cast<double>(point.time_from_start.sec) +
      static_cast<double>(point.time_from_start.nanosec) * 1e-9;
  }

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> resampleTrajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    double sample_period) const
  {
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> output;
    const auto & input = trajectory.points;
    if (input.empty() || sample_period <= 0.0) {
      return output;
    }
    if (input.size() == 1) {
      return input;
    }

    const double start_time = pointTimeSec(input.front());
    const double end_time = pointTimeSec(input.back());
    if (end_time <= start_time) {
      return input;
    }

    std::size_t segment_index = 1;
    for (double sample_time = start_time; sample_time < end_time; sample_time += sample_period) {
      while (segment_index + 1 < input.size() && pointTimeSec(input[segment_index]) < sample_time) {
        ++segment_index;
      }

      const auto & previous = input[segment_index - 1];
      const auto & next = input[segment_index];
      if (previous.positions.size() < 6 || next.positions.size() < 6) {
        RCLCPP_ERROR(
          get_logger(),
          "Cannot resample trajectory: point positions size is %zu and %zu",
          previous.positions.size(), next.positions.size());
        return {};
      }

      const double previous_time = pointTimeSec(previous);
      const double next_time = pointTimeSec(next);
      const double duration = next_time - previous_time;
      const double ratio = duration > 0.0 ? (sample_time - previous_time) / duration : 0.0;

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.resize(6);
      for (std::size_t joint_index = 0; joint_index < 6; ++joint_index) {
        point.positions[joint_index] = previous.positions[joint_index] +
          ratio * (next.positions[joint_index] - previous.positions[joint_index]);
      }

      const auto nanoseconds = static_cast<int64_t>(std::llround(sample_time * 1e9));
      point.time_from_start.sec = static_cast<int32_t>(nanoseconds / 1000000000);
      point.time_from_start.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000);
      output.push_back(point);
    }

    output.push_back(input.back());
    RCLCPP_INFO(
      get_logger(),
      "resampled trajectory: %zu -> %zu points at %.4f sec",
      input.size(), output.size(), sample_period);
    return output;
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

  bool waitUntilReached(
    const JointValue & target_joint,
    const std::shared_ptr<GoalHandleFollowJointTrajectory> & goal_handle)
  {
    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        return false;
      }
      if (isReached(target_joint)) {
        return true;
      }
      rate.sleep();
    }
    return false;
  }

  bool ensureRobotReadyForExecution()
  {
    RobotStatus status{};
    const int status_result = m_robot.get_robot_status(&status);
    if (status_result == 0) {
      RCLCPP_INFO(
        get_logger(),
        "JAKA status before execute: err=%d powered=%d enabled=%d protective_stop=%d emergency_stop=%d",
        status.errcode, status.powered_on, status.enabled,
        status.protective_stop, status.emergency_stop);

      if (status.emergency_stop != 0) {
        RCLCPP_ERROR(get_logger(), "JAKA emergency stop is active; cannot execute trajectory");
        return false;
      }

      if (status.protective_stop != 0) {
        const int recover_result = m_robot.collision_recover();
        RCLCPP_WARN(get_logger(), "JAKA collision_recover result: %d", recover_result);
      }

      if (status.errcode != 0) {
        const int clear_result = m_robot.clear_error();
        RCLCPP_WARN(get_logger(), "JAKA clear_error result: %d", clear_result);
      }
    } else {
      RCLCPP_WARN(get_logger(), "get_robot_status failed before execute: %d", status_result);
    }

    RobotStatus_simple simple_status{};
    const int simple_result = m_robot.get_robot_status_simple(&simple_status);
    if (simple_result != 0) {
      RCLCPP_ERROR(get_logger(), "get_robot_status_simple failed before execute: %d", simple_result);
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "JAKA simple status before execute: err=%d powered=%d enabled=%d msg=%s",
      simple_status.errcode, simple_status.powered_on, simple_status.enabled,
      simple_status.errmsg);

    if (simple_status.errcode != 0) {
      const int clear_result = m_robot.clear_error();
      RCLCPP_WARN(get_logger(), "JAKA clear_error result: %d", clear_result);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    if (simple_status.powered_on == 0) {
      if (!m_auto_power_on) {
        RCLCPP_ERROR(get_logger(), "JAKA is powered off and auto_power_on=false");
        return false;
      }
      const int power_result = m_robot.power_on();
      RCLCPP_WARN(get_logger(), "JAKA power_on before execute result: %d", power_result);
      rclcpp::sleep_for(std::chrono::seconds(8));
    }

    if (simple_status.enabled == 0) {
      if (!m_auto_enable) {
        RCLCPP_ERROR(get_logger(), "JAKA is disabled and auto_enable=false");
        return false;
      }
      const int enable_result = m_robot.enable_robot();
      RCLCPP_WARN(get_logger(), "JAKA enable_robot before execute result: %d", enable_result);
      rclcpp::sleep_for(std::chrono::seconds(4));
    }

    RobotStatus_simple final_status{};
    const int final_result = m_robot.get_robot_status_simple(&final_status);
    if (final_result != 0) {
      RCLCPP_ERROR(get_logger(), "final get_robot_status_simple failed before execute: %d", final_result);
      return false;
    }

    if (final_status.errcode != 0 || final_status.powered_on == 0 || final_status.enabled == 0) {
      RCLCPP_ERROR(
        get_logger(),
        "JAKA not ready after recovery: err=%d powered=%d enabled=%d msg=%s",
        final_status.errcode, final_status.powered_on, final_status.enabled,
        final_status.errmsg);
      return false;
    }

    return true;
  }

  std::string m_robot_ip;
  std::string m_model;
  double m_publish_rate;
  double m_servo_period;
  double m_servo_send_period;
  int m_servo_step_num;
  bool m_servo_queue_control;
  int m_servo_queue_low;
  int m_servo_queue_high;
  int m_servo_queue_hard_limit;
  double m_reach_tolerance_deg;
  bool m_auto_power_on;
  bool m_auto_enable;
  bool m_debug_servo_timing;

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
