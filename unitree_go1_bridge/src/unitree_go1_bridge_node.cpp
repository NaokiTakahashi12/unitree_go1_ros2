// BSD 3-Clause License
//
// Copyright (c) 2023, NaokiTakahashi
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <cmath>

#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <string>
#include <algorithm>
#include <array>
#include <deque>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <std_srvs/srv/empty.hpp>

#include <unitree_go1_bridge/utility.hpp>
#include <unitree_go1_bridge/control_communicator.hpp>

#include <unitree_go1_bridge_node_parameters.hpp>


namespace unitree_go1_bridge
{
struct UnitreeGo1MotorParam
{
  int motor_id;
  double kp, kd;
  float max_velocity, max_torque;
};

class UnitreeGo1BridgeNode : public rclcpp::Node
{
public:
  explicit UnitreeGo1BridgeNode(const rclcpp::NodeOptions &);
  ~UnitreeGo1BridgeNode();

private:
  static constexpr int m_max_foot_force_size = 4;
  static constexpr char m_this_node_name[] = "unitree_go1_bridge_node";

  bool m_offset_calibrated;

  std::array<float, m_max_foot_force_size> m_offset_force;

  std::deque<std::string> m_joint_names;
  std::unordered_map<std::string, UnitreeGo1MotorParam> m_joint_map;

  std::deque<std::array<float, m_max_foot_force_size>> m_foot_force_average_filter_buffer;

  std::mutex m_joint_trajectory_mutex, m_calibration_mutex;

  std::unique_ptr<unitree_go1_bridge::ControlCommunicator> m_communicator;

  trajectory_msgs::msg::JointTrajectory::ConstSharedPtr m_joint_trajectory;

  std::shared_ptr<unitree_go1_bridge_node::ParamListener> m_param_listener;
  std::unique_ptr<unitree_go1_bridge_node::Params> m_params;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    m_joint_trajectory_subscriber;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr m_imu_temperature_publisher;
  std::array<rclcpp::Publisher<
      geometry_msgs::msg::Vector3Stamped>::SharedPtr, m_max_foot_force_size
  > m_raw_force_sensor_publishers,
    m_force_sensor_publishers;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_do_foot_force_calibration_service;

  rclcpp::TimerBase::SharedPtr
    m_bridge_timer,
    m_sensor_publish_timer;

  void bridgeCallback();
  void sensorPublishCallback();
  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr);

  void calibrateFootForce(
    const std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr
  );

  void publishSensorState(const unitree_go1_bridge::ControlCommunicator::State &);
  void publishJointState(const unitree_go1_bridge::ControlCommunicator::State &);

  void initializeJointNames();
  void initializeJointMap();
};

UnitreeGo1BridgeNode::UnitreeGo1BridgeNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node(m_this_node_name, node_options),
  m_offset_calibrated(false),
  m_offset_force({0})
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << m_this_node_name);

  m_param_listener = std::make_shared<unitree_go1_bridge_node::ParamListener>(
    this->get_node_parameters_interface()
  );
  m_params = std::make_unique<unitree_go1_bridge_node::Params>(
    m_param_listener->get_params()
  );

  initializeJointNames();
  initializeJointMap();

  m_communicator = std::make_unique<unitree_go1_bridge::ControlCommunicator>();

  m_joint_trajectory_subscriber = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory",
    rclcpp::QoS(5),
    ::std::bind(
      &UnitreeGo1BridgeNode::jointTrajectoryCallback,
      this,
      ::std::placeholders::_1
    )
  );

  m_joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>(
    "~/joint_states",
    rclcpp::QoS(5)
  );
  m_imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>(
    "~/imu",
    rclcpp::QoS(5)
  );
  m_imu_temperature_publisher = this->create_publisher<sensor_msgs::msg::Temperature>(
    "~/imu/temperature",
    rclcpp::QoS(5)
  );

  m_raw_force_sensor_publishers[0] = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "~/force/fr/raw",
    rclcpp::QoS(5)
  );
  m_raw_force_sensor_publishers[1] = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "~/force/fl/raw",
    rclcpp::QoS(5)
  );
  m_raw_force_sensor_publishers[2] = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "~/force/rr/raw",
    rclcpp::QoS(5)
  );
  m_raw_force_sensor_publishers[3] = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "~/force/rl/raw",
    rclcpp::QoS(5)
  );
  m_force_sensor_publishers[0] = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "~/force/fr",
    rclcpp::QoS(5)
  );
  m_force_sensor_publishers[1] = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "~/force/fl",
    rclcpp::QoS(5)
  );
  m_force_sensor_publishers[2] = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "~/force/rr",
    rclcpp::QoS(5)
  );
  m_force_sensor_publishers[3] = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "~/force/rl",
    rclcpp::QoS(5)
  );

  m_do_foot_force_calibration_service = this->create_service<std_srvs::srv::Empty>(
    "~/calibrate_foot_force",
    ::std::bind(
      &UnitreeGo1BridgeNode::calibrateFootForce,
      this,
      ::std::placeholders::_1,
      ::std::placeholders::_2
    )
  );
  const unsigned int bridge_timer_milliseconds = 1e3 / m_params->bridge_frequency;
  const unsigned int sensor_publish_timer_milliseconds = 1e3 / m_params->sensor_publish_frequency;

  if (bridge_timer_milliseconds >= sensor_publish_timer_milliseconds) {
    std::string
      error_message{"Please set the sensor publish frequency lower than the bridge frequency"};
    RCLCPP_ERROR(this->get_logger(), error_message.c_str());
    throw std::runtime_error(error_message);
  }
  m_bridge_timer = this->create_wall_timer(
    std::chrono::milliseconds(
      bridge_timer_milliseconds
    ),
    ::std::bind(
      &UnitreeGo1BridgeNode::bridgeCallback,
      this
    )
  );
  m_sensor_publish_timer = this->create_wall_timer(
    std::chrono::milliseconds(
      sensor_publish_timer_milliseconds
    ),
    ::std::bind(
      &UnitreeGo1BridgeNode::sensorPublishCallback,
      this
    )
  );
}

UnitreeGo1BridgeNode::~UnitreeGo1BridgeNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << m_this_node_name);
}

void UnitreeGo1BridgeNode::bridgeCallback()
{
  std::lock_guard<std::mutex> calibration_lock{m_calibration_mutex};

  const auto state = m_communicator->receive();
  publishJointState(state);
  {
    std::lock_guard<std::mutex> trajectory_lock{m_joint_trajectory_mutex};
    unsigned int joint_trajectory_count = 0;

    if (not m_joint_trajectory) {
      // Because the state is not updated without sending some data
      //
      // m_communicator will not work even if it is sent because the position
      // command and speed command are invalid after initialization.
      m_communicator->send();
      return;
    }
    if (m_joint_trajectory->joint_names.size() != m_joint_trajectory->points.size()) {
      RCLCPP_WARN(this->get_logger(), "Different size of joint trajectory joint_names vs points");
      return;
    }
    for (const auto & joint_name : m_joint_trajectory->joint_names) {
      if (m_joint_map.count(joint_name) != 1) {
        RCLCPP_WARN(this->get_logger(), "Not found joint trajectory point name");
        joint_trajectory_count++;
        continue;
      }
      unitree_go1_bridge::ControlCommunicator::MotorCommand motor_command;
      unitree_go1_bridge::utility::resetMotorCommand(motor_command);

      decltype(auto) joint_config = m_joint_map[joint_name];

      if (m_joint_trajectory->points[joint_trajectory_count].positions.size() > 0) {
        motor_command.q = m_joint_trajectory->points[joint_trajectory_count].positions[0];
      }
      if (m_joint_trajectory->points[joint_trajectory_count].velocities.size() > 0) {
        motor_command.dq = m_joint_trajectory->points[joint_trajectory_count].velocities[0];
      }
      if (m_joint_trajectory->points[joint_trajectory_count].effort.size() > 0) {
        motor_command.tau = m_joint_trajectory->points[joint_trajectory_count].effort[0];
      }
      std::clamp(
        motor_command.dq,
        -joint_config.max_velocity,
        joint_config.max_velocity
      );
      std::clamp(
        motor_command.tau,
        -joint_config.max_torque,
        joint_config.max_torque
      );
      motor_command.mode = 0x0A;
      motor_command.Kp = joint_config.kp;
      motor_command.Kd = joint_config.kd;
      m_communicator->setMotorCommand(motor_command, joint_config.motor_id);
      joint_trajectory_count++;
    }
  }
  m_communicator->send();
}

void UnitreeGo1BridgeNode::sensorPublishCallback()
{
  std::lock_guard<std::mutex> calibration_lock{m_calibration_mutex};
  const auto state = m_communicator->getLatestState();
  publishSensorState(state);
}

void UnitreeGo1BridgeNode::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory_msg)
{
  std::lock_guard<std::mutex> lock{m_joint_trajectory_mutex};
  m_joint_trajectory = joint_trajectory_msg;
}

void UnitreeGo1BridgeNode::calibrateFootForce(
  const std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr
)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Called do calibrate foot force");
  const unsigned int wait_milliseconds = 1e3 / m_params->bridge_frequency;
  m_offset_calibrated = false;

  unsigned int count_samples = 0;
  std::array<int16_t, m_max_foot_force_size> sum_foot_forces{0};

  for (
    count_samples = 0;
    count_samples < m_params->offset_calibration_samples;
    ++count_samples
  )
  {
    std::this_thread::sleep_for(
      std::chrono::milliseconds(wait_milliseconds)
    );
    {
      std::lock_guard<std::mutex> calibration_lock{m_calibration_mutex};
      const auto state = m_communicator->getLatestState();
      for (unsigned int i = 0; i < m_max_foot_force_size; ++i) {
        sum_foot_forces[i] += state.footForce[i];
      }
    }
  }
  for (int i = 0; i < m_max_foot_force_size; ++i) {
    m_offset_force[i] = -static_cast<float>(sum_foot_forces[i]) / static_cast<float>(count_samples);
  }
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Offset force is: "
      << m_offset_force[0] << ", "
      << m_offset_force[1] << ", "
      << m_offset_force[2] << ", "
      << m_offset_force[3]
  );
  m_offset_calibrated = true;
}

void UnitreeGo1BridgeNode::publishJointState(
  const unitree_go1_bridge::ControlCommunicator::State & state)
{
  const auto current_time_stamp = this->get_clock()->now();
  auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state_msg->header.stamp = current_time_stamp;

  joint_state_msg->name.resize(m_joint_names.size());
  std::copy(m_joint_names.cbegin(), m_joint_names.cend(), joint_state_msg->name.begin());

  for (const auto & joint_name : joint_state_msg->name) {
    const int joint_index = m_joint_map[joint_name].motor_id;
    joint_state_msg->position.push_back(
      state.motorState[joint_index].q
    );
    joint_state_msg->velocity.push_back(
      state.motorState[joint_index].dq
    );
    joint_state_msg->effort.push_back(
      state.motorState[joint_index].tauEst
    );
  }
  m_joint_state_publisher->publish(std::move(joint_state_msg));
}

void UnitreeGo1BridgeNode::publishSensorState(
  const unitree_go1_bridge::ControlCommunicator::State & state)
{
  const auto current_time_stamp = this->get_clock()->now();
  {
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

    imu_msg->header.frame_id = m_params->imu_frame_id;
    imu_msg->header.stamp = current_time_stamp;
    imu_msg->linear_acceleration.x = state.imu.accelerometer[0];
    imu_msg->linear_acceleration.y = state.imu.accelerometer[1];
    imu_msg->linear_acceleration.z = state.imu.accelerometer[2];
    imu_msg->angular_velocity.x = state.imu.gyroscope[0];
    imu_msg->angular_velocity.y = state.imu.gyroscope[1];
    imu_msg->angular_velocity.z = state.imu.gyroscope[2];
    imu_msg->orientation.x = state.imu.quaternion[1];
    imu_msg->orientation.y = state.imu.quaternion[1];
    imu_msg->orientation.z = state.imu.quaternion[1];
    imu_msg->orientation.w = state.imu.quaternion[0];
    m_imu_publisher->publish(std::move(imu_msg));
  }
  {
    auto imu_temperature_msg = std::make_unique<sensor_msgs::msg::Temperature>();

    imu_temperature_msg->header.frame_id = m_params->imu_frame_id;
    imu_temperature_msg->header.stamp = current_time_stamp;
    imu_temperature_msg->temperature = state.imu.temperature;
    m_imu_temperature_publisher->publish(std::move(imu_temperature_msg));
  }
  {
    const double sensor_z_offset_angle = std::sin(m_params->force_sensor_offset_angles.pitch);
    const double sensor_x_offset_angle = std::cos(m_params->force_sensor_offset_angles.pitch);

    std::array<float, m_max_foot_force_size> average_foot_force{0};

    std::array<
      geometry_msgs::msg::Vector3Stamped::UniquePtr,
      m_max_foot_force_size
    > force_sensor_msgs,
      offset_calibrated_force_sensor_msgs;

    for (auto && fsm : force_sensor_msgs) {
      fsm = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
    }
    for (auto && ocfsm : offset_calibrated_force_sensor_msgs) {
      ocfsm = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
    }

    force_sensor_msgs[0]->header.frame_id = m_params->foot_force_sensor_frame_id.fr;
    force_sensor_msgs[1]->header.frame_id = m_params->foot_force_sensor_frame_id.fl;
    force_sensor_msgs[2]->header.frame_id = m_params->foot_force_sensor_frame_id.rr;
    force_sensor_msgs[3]->header.frame_id = m_params->foot_force_sensor_frame_id.rl;

    for (auto && fsm : force_sensor_msgs) {
      fsm->header.stamp = current_time_stamp;
    }
    for (unsigned int i = 0; i < m_max_foot_force_size; ++i) {
      const float foot_force = m_params->foot_force_coefficient * state.footForce[i];
      force_sensor_msgs[i]->vector.x = sensor_x_offset_angle * foot_force;
      force_sensor_msgs[i]->vector.z = sensor_z_offset_angle * foot_force;
    }

    for (unsigned int i = 0; i < m_raw_force_sensor_publishers.size(); ++i) {
      offset_calibrated_force_sensor_msgs[i]->header =
        force_sensor_msgs[i]->header;
    }
    for (unsigned int i = 0; i < m_raw_force_sensor_publishers.size(); ++i) {
      m_raw_force_sensor_publishers[i]->publish(
        std::move(force_sensor_msgs[i])
      );
    }
    if (!m_offset_calibrated) {
      return;
    }
    m_foot_force_average_filter_buffer.emplace_back(std::array<float, m_max_foot_force_size>{});

    for (unsigned int i = 0; i < m_max_foot_force_size; ++i) {
      m_foot_force_average_filter_buffer.back()[i] = state.footForce[i];
    }
    if (
      m_foot_force_average_filter_buffer.size() >
      static_cast<unsigned int>(m_params->foot_force_average_filter.history_length))
    {
      m_foot_force_average_filter_buffer.pop_front();
    }
    for (unsigned int i = 0; i < m_max_foot_force_size; ++i) {
      float sum = 0;
      for (unsigned int j = 0; j < m_foot_force_average_filter_buffer.size(); ++j) {
        sum += m_foot_force_average_filter_buffer[j][i];
      }
      average_foot_force[i] = sum / m_foot_force_average_filter_buffer.size();
    }
    for (unsigned int i = 0; i < m_max_foot_force_size; ++i) {
      const float foot_force = m_params->foot_force_coefficient *
        (average_foot_force[i] + m_offset_force[i]);
      offset_calibrated_force_sensor_msgs[i]->vector.x = sensor_x_offset_angle * foot_force;
      offset_calibrated_force_sensor_msgs[i]->vector.z = sensor_z_offset_angle * foot_force;
    }
    for (unsigned int i = 0; i < m_force_sensor_publishers.size(); ++i) {
      m_force_sensor_publishers[i]->publish(
        std::move(offset_calibrated_force_sensor_msgs[i])
      );
    }
  }
}

void UnitreeGo1BridgeNode::initializeJointNames()
{
  if (!m_params) {
    throw std::runtime_error("Failed access m_params");
  }
  m_joint_names.clear();
  m_joint_names.push_back(m_params->joint_configs.fr_hip.name);
  m_joint_names.push_back(m_params->joint_configs.fr_thigh.name);
  m_joint_names.push_back(m_params->joint_configs.fr_calf.name);
  m_joint_names.push_back(m_params->joint_configs.fl_hip.name);
  m_joint_names.push_back(m_params->joint_configs.fl_thigh.name);
  m_joint_names.push_back(m_params->joint_configs.fl_calf.name);
  m_joint_names.push_back(m_params->joint_configs.rr_hip.name);
  m_joint_names.push_back(m_params->joint_configs.rr_thigh.name);
  m_joint_names.push_back(m_params->joint_configs.rr_calf.name);
  m_joint_names.push_back(m_params->joint_configs.rl_hip.name);
  m_joint_names.push_back(m_params->joint_configs.rl_thigh.name);
  m_joint_names.push_back(m_params->joint_configs.rl_calf.name);
  m_joint_names.shrink_to_fit();
}

template<typename T>
void setUnitreeGo1MotorParamFromGeneratedParam(
  UnitreeGo1MotorParam & param, const int id,
  const T & joint_config_param)
{
  param.motor_id = id;
  param.kp = joint_config_param.kp;
  param.kd = joint_config_param.kd;
  param.max_torque = joint_config_param.max_torque;
  param.max_velocity = joint_config_param.max_velocity;
}

void UnitreeGo1BridgeNode::initializeJointMap()
{
  if (!m_params) {
    throw std::runtime_error("Failed access m_params");
  }
  m_joint_map.clear();
  setUnitreeGo1MotorParamFromGeneratedParam(
    m_joint_map[m_params->joint_configs.fr_hip.name],
    unitree_go1_bridge::unitree_legged_sdk::FR_0,
    m_params->joint_configs.fr_hip
  );
  setUnitreeGo1MotorParamFromGeneratedParam(
    m_joint_map[m_params->joint_configs.fr_thigh.name],
    unitree_go1_bridge::unitree_legged_sdk::FR_1,
    m_params->joint_configs.fr_thigh
  );
  setUnitreeGo1MotorParamFromGeneratedParam(
    m_joint_map[m_params->joint_configs.fr_calf.name],
    unitree_go1_bridge::unitree_legged_sdk::FR_2,
    m_params->joint_configs.fr_calf
  );
  setUnitreeGo1MotorParamFromGeneratedParam(
    m_joint_map[m_params->joint_configs.fl_hip.name],
    unitree_go1_bridge::unitree_legged_sdk::FL_0,
    m_params->joint_configs.fl_hip
  );
  setUnitreeGo1MotorParamFromGeneratedParam(
    m_joint_map[m_params->joint_configs.fl_thigh.name],
    unitree_go1_bridge::unitree_legged_sdk::FL_1,
    m_params->joint_configs.fl_thigh
  );
  setUnitreeGo1MotorParamFromGeneratedParam(
    m_joint_map[m_params->joint_configs.fl_calf.name],
    unitree_go1_bridge::unitree_legged_sdk::FL_2,
    m_params->joint_configs.fl_calf
  );
  setUnitreeGo1MotorParamFromGeneratedParam(
    m_joint_map[m_params->joint_configs.rr_hip.name],
    unitree_go1_bridge::unitree_legged_sdk::RR_0,
    m_params->joint_configs.rr_hip
  );
  setUnitreeGo1MotorParamFromGeneratedParam(
    m_joint_map[m_params->joint_configs.rr_thigh.name],
    unitree_go1_bridge::unitree_legged_sdk::RR_1,
    m_params->joint_configs.rr_thigh
  );
  setUnitreeGo1MotorParamFromGeneratedParam(
    m_joint_map[m_params->joint_configs.rr_calf.name],
    unitree_go1_bridge::unitree_legged_sdk::RR_2,
    m_params->joint_configs.rr_calf
  );
  setUnitreeGo1MotorParamFromGeneratedParam(
    m_joint_map[m_params->joint_configs.rl_hip.name],
    unitree_go1_bridge::unitree_legged_sdk::RL_0,
    m_params->joint_configs.rl_hip
  );
  setUnitreeGo1MotorParamFromGeneratedParam(
    m_joint_map[m_params->joint_configs.rl_thigh.name],
    unitree_go1_bridge::unitree_legged_sdk::RL_1,
    m_params->joint_configs.rl_thigh
  );
  setUnitreeGo1MotorParamFromGeneratedParam(
    m_joint_map[m_params->joint_configs.rl_calf.name],
    unitree_go1_bridge::unitree_legged_sdk::RL_2,
    m_params->joint_configs.rl_calf
  );
}
}  // namespace unitree_go1_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_go1_bridge::UnitreeGo1BridgeNode)
