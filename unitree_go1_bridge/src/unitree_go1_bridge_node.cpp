#include <cmath>

#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <string>
#include <algorithm>
#include <array>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <unitree_go1_bridge/utility.hpp>
#include <unitree_go1_bridge/control_communicator.hpp>


namespace unitree_go1_bridge
{
class UnitreeGo1BridgeNode : public rclcpp::Node
{
public:
  explicit UnitreeGo1BridgeNode(const rclcpp::NodeOptions &);
  ~UnitreeGo1BridgeNode();

private:
  static constexpr char m_this_node_name[] = "unitree_go1_bridge_node";

  std::vector<std::string> m_joint_names;
  std::unordered_map <std::string, int> m_joint_map;

  std::unique_ptr<unitree_go1_bridge::ControlCommunicator> m_communicator;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr m_imu_temperature_publisher;
  std::array<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr, 4>
    m_force_sensor_publishers;

  rclcpp::TimerBase::SharedPtr m_bridge_timer;

  void bridgeCallback();

  void publishState(const unitree_go1_bridge::ControlCommunicator::State &);

  void initializeJointNames();
  void initializeJointMap();
};

UnitreeGo1BridgeNode::UnitreeGo1BridgeNode(const rclcpp::NodeOptions &node_options)
: rclcpp::Node(m_this_node_name, node_options)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << m_this_node_name);

  initializeJointNames();
  initializeJointMap();

  m_communicator = std::make_unique<unitree_go1_bridge::ControlCommunicator>();

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

  m_bridge_timer = this->create_wall_timer(
    std::chrono::milliseconds(5),
    ::std::bind(
      &UnitreeGo1BridgeNode::bridgeCallback,
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
  const auto state = m_communicator->receive();
  publishState(state);

  {
    for(unsigned int i = 0; i < 12; ++ i)
    {
      unitree_go1_bridge::ControlCommunicator::MotorCommand motor_command;
      unitree_go1_bridge::utility::resetMotorCommand(motor_command);
      m_communicator->setMotorCommand(motor_command, i);
    }
  }
  m_communicator->send();
}

void UnitreeGo1BridgeNode::publishState(const unitree_go1_bridge::ControlCommunicator::State &state)
{
  const auto current_time_stamp = this->get_clock()->now();

  {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = current_time_stamp;

    joint_state_msg.name.resize(m_joint_names.size());
    std::copy(m_joint_names.cbegin(), m_joint_names.cend(), joint_state_msg.name.begin());

    for(const auto &joint_name : joint_state_msg.name)
    {
      const int joint_index = m_joint_map[joint_name];
      joint_state_msg.position.push_back(
        state.motorState[joint_index].q
      );
      joint_state_msg.velocity.push_back(
        state.motorState[joint_index].dq
      );
      joint_state_msg.effort.push_back(
        state.motorState[joint_index].tauEst
      );
    }
    m_joint_state_publisher->publish(joint_state_msg);
  }
  {
    sensor_msgs::msg::Imu imu_msg;

    //! @todo from parameter
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = current_time_stamp;
    imu_msg.linear_acceleration.x = state.imu.accelerometer[0];
    imu_msg.linear_acceleration.y = state.imu.accelerometer[1];
    imu_msg.linear_acceleration.z = state.imu.accelerometer[2];
    imu_msg.angular_velocity.x = state.imu.gyroscope[0];
    imu_msg.angular_velocity.y = state.imu.gyroscope[1];
    imu_msg.angular_velocity.z = state.imu.gyroscope[2];
    imu_msg.orientation.x = state.imu.quaternion[1];
    imu_msg.orientation.y = state.imu.quaternion[1];
    imu_msg.orientation.z = state.imu.quaternion[1];
    imu_msg.orientation.w = state.imu.quaternion[0];
    m_imu_publisher->publish(imu_msg);
  }
  {
    sensor_msgs::msg::Temperature imu_temperature_msg;

    //! @todo from parameter
    imu_temperature_msg.header.frame_id = "imu_link";
    imu_temperature_msg.header.stamp = current_time_stamp;
    imu_temperature_msg.temperature = state.imu.temperature;
    m_imu_temperature_publisher->publish(imu_temperature_msg);
  }
  //! @todo Calibration and newton convert coefficent
  {
    constexpr double sensor_z_offset_angle = std::sin(60 * M_PI / 180);
    constexpr double sensor_x_offset_angle = std::cos(60 * M_PI / 180);
    std::array<geometry_msgs::msg::Vector3Stamped, 4> force_sensor_msg;
    //! @todo from parameter
    force_sensor_msg[0].header.frame_id = "fr_foot";
    force_sensor_msg[1].header.frame_id = "fl_foot";
    force_sensor_msg[2].header.frame_id = "rr_foot";
    force_sensor_msg[3].header.frame_id = "rl_foot";

    for(auto &&fsm : force_sensor_msg)
    {
      fsm.header.stamp = current_time_stamp;
    }
    for(unsigned int i = 0; i < 4; ++ i)
    {
      force_sensor_msg[i].vector.x = sensor_x_offset_angle * state.footForce[i];
      force_sensor_msg[i].vector.z = sensor_z_offset_angle * state.footForce[i];
    }
    for(unsigned int i = 0; i < m_force_sensor_publishers.size(); ++ i)
    {
      m_force_sensor_publishers[i]->publish(force_sensor_msg[i]);
    }
  }
}

void UnitreeGo1BridgeNode::initializeJointNames()
{
  m_joint_names.clear();
  m_joint_names.push_back("fr_hip_joint");
  m_joint_names.push_back("fr_thigh_joint");
  m_joint_names.push_back("fl_thigh_joint");
  m_joint_names.push_back("fl_calf_joint");
  m_joint_names.push_back("fl_hip_joint");
  m_joint_names.push_back("rr_hip_joint");
  m_joint_names.push_back("rl_hip_joint");
  m_joint_names.push_back("rr_thigh_joint");
  m_joint_names.push_back("rr_calf_joint");
  m_joint_names.push_back("rl_thigh_joint");
  m_joint_names.push_back("fr_calf_joint");
  m_joint_names.push_back("rl_calf_joint");
  m_joint_names.shrink_to_fit();
}

void UnitreeGo1BridgeNode::initializeJointMap()
{
  m_joint_map.clear();

  //! @todo from parameter list
  m_joint_map["fr_hip_joint"] = unitree_go1_bridge::unitree_legged_sdk::FR_0;
  m_joint_map["fr_thigh_joint"] = unitree_go1_bridge::unitree_legged_sdk::FR_1;
  m_joint_map["fr_calf_joint"] = unitree_go1_bridge::unitree_legged_sdk::FR_2;
  m_joint_map["fl_hip_joint"] = unitree_go1_bridge::unitree_legged_sdk::FL_0;
  m_joint_map["fl_thigh_joint"] = unitree_go1_bridge::unitree_legged_sdk::FL_1;
  m_joint_map["fl_calf_joint"] = unitree_go1_bridge::unitree_legged_sdk::FL_2;
  m_joint_map["rr_hip_joint"] = unitree_go1_bridge::unitree_legged_sdk::RR_0;
  m_joint_map["rr_thigh_joint"] = unitree_go1_bridge::unitree_legged_sdk::RR_1;
  m_joint_map["rr_calf_joint"] = unitree_go1_bridge::unitree_legged_sdk::RR_2;
  m_joint_map["rl_hip_joint"] = unitree_go1_bridge::unitree_legged_sdk::RL_0;
  m_joint_map["rl_thigh_joint"] = unitree_go1_bridge::unitree_legged_sdk::RL_1;
  m_joint_map["rl_calf_joint"] = unitree_go1_bridge::unitree_legged_sdk::RL_2;
}
}  // namespace unitree_go1_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_go1_bridge::UnitreeGo1BridgeNode)
