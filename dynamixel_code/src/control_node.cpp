// Open terminal #1
// $ ros2 run dynamixel_sdk_examples control_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 service call /set_device bachelor_interfaces/srv/SetDevice "id: 1"

#include <cstdio>
#include <memory>
#include <string>
#include <chrono>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "bachelor_interfaces/srv/set_device.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "control_node.hpp"

// Control table address for AX series
#define ADDR_OPERATING_MODE 8 // 2 byte
#define ADDR_TORQUE_ENABLE 24 // 1 byte
#define ADDR_GOAL_POSITION 30 // 2 byte
#define ADDR_PRESENT_POSITION 36 // 2 byte

// Added by me
#define ADDR_TORQUE_LIMIT 34 // 2 byte
#define ADDR_PRESENT_LOAD 40 // 2 byte

// Protocol version
#define PROTOCOL_VERSION 1.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyACM0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

ControlNode::ControlNode()
: Node("control_node")
{
  RCLCPP_INFO(this->get_logger(), "Run control node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  //const auto QOS_RKL10V =
  //  rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // Set device, make motor to go to high position, then check constantly whether position is met or load is high idicating device hits powerline
  auto set_device =
    [this](
    const std::shared_ptr<SetDevice::Request> request,
    std::shared_ptr<SetDevice::Response> response) -> void
    {
      // Go towards high position
      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,(uint8_t) request->id,ADDR_GOAL_POSITION,213,&dxl_error);

      rclcpp::sleep_for(std::chrono::milliseconds(500));    

      uint16_t load = 0;
      uint16_t position = 0;
      while(1) 
      {
        // read load
        dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_LOAD,
        &load,
        &dxl_error
        );
        // read position
        dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_POSITION,
        &position,
        &dxl_error
        );

        // Check load and position
        if(abs(load) > 250)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            break;
        } else if (position < 215)
        {
            break;
        }
      }

      // Check whether device is in cradle or not
      // set position right above rest, check for device
      packetHandler->write2ByteTxRx(portHandler,(uint8_t) request->id,ADDR_GOAL_POSITION,500,&dxl_error);
      rclcpp::sleep_for(std::chrono::seconds(1));
      packetHandler->read2ByteTxRx(portHandler,(uint8_t) request->id,ADDR_PRESENT_LOAD,&load,&dxl_error);
      // if there false, if not there true and hopefully succesful attachment
      if (load > 140)
      {
        device_status = false;
      } else
      {
        device_status = true;
      }
        
      // resting position
      packetHandler->write2ByteTxRx(portHandler,(uint8_t) request->id,ADDR_GOAL_POSITION,520,&dxl_error);

      // give response based on whether device still there
      RCLCPP_INFO(
        this->get_logger(),
        "Set [ID: %d] [Succes: %d]",
        request->id,
        device_status
      );

      response->check = device_status;
    };

  set_device_server_ = create_service<SetDevice>("set_device", set_device);
}

ControlNode::~ControlNode()
{
}

void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }

  // Set initial motor position
  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_GOAL_POSITION,
    520,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }
  
  // set the initial torque limit
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,dxl_id,ADDR_TORQUE_LIMIT,400,&dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("control_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("control_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("control_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("control_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto controlnode = std::make_shared<ControlNode>();
  rclcpp::spin(controlnode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
