#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "bachelor_interfaces/srv/set_device.hpp"


class ControlNode : public rclcpp::Node
{
public:
  using SetDevice = bachelor_interfaces::srv::SetDevice;


  ControlNode();
  virtual ~ControlNode();

private:
  rclcpp::Service<SetDevice>::SharedPtr set_device_server_;

  bool device_status;
};

#endif  // CONTROL_NODE_HPP_
