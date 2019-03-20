// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rplidar_node.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*180. / M_PI)


using namespace std::chrono_literals;


  RplidarNode::RplidarNode(RPlidarDriver* p_drv)
      : Node("rplidarNode"), drv(p_drv)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan");
    timer_ = this->create_wall_timer(
        500ms, std::bind(&RplidarNode::timer_callback, this));
  }


  bool RplidarNode::start_motor(std_srvs::srv::Empty::Request &req, std_srvs::srv::Empty::Response &res)
  {
    if (!drv)
      return false;
    RCLCPP_INFO(this->get_logger(), "Start motor");
    drv->startMotor();
    drv->startScan(0, 1);
    return true;
  }

  

  void RplidarNode::start_motor()
  {
    drv->startMotor();
  }

  bool RplidarNode::stop_motor(std_srvs::srv::Empty::Request &req, std_srvs::srv::Empty::Response &res)
  {
    if (!drv)
      return false;
    RCLCPP_INFO(this->get_logger(), "Stop motor");
    drv->stop();
    drv->stopMotor();
    return true;
  }

  void RplidarNode::test()
  {

    RCLCPP_INFO(this->get_logger(), "Stop motor");
  }

  bool RplidarNode::bind_lidar_serial_poart()
  {

    if (IS_FAIL(drv->connect(this->serial_port_.c_str(), (_u32)this->serial_baudrate_)))
    {
      RCLCPP_INFO(this->get_logger(), "Error, cannot bind to the specified serial port %s.", this->serial_port_.c_str());
      RPlidarDriver::DisposeDriver(drv);
      return -1;
    }
  }

  u_result RplidarNode::getScanResult(const std::string &scan_mode)
  {

    if (scan_mode.empty())
    {
      return drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
    }

    std::vector<RplidarScanMode> allSupportedScanModes;
    u_result op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

    if (not IS_OK(op_result))
    {
      return op_result;
    }

    auto selectedScanModeiter = std::find_if(
        allSupportedScanModes.begin(),
        allSupportedScanModes.end(),
        [&scan_mode](const RplidarScanMode &rpsmode) {
          return rpsmode.scan_mode == scan_mode;
        });

    if (selectedScanModeiter != allSupportedScanModes.end())
    {
      RCLCPP_INFO(this->get_logger(), "scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
      for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++)
      {
        RCLCPP_INFO(this->get_logger(), "\t%s: max_distance: %.1f m, Point number: %.1fK", iter->scan_mode,
                    iter->max_distance, (1000 / iter->us_per_sample));
      }
      return RESULT_OPERATION_FAIL;
    }

    return drv->startScanExpress(false /* not force scan */, selectedScanModeiter->id, 0, &current_scan_mode);
  }

  void RplidarNode::adjust_max_distance(u_result op_result)
  {

    if (IS_OK(op_result))
    {

      //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
      this->angle_compensate_multiple = (int)(1000 * 1000 / current_scan_mode.us_per_sample / 10.0 / 360.0);
      if (angle_compensate_multiple < 1)
        angle_compensate_multiple = 1;
      max_distance = current_scan_mode.max_distance;
      RCLCPP_INFO(this->get_logger(), "current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d", current_scan_mode.scan_mode,
                  current_scan_mode.max_distance, (1000 / current_scan_mode.us_per_sample), angle_compensate_multiple);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Can not start scan: %08x!", op_result);
    }
  }

  void RplidarNode::timer_callback()
  {
    double angle_resolution = 2500;
    double start_angle = -450000;
    double stop_angle = 2250000;
    double scan_frequency = 2500;
    float angle_min = DEG2RAD(0.0f);
    float angle_max = DEG2RAD(359.0f);

    auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    msg->header.frame_id = "laser_frame";

    rplidar_response_measurement_node_hq_t nodes[360 * 8];
    size_t count = _countof(nodes);

    msg->ranges.resize(static_cast<int>(count));
    msg->angle_increment = (msg->angle_max - msg->angle_min) / (double)(count - 1);
    msg->angle_min = angle_min;
    msg->angle_max = angle_max;
    msg->scan_time = rclcpp::Node::now().seconds();
    msg->range_min = 0.15;
    msg->range_max = max_distance;

    RCLCPP_INFO(this->get_logger(), "angle inc:\t%f", msg->angle_increment);
    //RCLCPP_INFO(this->get_logger(), "scan size:\t%zu", msg->ranges.size());
    // RCLCPP_INFO(this->get_logger(), "scan time increment: \t%f", msg->time_increment);

    publisher_->publish(msg);
  }

  int main(int argc, char *argv[])
  {
    auto drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RplidarNode>(drv);
    //RplidarNode node(drv);
    //node.bind_lidar_serial_poart();

    // Star The lidar"s motor
    // auto server = node.create_service<std_srvs::srv::Empty>("start_motor", handle_service); //rplidarNode::handle_service
    node->start_motor();

    u_result op_result = node->getScanResult("scan_mode");

    rclcpp::spin(node);//

    rclcpp::shutdown();

    return 0;
  }