#ifndef RPLIDAR_NODE_H
#define RPLIDAR_NODE_H

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rplidar.h"
#include "rcutils/cmdline_parser.h"
#include <math.h>

using namespace rp::standalone::rplidar;

/**
 * This class is a subclass of rclcpp::Node 
 * which contains necessary functions to interface the lidar module "rplidar" with ros2
 */
class RplidarNode : public rclcpp::Node
{
  public:
    RplidarNode(RPlidarDriver* p_drv);

    bool start_motor(std_srvs::srv::Empty::Request &req, std_srvs::srv::Empty::Response &res);

    void start_motor();

    bool stop_motor(std_srvs::srv::Empty::Request &req, std_srvs::srv::Empty::Response &res);

    void test();

    bool bind_lidar_serial_poart();

    u_result getScanResult(const std::string &scan_mode);

    void adjust_max_distance(u_result op_result);

  private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    RplidarScanMode current_scan_mode;
    RPlidarDriver *drv;
    size_t count_;
    std::string serial_port_ = "/dev/ttyUSB0";
    int serial_baudrate_ = 256000;
    int angle_compensate_multiple = 1;
    float max_distance = 8.0;
};

#endif
