/**
 * luma_link_surface.hpp
 *
 * @date       04/01/2023
 * Copyright (c) 2023 Pablo Gutiérrez
 *
 */

#ifndef LUMA_LINK_SURFACE_HPP
#define LUMA_LINK_SURFACE_HPP


// ROS 2 Headers
#include <chrono>
#include <memory>
#include <unistd.h>
#include "serial/serial.h"
#include <string>
#include <vector>
#include "util_tools.hpp"

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "dvl_msgs/msg/dvl.hpp"
#include "luma_msgs/msg/luma250_lp.hpp"
#include "uuv_msgs/msg/barometer.hpp" 
#include "luma_link/link_protocol.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::string;

namespace luma_link {

class Luma_Link_Surface: public rclcpp::Node
{
public:

    Luma_Link_Surface();
    ~Luma_Link_Surface();


private:

    float axes[4];
    int buttons[12]; 
    int prev_buttons[12]; 
    uint8_t *bytesAxes;
    uint8_t *prev_bytesAxes;
    uint8_t *bytesButtons;
    uint8_t *bytesJoystick;
    //uint8_t *str_crc;
    bool flagReceiveSensorData = false;
    bool flagStopReceiveSensorData = false;
    string serial_port;
    string joy_topic_name;
    //Serial input buffer
    std::vector<uint8_t> inputArray;
    
    bool joystickActive = false;
    uint8_t mode;
    uint8_t link;
    luma_msgs::msg::Luma250LP luma_msgs;
    

    serial::Serial my_serial;
    rclcpp::TimerBase::SharedPtr timer_serial;
    rclcpp::TimerBase::SharedPtr timer_read;
    std::chrono::steady_clock::time_point first_time;
    std::chrono::steady_clock::time_point first_time_handshake;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
    rclcpp::Publisher<dvl_msgs::msg::DVL>::SharedPtr dvl_pub_;
    rclcpp::Publisher<luma_msgs::msg::Luma250LP>::SharedPtr luma_pub_;
    rclcpp::Publisher<uuv_msgs::msg::Barometer>::SharedPtr barometer_pub_;

    // Create a callback group
    rclcpp::CallbackGroup::SharedPtr cb_group;


    void call_joystick(const sensor_msgs::msg::Joy::SharedPtr joy);
    void send_data();
    void read_data();



};

} // namespace

#endif // LUMA_LINK_SURFACE_HPP
