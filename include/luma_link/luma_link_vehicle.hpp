/**
 * luma_link_vehicle.hpp
 *
 * @date       04/01/2023
 * Copyright (c) 2023 Pablo Gutiérrez
 *
 */

#ifndef LUMA_LINK_VEHICLE_HPP
#define LUMA_LINK_VEHICLE_HPP


// ROS 2 Headers
#include <chrono>
#include <memory>
#include <unistd.h>
#include "serial/serial.h"
#include <string>
#include <vector>
#include "util_tools.hpp"
#include "link_protocol.h"

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include "uuv_msgs/msg/barometer.hpp"
#include "dvl_msgs/msg/dvl.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"



using namespace std::chrono_literals;
using std::placeholders::_1;
using std::string;

namespace luma_link {

class Luma_Link_Vehicle: public rclcpp::Node
{
public:

    Luma_Link_Vehicle();
    ~Luma_Link_Vehicle();


private:

    std::vector<uint8_t> inputArray;
    uint8_t *serial_euler_array;
    uint8_t *serial_baro_array;
    uint8_t *serial_dvl_array;
    std::vector<uint8_t> serial_euler;
    int count = 0;
    
    bool joystickActive = false;
    bool syncFlag = false;
    bool sendSensorData = false;
    bool saveROSbag = false;
    string serialCode = "3\n";
    string serial_port;
    util_tools::State state;
    util_tools::imu_t imu_data;
    util_tools::attitude_t euler_data;    
    std_msgs::msg::Bool rosbag_msgs;

    joy_t joy_struct;
    
    /*
    util_tools::imu_t            imu_data;
    util_tools::point_t          mag_data;
    util_tools::attitude_t       euler_data;
    util_tools::barometer_t      barometer_data;

    util_tools::i2c_receive_imu         receive_imu;
    util_tools::i2c_receive_euler       receive_euler;
    util_tools::i2c_receive_data        receive_data;
    util_tools::i2c_receive_bno055      receive_bno0ser55;
    */


    serial::Serial my_serial;
    rclcpp::TimerBase::SharedPtr timer_serial;
    rclcpp::TimerBase::SharedPtr timer_send;
    
    std::chrono::steady_clock::time_point first_time;
    std::chrono::steady_clock::time_point first_time_receive;
    std::chrono::steady_clock::time_point first_time_send;

    //ROSbag publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rosbag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_euler_;
    rclcpp::Subscription<uuv_msgs::msg::Barometer>::SharedPtr sub_barometer_;
    rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr sub_dvl_;

    // Create a callback group
    rclcpp::CallbackGroup::SharedPtr cb_group;


    

    

    void clear_output(sensor_msgs::msg::Joy &joy_msg);
    void receive_data();
    void send_data();
    //Subscribers
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu);
    void euler_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr euler);
    void pressure_callback(const uuv_msgs::msg::Barometer::SharedPtr barometer);
    //void barometer_callback(const rov_msgs::msg::Barometer::SharedPtr barometer);
    void dvl_callback(const dvl_msgs::msg::DVL::SharedPtr dvl);




};

} // namespace

#endif // LUMA_LINK_VEHICLE_HPP
