/**
 * luma_link_surface.cpp
 * @date       04/01/2023
 * Copyright (c) 2023 Pablo Gutiérrez
 *
 */

#include "luma_link/luma_link_surface.hpp"
#include <string>


namespace luma_link {


Luma_Link_Surface::Luma_Link_Surface():
Node("luma_link_surface_node")
{
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("joy_topic_name", "joy");
    
    joy_topic_name = this->get_parameter("joy_topic_name").as_string();
    serial_port = this->get_parameter("serial_port").as_string();
    RCLCPP_INFO(get_logger(), "SERIAL_PORT: '%s'", serial_port.c_str());

    bytesAxes = new uint8_t[4];
    prev_bytesAxes = new uint8_t[4];
    bytesButtons = new uint8_t[2];
    
    bytesJoystick = new uint8_t[12];
    //str_crc = new uint8_t[3];
    

    first_time = std::chrono::steady_clock::now();
    first_time_handshake = first_time;

    // Create a callback group
    cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    //ROS2 Custom qos
    auto qos_ = rclcpp::QoS(1);
    qos_.best_effort();
    qos_.keep_last(1);
    
    //ROS2 Publishers QoS
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    auto qos_sensors = rclcpp::QoS(
        rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth
            ),
        qos_profile);

    // Setup subscriber options
    auto rosSubOptions = rclcpp::SubscriptionOptions();
    rosSubOptions.callback_group = cb_group;
    
    sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic_name, 10, std::bind(&Luma_Link_Surface::call_joystick, this, _1));
    timer_serial = this->create_wall_timer(std::chrono::milliseconds(40),std::bind(&Luma_Link_Surface::send_data, this)); 
    timer_read = this->create_wall_timer(std::chrono::milliseconds(20),std::bind(&Luma_Link_Surface::read_data, this));

    euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("luma250/euler/data", qos_sensors); 
    dvl_pub_ = this->create_publisher<dvl_msgs::msg::DVL>("luma250/dvl/data", qos_sensors); 
    luma_pub_ = this->create_publisher<luma_msgs::msg::Luma250LP>("luma250/status", qos_sensors);
    barometer_pub_ = this->create_publisher<uuv_msgs::msg::Barometer>("luma250/barometer/data", qos_sensors);


    try
    {
        my_serial.setPort(serial_port);
        my_serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(50);
        my_serial.setTimeout(to);
        my_serial.open();
        RCLCPP_INFO(this->get_logger(), "Communication with Luma established");
        RCLCPP_INFO(this->get_logger(), "Serial port is open");
        luma_msgs.mode = luma_msgs::msg::Luma250LP::ENABLED;
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port");
        luma_msgs.mode = luma_msgs::msg::Luma250LP::DISABLED;
    }
  
    my_serial.flush();
    
    for(int i=0; i<4; i++)
        prev_bytesAxes[i] = (uint8_t)(100);
            
    for(int i=0; i<10; i++)
        prev_buttons[i] = 0;
    usleep(2000);

}

Luma_Link_Surface::~Luma_Link_Surface() {
    delete [] bytesAxes;
    delete [] prev_bytesAxes;
    delete [] bytesButtons;
  
    my_serial.flush();
    my_serial.close();
}


void Luma_Link_Surface::call_joystick(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    bytesJoystick[0] = 'w';
    bytesJoystick[1] = 'r';
    bytesJoystick[2] = 'j';
    bytesJoystick[3] = ',';
    
    int d_cmd = 0;
    axes[0] = -(float)joy->axes[0]*100;  //Sway  left analog horizontal
    axes[1] = (float)joy->axes[1]*100;   //Surge left analog vertical
    axes[2] = -(float)joy->axes[2]*100;  //Yaw   right analog horizontal
    axes[3] = (float)joy->axes[3]*100;   //Heave right analog vertical
    
    for(int i=0; i<4; i++){
        bytesAxes[i] = (uint8_t)(axes[i] + 100);
        if(prev_bytesAxes[i] != bytesAxes[i])
        {
            prev_bytesAxes[i] = bytesAxes[i];
            joystickActive = true;
        }
        //if(bytesAxes[i] != 100)
      	    //joystickActive = true;
    }
    
    for(int i=0; i<10; i++){
        buttons[i] = joy->buttons[i]; 
        if(prev_buttons[i] != buttons[i])
        {
            prev_buttons[i] = buttons[i];
            joystickActive = true;
        }
        //if(buttons[i] > 0)
            //joystickActive = true;
    }

    //1 Bytes per Button is assigned to 1 bit
    //Data is sent in decimal (uint8_t)
    for(int i=0; i<6; i++)
        d_cmd += (buttons[i] > 0) ? pow(2,i) : 0;
    d_cmd += (buttons[8] > 0) ? pow(2,6) : 0;
    d_cmd += (buttons[9] > 0) ? pow(2,7) : 0;
    bytesButtons[0] = (uint8_t)d_cmd;
    
    bytesJoystick[4] = bytesAxes[0];
    bytesJoystick[5] = bytesAxes[1];
    bytesJoystick[6] = bytesAxes[2];
    bytesJoystick[7] = bytesAxes[3];
    bytesJoystick[8] = bytesButtons[0];
    
    
    uint8_t crc = crc8(bytesJoystick, 9);
    bytesJoystick[9] = crc;
    bytesJoystick[10] = '\n';
    
    //RCLCPP_INFO(this->get_logger(), "checksum '%d' ", crc);   
}


void Luma_Link_Surface::read_data(){
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    
    if(my_serial.isOpen()){
        if(my_serial.available())
        {
            int count = 0;
            int length = 0;
            uint8_t data[180];
            uint8_t buffer[1];

            memset(buffer, '\0', sizeof(buffer));
            memset(data, '\0', sizeof(data));
            do
            {
                length = my_serial.read(&buffer[0], 1);
                sprintf((char*)&data[count], "%c", (char)buffer[0]);
                count = count + length;

            }while(buffer[0] != '\n' && length > 0);
            my_serial.flush();
            
            
            RCLCPP_DEBUG(this->get_logger(), "Bytes read '%d' ", count);
            
            uint8_t output[30];
            std::vector<uint8_t> input; 
            
            uint8_t crc = crc8(data, count - 2);
            if(crc == (uint8_t)data[count-2])
            {
                std::vector<uint8_t> stamp_sec_arr;
                std::vector<uint8_t> stamp_nanosec_arr;
                std::vector<uint8_t> data1_arr;
                std::vector<uint8_t> data2_arr;
                std::vector<uint8_t> data3_arr;
                std::vector<uint8_t> data4_arr;
                std::vector<uint8_t> data5_arr;
                
                if(!flagReceiveSensorData)
                {
                    RCLCPP_INFO(this->get_logger(), "Receiving sensor data...");
                    flagReceiveSensorData = true;
                }
                
                if(split_message_((char*)data, count, (char*)"wre", output))
                {
                
                    for(int i=0; i < 20; i++)
                        input.push_back(output[i]); 
                    
                    stamp_sec_arr = util_tools::slice(input, 0, 3);
                    stamp_nanosec_arr = util_tools::slice(input, 4, 7);
                    data1_arr = util_tools::slice(input, 8, 11);
                    data2_arr = util_tools::slice(input, 12, 15);
                    data3_arr = util_tools::slice(input, 16, 19);
                
                
                    //----- Publish Euler data  ----------------
                    geometry_msgs::msg::Vector3Stamped euler_msg;
                    euler_msg.header.stamp.sec = (int)util_tools::byteToInt(stamp_sec_arr);
                    euler_msg.header.stamp.nanosec = util_tools::byteToInt(stamp_nanosec_arr);
                    euler_msg.vector.x = (double)util_tools::byteToFloat(data1_arr);
                    euler_msg.vector.y = (double)util_tools::byteToFloat(data2_arr);
                    euler_msg.vector.z = (double)util_tools::byteToFloat(data3_arr);

                    euler_pub_->publish(euler_msg);

                }
                else if(split_message_((char*)data, count, (char*)"wrd", output))
                {
                
                    for(int i=0; i < 20; i++)
                        input.push_back(output[i]); 
                    
                    stamp_sec_arr = util_tools::slice(input, 0, 3);
                    stamp_nanosec_arr = util_tools::slice(input, 4, 7);
                    data1_arr = util_tools::slice(input, 8, 11);
                    data2_arr = util_tools::slice(input, 12, 15);
                    data3_arr = util_tools::slice(input, 16, 19);
                
                
                    int stamp_sec = util_tools::byteToInt(stamp_sec_arr);
                    int stamp_nanosec = util_tools::byteToInt(stamp_nanosec_arr);
                    float pressure = util_tools::byteToFloat(data1_arr);
                    float temperature = util_tools::byteToFloat(data2_arr);
                    float depth = util_tools::byteToFloat(data3_arr);
              
                    RCLCPP_DEBUG(this->get_logger(), "stamp_sec: '%d' stamp_nanosec: '%d' press: '%6.2f' temp: '%6.2f' depth: '%6.2f'", 
                    stamp_sec, 
                    stamp_nanosec, 
                    pressure, 
                    temperature, 
                    depth);
                    
                    //----- Publish barometer data  ----------------
                    uuv_msgs::msg::Barometer baro_msg;
                    baro_msg.header.stamp.sec = stamp_sec;
                    baro_msg.header.stamp.nanosec = stamp_nanosec;
                    baro_msg.header.frame_id = "luma250_barometer_data_link";
                    baro_msg.depth = (double)depth;
                    baro_msg.temperature = (double)temperature;
                    baro_msg.pressure = (double)pressure;

                    barometer_pub_->publish(baro_msg);
                    
                }
                else if(split_message_((char*)data, count, (char*)"wrz", output))
                {
                
                    for(int i=0; i < 28; i++)
                        input.push_back(output[i]); 
                    
                    stamp_sec_arr = util_tools::slice(input, 0, 3);
                    stamp_nanosec_arr = util_tools::slice(input, 4, 7);
                    data1_arr = util_tools::slice(input, 8, 11);
                    data2_arr = util_tools::slice(input, 12, 15);
                    data3_arr = util_tools::slice(input, 16, 19);
                    data4_arr = util_tools::slice(input, 20, 23);
                    data5_arr = util_tools::slice(input, 24, 27);
                
                
                    int stamp_sec = util_tools::byteToInt(stamp_sec_arr);
                    int stamp_nanosec = util_tools::byteToInt(stamp_nanosec_arr);
                    float altitude = util_tools::byteToFloat(data1_arr);
                    float velocity_x = util_tools::byteToFloat(data2_arr);
                    float velocity_y = util_tools::byteToFloat(data3_arr);
                    float velocity_z = util_tools::byteToFloat(data4_arr);
                    float fom = util_tools::byteToFloat(data5_arr);

                    // DVL message struct
                    /*
                    dvl_msgs::msg::DVLBeam beam0;
                    dvl_msgs::msg::DVLBeam beam1;
                    dvl_msgs::msg::DVLBeam beam2;
                    dvl_msgs::msg::DVLBeam beam3;
                    */
    
                    //dvl_msgs::msg::DVLDR DVLDeadReckoning;
                    dvl_msgs::msg::DVL dvl;
                    dvl.header.stamp.sec = stamp_sec;
                    dvl.header.stamp.nanosec = stamp_nanosec;
                    dvl.velocity.x = (double)velocity_x;
                    dvl.velocity.y = (double)velocity_y;
                    dvl.velocity.z = (double)velocity_z;
                    dvl.fom = (double)fom;
                    dvl.altitude = (double)altitude;
                    
                    dvl_pub_->publish(dvl);

                    
                }
            }
            else if(count <= 2)
            {
                if(data[0] == '0')
                    RCLCPP_INFO(this->get_logger(), "The rosbag was stoped");
                else if(data[0] == '1')
                    RCLCPP_INFO(this->get_logger(), "The rosbag was started");

                if(flagReceiveSensorData && data[0] == '3')
                {
                    RCLCPP_INFO(this->get_logger(), "No data is being received from sensors!");
                    flagReceiveSensorData = false;
                }
            }
            
            first_time_handshake = current_time;
            luma_msgs.link = luma_msgs::msg::Luma250LP::ACTIVE;
  
        }
        else
        {
            double dt = std::chrono::duration<double>(current_time - first_time_handshake).count();
            //RCLCPP_INFO(this->get_logger(), "dt '%6.2f' ", dt);
            if(dt >= 3.1)
            {
                RCLCPP_WARN(this->get_logger(), "Serial link lost!");
                first_time_handshake = current_time;
                luma_msgs.link = luma_msgs::msg::Luma250LP::LOST;
                flagReceiveSensorData = false;
            }
        }
        
        luma_pub_->publish(luma_msgs);
    }
}


void Luma_Link_Surface::send_data(){
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - first_time).count();
    if(my_serial.isOpen()){
        if(joystickActive)
        {
            my_serial.flushOutput();
            my_serial.write(bytesJoystick, 11);
            first_time = current_time;
            joystickActive = false;
        }
        else
        {
            if(dt >= 0.5)
            {
              my_serial.flushOutput();
              my_serial.write("3\n");
              first_time = current_time;
              //RCLCPP_INFO(this->get_logger(), "Trigger each '%6.2f' secs", dt);
            }
        }	
    }
}



}//end namespace

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<luma_link::Luma_Link_Surface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
