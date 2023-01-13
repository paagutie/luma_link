/**
 * serial_joystick_sender.cpp
 * @date       04/01/2023
 * Copyright (c) 2023 Pablo Gutiérrez
 *
 */

#include "luma_link/luma_link_vehicle.hpp"


namespace luma_link {


Luma_Link_Vehicle::Luma_Link_Vehicle():
Node("luma_link_vehicle_node")
{
    this->declare_parameter<std::string>("serial_port", "/dev/ttyS1");
    serial_port = this->get_parameter("serial_port").as_string();
    RCLCPP_INFO(get_logger(), "SERIAL_PORT: '%s'", serial_port.c_str());

    serial_euler_array = new uint8_t[50];
    serial_baro_array = new uint8_t[50];
    serial_dvl_array = new uint8_t[50];
    
    first_time = std::chrono::steady_clock::now();
    first_time_send = first_time;
    first_time_receive = first_time;
    
    // Create a callback group
    cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    auto qos_ = rclcpp::QoS(1);
    qos_.best_effort();
    qos_.keep_last(1);

    // Setup subscriber options
    auto rosSubOptions = rclcpp::SubscriptionOptions();
    rosSubOptions.callback_group = cb_group;

    rosbag_pub_ = this->create_publisher<std_msgs::msg::Bool>("rosbag/status", 10);
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("serial/joy", qos_);
    timer_serial = this->create_wall_timer(std::chrono::milliseconds(40),std::bind(&Luma_Link_Vehicle::receive_data, this)); 
    timer_send = this->create_wall_timer(std::chrono::milliseconds(40),std::bind(&Luma_Link_Vehicle::send_data, this)); 

    sub_barometer_ = this->create_subscription<uuv_msgs::msg::Barometer>(
                         "barometer/data",
                         rclcpp::SensorDataQoS(), 
                         std::bind(&Luma_Link_Vehicle::pressure_callback, this, _1),
                         rosSubOptions);

    sub_euler_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
                        "euler/data", 
                        rclcpp::SensorDataQoS(), 
                        std::bind(&Luma_Link_Vehicle::euler_callback, this, _1),
                        rosSubOptions);
    
    sub_dvl_ = this->create_subscription<dvl_msgs::msg::DVL>(
                        "dvl/data", 
                        rclcpp::SensorDataQoS(), 
                        std::bind(&Luma_Link_Vehicle::dvl_callback, this, _1),
                        rosSubOptions);
   
    try
    {
        my_serial.setPort(serial_port);
        my_serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(50);
        my_serial.setTimeout(to);
        my_serial.open();
        RCLCPP_INFO(this->get_logger(), "Communication with Luma established");
        RCLCPP_INFO(this->get_logger(), "Serial port is open");
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port");
        exit(1);
    }
  
    state.joystickActive = true;
    state.brokenSignal = true;

    //Init joy struct
    for(int i=0; i<8; i++)
        joy_struct.buttons[i] = 0;
    for(int i=0; i<4; i++)
        joy_struct.axes[i] = 0.0;

    serialCode = "3\n";
    my_serial.flush();
    usleep(2000);

}

Luma_Link_Vehicle::~Luma_Link_Vehicle() {

    delete [] serial_euler_array;
    delete [] serial_baro_array;
    delete [] serial_dvl_array;
  
    my_serial.flush();
    my_serial.close();
}


void Luma_Link_Vehicle::clear_output(sensor_msgs::msg::Joy &joy_msg)
{
    my_serial.flush(); 
    for(int i=0; i<8; i++){
        joy_msg.buttons.push_back(0);
    }
    for(int i=0; i<4; i++)
    {
        joy_struct.axes[i] = 0.0;
        joy_msg.axes.push_back(joy_struct.axes[i]);
    }

    joy_pub_->publish(joy_msg);  
}

//function to set date and time
void Luma_Link_Vehicle::setDateTime(int date, int month, int year,int hour, int min, int sec)
{
    //buffer to format command
    char date_array[32]={0};
    char hour_array[32]={0};

    sprintf((char*)date_array,"\"%04d%02d%02d\"",year,month,date);
    sprintf((char*)hour_array,"\"%02d:%02d:%02d\"",hour,min,sec);

    string str_date(date_array);
    string str_hour(hour_array);

    string buffer_date;
    string buffer_hour;
    buffer_date = "sudo date +%Y%m%d -s " + str_date;
    buffer_hour = "sudo date +%T -s " + str_hour;

    const char * command_date = buffer_date.c_str();
    const char * command_hour = buffer_hour.c_str();

    //formatting command with the given parameters
    //sprintf((char*)buff,(const char *)"sudo date -s \"%02d/%02d/%04d %02d:%02d:%02d\"",month,date,year,hour,min,sec);
    //sprintf((char*)buff,(char *)"sudo date +%Y%m%d -s ");
    //strcat(buff, fecha);

    //execute formatted command using system()
    system(command_date);
    system(command_hour);
}

void Luma_Link_Vehicle::pressure_callback(const uuv_msgs::msg::Barometer::SharedPtr barometer)
{
    int sec = barometer->header.stamp.sec;
    int nanosec = barometer->header.stamp.nanosec;
    float pressure = (float)barometer->pressure;
    float temperature = (float)barometer->temperature;
    float depth = (float)barometer->depth;

    static_assert(sizeof(float) == 4);
    uint8_t *stamp_sec_arr = reinterpret_cast<uint8_t *>(&sec);
    uint8_t *nano_sec_arr = reinterpret_cast<uint8_t *>(&nanosec);
    uint8_t *pressure_arr = reinterpret_cast<uint8_t *>(&pressure);
    uint8_t *temperature_arr = reinterpret_cast<uint8_t *>(&temperature);
    uint8_t *depth_arr = reinterpret_cast<uint8_t *>(&depth);

    serial_baro_array[0] = 'w';
    serial_baro_array[1] = 'r';
    serial_baro_array[2] = 'd';
    serial_baro_array[3] = ',';
    util_tools::array_to_array(serial_baro_array, 4,7, stamp_sec_arr);
    util_tools::array_to_array(serial_baro_array, 8,11, nano_sec_arr);
    util_tools::array_to_array(serial_baro_array, 12,15, pressure_arr);
    util_tools::array_to_array(serial_baro_array, 16,19, temperature_arr);
    util_tools::array_to_array(serial_baro_array, 20,23, depth_arr);

    uint8_t crc = crc8(serial_baro_array, 24);
    //RCLCPP_INFO(this->get_logger(), "checksum '%d' size: '%d'", crc, 24);

    serial_baro_array[24] = crc;
    serial_baro_array[25] = '\n';
}

void Luma_Link_Vehicle::euler_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr euler)
{
    int stamp_sec_ = (int)euler->header.stamp.sec;
    int stamp_nanosec_ = (int)euler->header.stamp.nanosec;
    float roll_ = (float)euler->vector.x;
    float pitch_ = (float)euler->vector.y;
    float yaw_ = (float)euler->vector.z;

    static_assert(sizeof(float) == 4);
    uint8_t *stamp_sec = reinterpret_cast<uint8_t *>(&stamp_sec_);
    uint8_t *stamp_nanosec = reinterpret_cast<uint8_t *>(&stamp_nanosec_);
    uint8_t *roll = reinterpret_cast<uint8_t *>(&roll_);
    uint8_t *pitch = reinterpret_cast<uint8_t *>(&pitch_);
    uint8_t *yaw = reinterpret_cast<uint8_t *>(&yaw_);
    
    serial_euler_array[0] = 'w';
    serial_euler_array[1] = 'r';
    serial_euler_array[2] = 'e';
    serial_euler_array[3] = ',';
    util_tools::array_to_array(serial_euler_array, 4,7, stamp_sec);
    util_tools::array_to_array(serial_euler_array, 8,11, stamp_nanosec);
    util_tools::array_to_array(serial_euler_array, 12,15, roll);
    util_tools::array_to_array(serial_euler_array, 16,19, pitch);
    util_tools::array_to_array(serial_euler_array, 20,23, yaw);
    
    uint8_t crc = crc8(serial_euler_array, 24);
    //RCLCPP_INFO(this->get_logger(), "checksum '%d' size: '%d'", crc, 20);

    serial_euler_array[24] = crc;
    serial_euler_array[25] = '\n';
    
}

void Luma_Link_Vehicle::dvl_callback(const dvl_msgs::msg::DVL::SharedPtr dvl)
{
    int stamp_sec_ = (int)dvl->header.stamp.sec;
    int stamp_nanosec_ = (int)dvl->header.stamp.nanosec;
    float altitude_ = (float)dvl->altitude;
    float velocity_x_ = (float)dvl->velocity.x;
    float velocity_y_ = (float)dvl->velocity.y;
    float velocity_z_ = (float)dvl->velocity.z;
    float fom_ = (float)dvl->fom;

    // 28 bytes
    static_assert(sizeof(float) == 4);
    uint8_t *stamp_sec = reinterpret_cast<uint8_t *>(&stamp_sec_);
    uint8_t *stamp_nanosec = reinterpret_cast<uint8_t *>(&stamp_nanosec_);
    uint8_t *altitude = reinterpret_cast<uint8_t *>(&altitude_);
    uint8_t *velocity_x = reinterpret_cast<uint8_t *>(&velocity_x_);
    uint8_t *velocity_y = reinterpret_cast<uint8_t *>(&velocity_y_);
    uint8_t *velocity_z = reinterpret_cast<uint8_t *>(&velocity_z_);
    uint8_t *fom = reinterpret_cast<uint8_t *>(&fom_);

    serial_dvl_array[0] = 'w';
    serial_dvl_array[1] = 'r';
    serial_dvl_array[2] = 'z';
    serial_dvl_array[3] = ',';
    util_tools::array_to_array(serial_dvl_array, 4,7, stamp_sec);
    util_tools::array_to_array(serial_dvl_array, 8,11, stamp_nanosec);
    util_tools::array_to_array(serial_dvl_array, 12,15, altitude);
    util_tools::array_to_array(serial_dvl_array, 16,19, velocity_x);
    util_tools::array_to_array(serial_dvl_array, 20,23, velocity_y);
    util_tools::array_to_array(serial_dvl_array, 24,27, velocity_z);
    util_tools::array_to_array(serial_dvl_array, 28,31, fom);

    uint8_t crc = crc8(serial_dvl_array, 32);
    //RCLCPP_INFO(this->get_logger(), "checksum '%d' size: '%d'", crc, 32);

    serial_dvl_array[32] = crc;
    serial_dvl_array[33] = '\n';
}

void Luma_Link_Vehicle::send_data()
{
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - first_time_send).count();
    
    if(my_serial.isOpen()){
        if(!sendSensorData && (dt >= 1.0) && !state.brokenSignal)
        {
            my_serial.flushOutput();
            my_serial.write(serialCode);
            first_time_send = current_time;
            serialCode = "3\n";
            //RCLCPP_INFO(this->get_logger(), "Trigger each '%6.2f' secs", dt);
        }
        if(sendSensorData && (dt >= 0.05))
        {
            first_time_send = current_time;

            if(count == 0)
            {
                my_serial.flushOutput();
                my_serial.write(serial_euler_array, 26);
                count++;
            }
            else if(count == 1)
            {
                my_serial.flushOutput();
                my_serial.write(serial_euler_array, 26);
                my_serial.write(serial_baro_array, 34);
                count++;
            }
            else if(count == 2)
            {
                my_serial.flushOutput();
                my_serial.write(serial_euler_array, 26);
                my_serial.write(serial_dvl_array, 34);
                my_serial.write(serial_baro_array, 26);
                count = 0;
            }
        }

    }  
}

void Luma_Link_Vehicle::receive_data(){

    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    
    //INIT 
    sensor_msgs::msg::Joy joy_ros;
    
    if(state.joystickActive)
    {
        if(my_serial.isOpen() && my_serial.available())
        {
            inputArray.clear();
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
            //RCLCPP_INFO(this->get_logger(), "Bytes read '%d' ", count);
            
            first_time = current_time;
            state.brokenSignal = false;

            uint8_t output[30];
            std::vector<uint8_t> input;

            uint8_t crc = crc8(data, count - 2);
            //RCLCPP_INFO(this->get_logger(), "checksum '%d' checksum mess '%d'", crc, (uint8_t)data[count-2]);
            //Compare Checksum 
            if(crc == (uint8_t)data[count-2])
            {
                if(split_joystick_message((char*)data, count, (char*)"wrj", &joy_struct))
                {
                    //RCLCPP_INFO(this->get_logger(), "axes0: '%6.2f'", joy_struct.axes[0]);
                    joy_ros.axes.clear();
                    joy_ros.buttons.clear();

                    for(int i=0; i<4; i++)
                        joy_ros.axes.push_back((double)joy_struct.axes[i]);

                    for(int i = 0; i<10; i++)
                        if(i < 8)
                            joy_ros.buttons.push_back(joy_struct.buttons[i]);
                        else
                            joy_ros.buttons.push_back(0.0);

                    //Publish Joystick data to ROS network
                    joy_pub_->publish(joy_ros);

                    
                    //joy_struct.buttons[0] = X
                    if(joy_struct.buttons[0] && !sendSensorData)
                        sendSensorData = true;
                    else if(joy_struct.buttons[0] && sendSensorData)
                        sendSensorData = false;

                    //Execute rosbag
                    //joy_struct.buttons[1] = A
                    if(joy_struct.buttons[1] && saveROSbag)
                    {
                        RCLCPP_INFO(this->get_logger(), "The rosbag was stoped");                        
                        rosbag_msgs.data = false;
                        rosbag_pub_->publish(rosbag_msgs);
            
                        saveROSbag = false;
                        serialCode = "0\n";
                    }
                    else if(joy_struct.buttons[1] && !saveROSbag)
                    {
                        RCLCPP_INFO(this->get_logger(), "The rosbag was started");
                        
                        rosbag_msgs.data = true;
                        rosbag_pub_->publish(rosbag_msgs);
            
                        saveROSbag = true;
                        serialCode = "1\n";
                    }

                }
                else if(split_message_((char*)data, count, (char*)"wrt", output))
                {
                    RCLCPP_INFO(this->get_logger(), "Synchronizing systems...");

                    for(int i=0; i < 8; i++)
                        input.push_back(output[i]);

                    //std::ostringstream str1;
                    std::vector<uint8_t> time1 = util_tools::slice(input, 0, 8);
                    long int time_value1 = (long int)util_tools::byteToLongInt(time1);

                    /*
                    str1 << time_value1;
                    str2 << time_value2;
                    std::string str = str1.str() + "." + str2.str();

                    double final_time = 0;
                    std::stringstream geek(str);
                    geek >> final_time;
                    */


                    std::time_t t3 = (double)time_value1;

                    //CTU
                    //tm *tm_local = localtime(&t3);

                    //UTC
                    char* dt_;
                    tm *gmtm = gmtime(&t3);
                    dt_ = asctime(gmtm);

                    //std::cout << "The UTC date and time is " << dt_ << std::endl;
                    this->setDateTime(gmtm->tm_mday, gmtm->tm_mon+1, 1900+gmtm->tm_year,gmtm->tm_hour, gmtm->tm_min, gmtm->tm_sec);

                    RCLCPP_INFO(this->get_logger(), "The UTC date and time is '%s'", dt_);
                    RCLCPP_INFO(this->get_logger(), "Systems are already synchronized :)");
                    serialCode = "4\n";
                    input.clear();

                }
            }
            
            //By keeping the communication active, the control output is set to its default values
            //Otherwise, in the outOfRange method, the automatic emergency control will be activated
            //if(joy_ros.axes.empty() && joy_ros.buttons.empty()){
                //this->clear_output(joy_ros);
            //}
            
        } 
        else //!my_serial.available()
        {
            double dt = std::chrono::duration<double>(current_time - first_time).count();

            if(!my_serial.available() && dt >= 2.5)
            {
                RCLCPP_WARN(this->get_logger(), "Serial link lost!");
                first_time = current_time;
                state.brokenSignal = true;
                my_serial.flush();
            }

            if(state.brokenSignal)
            {
               this->clear_output(joy_ros);   
            }
        }
      
    } // End "if !state.joystick_active"
   
}



}//end namespace

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<luma_link::Luma_Link_Vehicle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
