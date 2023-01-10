/**
 * util_tools.hpp
 * Copyright (c) 2022 Pablo Gutiérrez
 *
 */
 
#ifndef UTIL_TOOLS_HPP
#define UTIL_TOOLS_HPP

#include <cstdint>
#include <math.h>

namespace util_tools {

struct State
{
   bool brokenSignal;
   int count;
   bool joystickActive;
};

// IMU data structure
struct imu_t {
  float ax, ay, az, gx, gy, gz;
  void clear()
  {
    ax = 0.0;
    ay = 0.0;
    az = 0.0;
    gx = 0.0;
    gy = 0.0;
    gz = 0.0;
  }
};


// Gyro calibration structure
struct point_t {
  float x, y, z;
  void clear()
  {
     x = 0.0;
     y = 0.0;
     z = 0.0;
  }
};

// Attitude structure
struct attitude_t {
  float roll, pitch, yaw;
  void clear()
  {
     roll = 0.0;
     pitch = 0.0;
     yaw = 0.0;
  }
};

struct barometer_t{
  float depth;
  float temperature;
  float pressure;

};


bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

// @param array : input unsigned char array 
// @param array : input array min position
// @param array : input array max position
// @return : Output unsigned char array
template<typename T>std::vector<T> slice(const std::vector<T> v, int m, int n)
{
    std::vector<T> vec(n - m + 1);
    copy(v.begin() + m, v.begin() + n + 1, vec.begin());
    return vec;
}

// @param array : output array
// @return : int16_t
int byteToInt(const std::vector<uint8_t> array)
{
    union{
        int myInt;
	uint8_t myChars[sizeof(int)];
    } test;

    for( int k = 0; k < (int)sizeof(int); k++ )
        test.myChars[k] = array[k];

     
    return test.myInt;
}

// @param array : output array
// @return : double
float byteToFloat(const std::vector<uint8_t> array)
{
    union{
        float myInt;
        uint8_t array[sizeof(float)];
    }union_;

    for( int k = 0; k < (int)sizeof(float); k++ )
        union_.array[k] = array[k];

     
    return union_.myInt;
}

void array_to_array(uint8_t *array1, uint8_t start, uint8_t end, uint8_t *array2)
{
    uint8_t count = 0;
    for(uint8_t i=start; i<=end; i++)
    {
        array1[i] = array2[count];
        count++;
    }
}

} //end_namespace

#endif //UTIL_TOOLS_HPP
