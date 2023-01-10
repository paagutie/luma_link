#ifndef _LINK_PROTOCOL_H_
#define _LINK_PROTOCOL_H_

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include <iostream>

#ifdef __cplusplus
extern "C"
{
#endif

//Lookup table taked ftom https://waterlinked.github.io/dvl/dvl-protocol/#serial-protocol
static const uint8_t lookup_table[256] = {
    0x00U,0x07U,0x0EU,0x09U,0x1CU,0x1BU,0x12U,0x15U,
    0x38U,0x3FU,0x36U,0x31U,0x24U,0x23U,0x2AU,0x2DU,
    0x70U,0x77U,0x7EU,0x79U,0x6CU,0x6BU,0x62U,0x65U,
    0x48U,0x4FU,0x46U,0x41U,0x54U,0x53U,0x5AU,0x5DU,
    0xE0U,0xE7U,0xEEU,0xE9U,0xFCU,0xFBU,0xF2U,0xF5U,
    0xD8U,0xDFU,0xD6U,0xD1U,0xC4U,0xC3U,0xCAU,0xCDU,
    0x90U,0x97U,0x9EU,0x99U,0x8CU,0x8BU,0x82U,0x85U,
    0xA8U,0xAFU,0xA6U,0xA1U,0xB4U,0xB3U,0xBAU,0xBDU,
    0xC7U,0xC0U,0xC9U,0xCEU,0xDBU,0xDCU,0xD5U,0xD2U,
    0xFFU,0xF8U,0xF1U,0xF6U,0xE3U,0xE4U,0xEDU,0xEAU,
    0xB7U,0xB0U,0xB9U,0xBEU,0xABU,0xACU,0xA5U,0xA2U,
    0x8FU,0x88U,0x81U,0x86U,0x93U,0x94U,0x9DU,0x9AU,
    0x27U,0x20U,0x29U,0x2EU,0x3BU,0x3CU,0x35U,0x32U,
    0x1FU,0x18U,0x11U,0x16U,0x03U,0x04U,0x0DU,0x0AU,
    0x57U,0x50U,0x59U,0x5EU,0x4BU,0x4CU,0x45U,0x42U,
    0x6FU,0x68U,0x61U,0x66U,0x73U,0x74U,0x7DU,0x7AU,
    0x89U,0x8EU,0x87U,0x80U,0x95U,0x92U,0x9BU,0x9CU,
    0xB1U,0xB6U,0xBFU,0xB8U,0xADU,0xAAU,0xA3U,0xA4U,
    0xF9U,0xFEU,0xF7U,0xF0U,0xE5U,0xE2U,0xEBU,0xECU,
    0xC1U,0xC6U,0xCFU,0xC8U,0xDDU,0xDAU,0xD3U,0xD4U,
    0x69U,0x6EU,0x67U,0x60U,0x75U,0x72U,0x7BU,0x7CU,
    0x51U,0x56U,0x5FU,0x58U,0x4DU,0x4AU,0x43U,0x44U,
    0x19U,0x1EU,0x17U,0x10U,0x05U,0x02U,0x0BU,0x0CU,
    0x21U,0x26U,0x2FU,0x28U,0x3DU,0x3AU,0x33U,0x34U,
    0x4EU,0x49U,0x40U,0x47U,0x52U,0x55U,0x5CU,0x5BU,
    0x76U,0x71U,0x78U,0x7FU,0x6AU,0x6DU,0x64U,0x63U,
    0x3EU,0x39U,0x30U,0x37U,0x22U,0x25U,0x2CU,0x2BU,
    0x06U,0x01U,0x08U,0x0FU,0x1AU,0x1DU,0x14U,0x13U,
    0xAEU,0xA9U,0xA0U,0xA7U,0xB2U,0xB5U,0xBCU,0xBBU,
    0x96U,0x91U,0x98U,0x9FU,0x8AU,0x8DU,0x84U,0x83U,
    0xDEU,0xD9U,0xD0U,0xD7U,0xC2U,0xC5U,0xCCU,0xCBU,
    0xE6U,0xE1U,0xE8U,0xEFU,0xFAU,0xFDU,0xF4U,0xF3U,
};

typedef struct vector3
{        
    double x;
    double y;
    double z;
}vector3;

//AHRS
struct ahrs_t
{
    uint8_t header0;
    uint8_t header1;
    uint8_t header2;
    uint8_t header3;
    int time_stamp;
    float roll;
    float pitch;
    float yaw;
};

struct serial_wre_t
{
    uint8_t header0;
    uint8_t header1;
    uint8_t header2;
    uint8_t header3;
    int time_stamp;
    float roll;
    float pitch;
    float yaw;
    uint8_t crc;
    uint8_t ctr;
};

union union_euler_t{
  serial_wre_t data;
  uint8_t package[sizeof(serial_wre_t)];
};


//Pressure sensor
struct wrd_t
{
    double time_stamp;
    double pressure;
    double temperature;
    double depth;
};

//IMU sensor
struct wri_t
{
    double time_stamp;
    vector3 angular_velocity;
    vector3 linear_aceleration;
};

//AHRS euler angles
struct wre_t
{
    double time_stamp;
    vector3 euler_angles;
};

//DVL Sensor
//Velocity report (wrz) struct    
struct wrz_t
{
    vector3 velocity;
    char *valid;
    double altitude;
    double fom;
    double covariance[9];
    int time_of_validity;
    int time_of_transmission;
    int status;
};

//Transducer report (wru) struct
struct wru_t
{
    int id;
    double velocity;
    double distance;
    int rssi;
    int nsd;
};

//Dead reckoning report (wrp) struct
struct wrp_t
{
    double time_stamp;
    vector3 position;
    double pos_std;
    double roll;
    double pitch;
    double yaw;
    double velocity;
    int status;
};

struct joy_t
{
    float axes[4];
    int buttons[12];
};

//Structure for already split data
typedef enum ElementType {
    et_str, 
    et_lint, 
    et_int,
    et_dbl
}ElementType;

struct Element
{
    ElementType type;
    void *data;
};

// CR8 function taked from https://waterlinked.github.io/dvl/dvl-protocol/#serial-protocol
uint8_t crc8(uint8_t *message, int message_length) {
    uint8_t checksum = 0;
    while (message_length > 0) {
        checksum = lookup_table[*message ^ checksum];
        message++;
        message_length--;
        //printf("mess: %c checksum: %03x\n", *message, checksum);
    }
    return checksum;
}

//Struct for joystick
void create_joytick_struct(struct Element *arr_joy)
{
    for(int i = 0; i<5; i++)
    {
        arr_joy[i].type = et_str;
        arr_joy[i].data = malloc(sizeof(char));
    }
}

//Struct for IMU sensor
//Struct for AHRS euler angles
//Struct for Pressure sensor
void create_wrx_double_struct(struct Element *arr_wrx, int message_length)
{
    for(int i = 0; i<message_length; i++)
    {
        arr_wrx[i].type = et_dbl;
        arr_wrx[i].data = malloc(sizeof(double));
    }
}


void create_wrp_struct(struct Element *arr_wrp)
{
    for (int i = 0; i < 10; i++)
    {
        if(i==9)
        {
            arr_wrp[i].type = et_int;
            arr_wrp[i].data = malloc(sizeof(long));
        }
        else
        {
            arr_wrp[i].type = et_dbl;
            arr_wrp[i].data = malloc(sizeof(double));
        }
    }
}

void create_wru_struct(struct Element *arr_wru)
{
    for (int i = 0; i < 5; i++)
    {
        if(i==0 || i==3 || i==4)
        {
            arr_wru[i].type = et_int;
            arr_wru[i].data = malloc(sizeof(long));
        }
        if(i==1 || i==2)
        {
            arr_wru[i].type = et_dbl;
            arr_wru[i].data = malloc(sizeof(double));
        }
    }
}

void create_wrz_struct(struct Element *arr_wrz)
{
    /* Initialize element's struct */
    for (int i = 0; i < 18; i++)
    {
        if(i == 3) //valid (3)
        {
            arr_wrz[i].type = et_str;
            arr_wrz[i].data = malloc(sizeof(char));
        }
        else if(i == 15 || i == 16 || i == 18) //time_of_validity (15), time_of_transmissionb(16) and status (17)
        {
            arr_wrz[i].type = et_int;
            arr_wrz[i].data = malloc(sizeof(long));
        }
        else // Velocity (0,1,2), altitude (4), fom (5), covariance (6-14)
        {
            arr_wrz[i].type = et_dbl;
            arr_wrz[i].data = malloc(sizeof(double));
        }
    }
}


void free_struct(struct Element *array, int size)
{
    /* All data was dynamically allocated, so free each item's data */
    for(int i=0; i<size; i++)
        free(array[i].data);
    free(array);
}

uint8_t split_joystick_message(char *str, int size, char *type, struct joy_t *joy)
{
    char *ptr = strstr(str, type);

    if(ptr != NULL)
    {
        char data[180];
        memset(data, '\0', sizeof(data));
        
        for(int i=4; i <size-2 ; i++ )
        {
            sprintf(&data[i-4], "%c", str[i]);
        }

        for(int i=0; i<4; i++)
        {
            float serialInput = float(data[i]/100.0 - 1.0);
            joy->axes[i] = serialInput;
        }
        
        
        int dec = (int)data[4];
        joy->buttons[0] = dec;

        for(int i = 0; i<8; i++)
        {
            joy->buttons[i] = (dec % 2 == 0 ? false : true);
            dec /= 2;
        }

        return 1;
        
    }
    else
        return 0;
}

uint8_t split_message_(char *str, int size, char *type, uint8_t *output)
{
    char *ptr = strstr(str, type);
    if(ptr != NULL)
    {
        //fprintf(stderr, "Incoming data: %s\n", ptr);
        
        char data[180];
        memset(data, '\0', sizeof(data));
        for(int i=4; i <size-2 ; i++ )
        {
            sprintf(&data[i-4], "%c", str[i]);
            output[i-4] = (uint8_t)data[i-4];
        }

        return 1;
    }
    else
        return 0;

}

uint8_t split_message(char *str, int size, char *type, struct Element *output)
{
    char *ptr = strstr(str, type);
    if(ptr != NULL)
    {
        fprintf(stderr, "Incoming data: %s", ptr);
        
        //fprintf(stderr, "time: %ld got %s", time,  str);

        //char *ptr = strtok(buffer, "\n");
        char data[180];
        memset(data, '\0', sizeof(data));
        for(int i=4; i <size-5 ; i++ )
        {
            sprintf(&data[i-4], "%c", str[i]);
        }

        printf("count: %s\n", data);

        /*
        char *ptr = strtok(data, ";");
        while(ptr != NULL)
        {
            printf("%s\n", ptr);
            ptr = strtok(NULL, ";"); 
        }
        */

        char *token, *subtoken;
        char *saveptr1, *saveptr2;
        int count = 0;

        for (token = strtok_r(data, ";", &saveptr1); 
            token != NULL; 
            token = strtok_r(NULL, ";", &saveptr1)) {   
            //printf("token:%s\n", token);
            for (subtoken = strtok_r(token, ",", &saveptr2); 
                subtoken != NULL; 
                subtoken = strtok_r(NULL, ",", &saveptr2))
            {

                double ftemp = 0.0;
                unsigned long itemp = 0;
                switch (output[count].type)
                {
                case et_dbl:
                    ftemp = atof(subtoken);
                    *((double*)(output[count].data)) = ftemp;
                    //printf("float: %4.11f\n", *((double*)(output[count].data)));
                    break;
                
                case et_int:
                    itemp = strtol(subtoken, NULL, 10);
                    *((long*)(output[count].data)) = itemp;
                    //printf("int: %ld\n", *((long*)(output[count].data)));
                    break;

                case et_str:
                    output[count].data = (char*)subtoken;
                    //printf("str: %s\n", (char*)output[count].data);
                    break;
                
                default:
                    break;
                }

                count++;
            }
        }

        return 1;
    }
    else
        return 0;

}

uint8_t is_dead_reckoning_reset_successful(char *str)
{
    char *ptr;
    ptr = strstr(str, "wra");
    if(ptr != NULL)
        return 1;
    ptr = strstr(str, "wrn");
    if(ptr != NULL)
        return 0; 

    return 2;
}

#ifdef __cplusplus
}
#endif

#endif  // _LINK_PROTOCOL_H_
