#ifndef _ROS_gonzo_msgs_BatteryState_h
#define _ROS_gonzo_msgs_BatteryState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace gonzo_msgs
{

  class BatteryState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _percentage_type;
      _percentage_type percentage;
      typedef float _battery_voltage_type;
      _battery_voltage_type battery_voltage;
      uint32_t cell_voltage_length;
      typedef float _cell_voltage_type;
      _cell_voltage_type st_cell_voltage;
      _cell_voltage_type * cell_voltage;
      typedef bool _present_type;
      _present_type present;

    BatteryState():
      header(),
      percentage(0),
      battery_voltage(0),
      cell_voltage_length(0), st_cell_voltage(), cell_voltage(nullptr),
      present(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_percentage;
      u_percentage.real = this->percentage;
      *(outbuffer + offset + 0) = (u_percentage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_percentage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_percentage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_percentage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->percentage);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.real = this->battery_voltage;
      *(outbuffer + offset + 0) = (u_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_voltage);
      *(outbuffer + offset + 0) = (this->cell_voltage_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cell_voltage_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cell_voltage_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cell_voltage_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_voltage_length);
      for( uint32_t i = 0; i < cell_voltage_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cell_voltagei;
      u_cell_voltagei.real = this->cell_voltage[i];
      *(outbuffer + offset + 0) = (u_cell_voltagei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell_voltagei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell_voltagei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell_voltagei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_voltage[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_present;
      u_present.real = this->present;
      *(outbuffer + offset + 0) = (u_present.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->present);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_percentage;
      u_percentage.base = 0;
      u_percentage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_percentage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_percentage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_percentage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->percentage = u_percentage.real;
      offset += sizeof(this->percentage);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.base = 0;
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_voltage = u_battery_voltage.real;
      offset += sizeof(this->battery_voltage);
      uint32_t cell_voltage_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cell_voltage_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cell_voltage_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cell_voltage_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cell_voltage_length);
      if(cell_voltage_lengthT > cell_voltage_length)
        this->cell_voltage = (float*)realloc(this->cell_voltage, cell_voltage_lengthT * sizeof(float));
      cell_voltage_length = cell_voltage_lengthT;
      for( uint32_t i = 0; i < cell_voltage_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cell_voltage;
      u_st_cell_voltage.base = 0;
      u_st_cell_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cell_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cell_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cell_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cell_voltage = u_st_cell_voltage.real;
      offset += sizeof(this->st_cell_voltage);
        memcpy( &(this->cell_voltage[i]), &(this->st_cell_voltage), sizeof(float));
      }
      union {
        bool real;
        uint8_t base;
      } u_present;
      u_present.base = 0;
      u_present.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->present = u_present.real;
      offset += sizeof(this->present);
     return offset;
    }

    virtual const char * getType() override { return "gonzo_msgs/BatteryState"; };
    virtual const char * getMD5() override { return "eaa84c2fdd07d8077df6e4e4229c7896"; };

  };

}
#endif
