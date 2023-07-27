#ifndef _ROS_gonzo_msgs_Encoders_h
#define _ROS_gonzo_msgs_Encoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gonzo_msgs
{

  class Encoders : public ros::Msg
  {
    public:
      int64_t ticks[2];

    Encoders():
      ticks()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 2; i++){
      union {
        int64_t real;
        uint64_t base;
      } u_ticksi;
      u_ticksi.real = this->ticks[i];
      *(outbuffer + offset + 0) = (u_ticksi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ticksi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ticksi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ticksi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ticksi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ticksi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ticksi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ticksi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ticks[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 2; i++){
      union {
        int64_t real;
        uint64_t base;
      } u_ticksi;
      u_ticksi.base = 0;
      u_ticksi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ticksi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ticksi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ticksi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ticksi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ticksi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ticksi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ticksi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ticks[i] = u_ticksi.real;
      offset += sizeof(this->ticks[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "gonzo_msgs/Encoders"; };
    virtual const char * getMD5() override { return "83dae13cd2c8306062500bcb5bd87044"; };

  };

}
#endif
