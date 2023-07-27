#ifndef _ROS_gonzo_msgs_AngularVelocities_h
#define _ROS_gonzo_msgs_AngularVelocities_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gonzo_msgs
{

  class AngularVelocities : public ros::Msg
  {
    public:
      uint32_t joint_velocity_length;
      typedef float _joint_velocity_type;
      _joint_velocity_type st_joint_velocity;
      _joint_velocity_type * joint_velocity;

    AngularVelocities():
      joint_velocity_length(0), st_joint_velocity(), joint_velocity(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint_velocity_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_velocity_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_velocity_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_velocity_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_velocity_length);
      for( uint32_t i = 0; i < joint_velocity_length; i++){
      union {
        float real;
        uint32_t base;
      } u_joint_velocityi;
      u_joint_velocityi.real = this->joint_velocity[i];
      *(outbuffer + offset + 0) = (u_joint_velocityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_velocityi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_velocityi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_velocityi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_velocity[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t joint_velocity_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_velocity_length);
      if(joint_velocity_lengthT > joint_velocity_length)
        this->joint_velocity = (float*)realloc(this->joint_velocity, joint_velocity_lengthT * sizeof(float));
      joint_velocity_length = joint_velocity_lengthT;
      for( uint32_t i = 0; i < joint_velocity_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_joint_velocity;
      u_st_joint_velocity.base = 0;
      u_st_joint_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_joint_velocity = u_st_joint_velocity.real;
      offset += sizeof(this->st_joint_velocity);
        memcpy( &(this->joint_velocity[i]), &(this->st_joint_velocity), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "gonzo_msgs/AngularVelocities"; };
    virtual const char * getMD5() override { return "6ff762cfd0bfa60f616148f97b45a5cd"; };

  };

}
#endif
