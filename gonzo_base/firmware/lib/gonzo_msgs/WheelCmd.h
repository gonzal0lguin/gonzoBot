#ifndef _ROS_gonzo_msgs_WheelCmd_h
#define _ROS_gonzo_msgs_WheelCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "gonzo_msgs/AngularVelocities.h"

namespace gonzo_msgs
{

  class WheelCmd : public ros::Msg
  {
    public:
      typedef gonzo_msgs::AngularVelocities _angular_velocities_type;
      _angular_velocities_type angular_velocities;

    WheelCmd():
      angular_velocities()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->angular_velocities.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->angular_velocities.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "gonzo_msgs/WheelCmd"; };
    virtual const char * getMD5() override { return "7d69bd697799a6d78f0fd12893b19fff"; };

  };

}
#endif
