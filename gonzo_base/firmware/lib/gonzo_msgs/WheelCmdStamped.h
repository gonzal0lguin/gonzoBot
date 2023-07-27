#ifndef _ROS_gonzo_msgs_WheelCmdStamped_h
#define _ROS_gonzo_msgs_WheelCmdStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "gonzo_msgs/WheelCmd.h"

namespace gonzo_msgs
{

  class WheelCmdStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef gonzo_msgs::WheelCmd _wheel_cmd_type;
      _wheel_cmd_type wheel_cmd;

    WheelCmdStamped():
      header(),
      wheel_cmd()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->wheel_cmd.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->wheel_cmd.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "gonzo_msgs/WheelCmdStamped"; };
    virtual const char * getMD5() override { return "f25be03fc85bbdfd52161812fcd3cd18"; };

  };

}
#endif
