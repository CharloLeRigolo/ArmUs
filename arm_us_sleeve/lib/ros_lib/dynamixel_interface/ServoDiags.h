#ifndef _ROS_dynamixel_interface_ServoDiags_h
#define _ROS_dynamixel_interface_ServoDiags_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dynamixel_interface/ServoDiag.h"

namespace dynamixel_interface
{

  class ServoDiags : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t diagnostics_length;
      typedef dynamixel_interface::ServoDiag _diagnostics_type;
      _diagnostics_type st_diagnostics;
      _diagnostics_type * diagnostics;

    ServoDiags():
      header(),
      diagnostics_length(0), st_diagnostics(), diagnostics(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->diagnostics_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->diagnostics_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->diagnostics_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->diagnostics_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->diagnostics_length);
      for( uint32_t i = 0; i < diagnostics_length; i++){
      offset += this->diagnostics[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t diagnostics_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      diagnostics_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      diagnostics_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      diagnostics_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->diagnostics_length);
      if(diagnostics_lengthT > diagnostics_length)
        this->diagnostics = (dynamixel_interface::ServoDiag*)realloc(this->diagnostics, diagnostics_lengthT * sizeof(dynamixel_interface::ServoDiag));
      diagnostics_length = diagnostics_lengthT;
      for( uint32_t i = 0; i < diagnostics_length; i++){
      offset += this->st_diagnostics.deserialize(inbuffer + offset);
        memcpy( &(this->diagnostics[i]), &(this->st_diagnostics), sizeof(dynamixel_interface::ServoDiag));
      }
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_interface/ServoDiags"; };
    virtual const char * getMD5() override { return "e0c439c1a732520d1c71cd95240a273a"; };

  };

}
#endif
