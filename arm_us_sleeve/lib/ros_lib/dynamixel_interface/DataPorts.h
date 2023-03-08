#ifndef _ROS_dynamixel_interface_DataPorts_h
#define _ROS_dynamixel_interface_DataPorts_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dynamixel_interface/DataPort.h"

namespace dynamixel_interface
{

  class DataPorts : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t states_length;
      typedef dynamixel_interface::DataPort _states_type;
      _states_type st_states;
      _states_type * states;

    DataPorts():
      header(),
      states_length(0), st_states(), states(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->states_length);
      for( uint32_t i = 0; i < states_length; i++){
      offset += this->states[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->states_length);
      if(states_lengthT > states_length)
        this->states = (dynamixel_interface::DataPort*)realloc(this->states, states_lengthT * sizeof(dynamixel_interface::DataPort));
      states_length = states_lengthT;
      for( uint32_t i = 0; i < states_length; i++){
      offset += this->st_states.deserialize(inbuffer + offset);
        memcpy( &(this->states[i]), &(this->st_states), sizeof(dynamixel_interface::DataPort));
      }
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_interface/DataPorts"; };
    virtual const char * getMD5() override { return "919597efe48a88349da09d8f46847ef6"; };

  };

}
#endif
