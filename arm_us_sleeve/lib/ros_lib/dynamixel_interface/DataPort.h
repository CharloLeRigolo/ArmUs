#ifndef _ROS_dynamixel_interface_DataPort_h
#define _ROS_dynamixel_interface_DataPort_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_interface
{

  class DataPort : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      uint32_t port_values_length;
      typedef uint16_t _port_values_type;
      _port_values_type st_port_values;
      _port_values_type * port_values;

    DataPort():
      name(""),
      port_values_length(0), st_port_values(), port_values(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset + 0) = (this->port_values_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->port_values_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->port_values_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->port_values_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->port_values_length);
      for( uint32_t i = 0; i < port_values_length; i++){
      *(outbuffer + offset + 0) = (this->port_values[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->port_values[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->port_values[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t port_values_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      port_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      port_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      port_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->port_values_length);
      if(port_values_lengthT > port_values_length)
        this->port_values = (uint16_t*)realloc(this->port_values, port_values_lengthT * sizeof(uint16_t));
      port_values_length = port_values_lengthT;
      for( uint32_t i = 0; i < port_values_length; i++){
      this->st_port_values =  ((uint16_t) (*(inbuffer + offset)));
      this->st_port_values |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_port_values);
        memcpy( &(this->port_values[i]), &(this->st_port_values), sizeof(uint16_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_interface/DataPort"; };
    virtual const char * getMD5() override { return "8b6c6004f40f6ef774b6478f360fb7d1"; };

  };

}
#endif
