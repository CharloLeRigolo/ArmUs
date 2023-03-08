#ifndef _ROS_dynamixel_interface_ServoDiag_h
#define _ROS_dynamixel_interface_ServoDiag_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_interface
{

  class ServoDiag : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef uint8_t _id_type;
      _id_type id;
      typedef const char* _model_name_type;
      _model_name_type model_name;
      typedef uint8_t _error_code_type;
      _error_code_type error_code;
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef float _temperature_type;
      _temperature_type temperature;

    ServoDiag():
      name(""),
      id(0),
      model_name(""),
      error_code(0),
      voltage(0),
      temperature(0)
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
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      uint32_t length_model_name = strlen(this->model_name);
      varToArr(outbuffer + offset, length_model_name);
      offset += 4;
      memcpy(outbuffer + offset, this->model_name, length_model_name);
      offset += length_model_name;
      *(outbuffer + offset + 0) = (this->error_code >> (8 * 0)) & 0xFF;
      offset += sizeof(this->error_code);
      offset += serializeAvrFloat64(outbuffer + offset, this->voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->temperature);
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
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      uint32_t length_model_name;
      arrToVar(length_model_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_model_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_model_name-1]=0;
      this->model_name = (char *)(inbuffer + offset-1);
      offset += length_model_name;
      this->error_code =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->error_code);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->temperature));
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_interface/ServoDiag"; };
    virtual const char * getMD5() override { return "48b049f9561095b33c05b6a03bd01ae5"; };

  };

}
#endif
