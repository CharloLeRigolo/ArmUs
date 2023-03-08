#ifndef _ROS_SERVICE_diffKinematicsCalc_h
#define _ROS_SERVICE_diffKinematicsCalc_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_arm
{

static const char DIFFKINEMATICSCALC[] = "rover_arm/diffKinematicsCalc";

  class diffKinematicsCalcRequest : public ros::Msg
  {
    public:
      float angles[4];
      float cmd[4];

    diffKinematicsCalcRequest():
      angles(),
      cmd()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->angles[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->cmd[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angles[i]));
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cmd[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return DIFFKINEMATICSCALC; };
    virtual const char * getMD5() override { return "90e073af1783f0808f2ebe86a61aa3fb"; };

  };

  class diffKinematicsCalcResponse : public ros::Msg
  {
    public:
      float vitesses[4];
      typedef bool _singularMatrix_type;
      _singularMatrix_type singularMatrix;

    diffKinematicsCalcResponse():
      vitesses(),
      singularMatrix(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->vitesses[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_singularMatrix;
      u_singularMatrix.real = this->singularMatrix;
      *(outbuffer + offset + 0) = (u_singularMatrix.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->singularMatrix);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vitesses[i]));
      }
      union {
        bool real;
        uint8_t base;
      } u_singularMatrix;
      u_singularMatrix.base = 0;
      u_singularMatrix.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->singularMatrix = u_singularMatrix.real;
      offset += sizeof(this->singularMatrix);
     return offset;
    }

    virtual const char * getType() override { return DIFFKINEMATICSCALC; };
    virtual const char * getMD5() override { return "85e3c5b18098bfef5a58e6d50e2c6617"; };

  };

  class diffKinematicsCalc {
    public:
    typedef diffKinematicsCalcRequest Request;
    typedef diffKinematicsCalcResponse Response;
  };

}
#endif
