#ifndef _ROS_SERVICE_E_stop_h
#define _ROS_SERVICE_E_stop_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_base_heartbeat
{

static const char E_STOP[] = "rover_base_heartbeat/E_stop";

  class E_stopRequest : public ros::Msg
  {
    public:
      typedef bool _E_stop_type;
      _E_stop_type E_stop;

    E_stopRequest():
      E_stop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_E_stop;
      u_E_stop.real = this->E_stop;
      *(outbuffer + offset + 0) = (u_E_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->E_stop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_E_stop;
      u_E_stop.base = 0;
      u_E_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->E_stop = u_E_stop.real;
      offset += sizeof(this->E_stop);
     return offset;
    }

    virtual const char * getType() override { return E_STOP; };
    virtual const char * getMD5() override { return "a5cc8431f16b6b7bd7d3ad44eefd92f9"; };

  };

  class E_stopResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    E_stopResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    virtual const char * getType() override { return E_STOP; };
    virtual const char * getMD5() override { return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class E_stop {
    public:
    typedef E_stopRequest Request;
    typedef E_stopResponse Response;
  };

}
#endif
