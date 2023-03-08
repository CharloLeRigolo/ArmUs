#ifndef _ROS_rover_udes_CamCommand_h
#define _ROS_rover_udes_CamCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_udes
{

  class CamCommand : public ros::Msg
  {
    public:
      typedef int8_t _mode_type;
      _mode_type mode;
      typedef float _cam_horizontal_type;
      _cam_horizontal_type cam_horizontal;
      typedef float _cam_vertical_type;
      _cam_vertical_type cam_vertical;

    CamCommand():
      mode(0),
      cam_horizontal(0),
      cam_vertical(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_mode;
      u_mode.real = this->mode;
      *(outbuffer + offset + 0) = (u_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      union {
        float real;
        uint32_t base;
      } u_cam_horizontal;
      u_cam_horizontal.real = this->cam_horizontal;
      *(outbuffer + offset + 0) = (u_cam_horizontal.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cam_horizontal.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cam_horizontal.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cam_horizontal.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cam_horizontal);
      union {
        float real;
        uint32_t base;
      } u_cam_vertical;
      u_cam_vertical.real = this->cam_vertical;
      *(outbuffer + offset + 0) = (u_cam_vertical.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cam_vertical.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cam_vertical.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cam_vertical.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cam_vertical);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_mode;
      u_mode.base = 0;
      u_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mode = u_mode.real;
      offset += sizeof(this->mode);
      union {
        float real;
        uint32_t base;
      } u_cam_horizontal;
      u_cam_horizontal.base = 0;
      u_cam_horizontal.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cam_horizontal.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cam_horizontal.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cam_horizontal.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cam_horizontal = u_cam_horizontal.real;
      offset += sizeof(this->cam_horizontal);
      union {
        float real;
        uint32_t base;
      } u_cam_vertical;
      u_cam_vertical.base = 0;
      u_cam_vertical.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cam_vertical.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cam_vertical.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cam_vertical.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cam_vertical = u_cam_vertical.real;
      offset += sizeof(this->cam_vertical);
     return offset;
    }

    virtual const char * getType() override { return "rover_udes/CamCommand"; };
    virtual const char * getMD5() override { return "2be502209bee8b0aed7fd322b07721f1"; };

  };

}
#endif
