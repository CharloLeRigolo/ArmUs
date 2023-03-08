#ifndef _ROS_rover_arm_vitesse_moteur_msg_h
#define _ROS_rover_arm_vitesse_moteur_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_arm
{

  class vitesse_moteur_msg : public ros::Msg
  {
    public:
      float Period[4];
      bool Dir[4];
      bool En[4];
      typedef bool _calib_type;
      _calib_type calib;

    vitesse_moteur_msg():
      Period(),
      Dir(),
      En(),
      calib(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Periodi;
      u_Periodi.real = this->Period[i];
      *(outbuffer + offset + 0) = (u_Periodi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Periodi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Periodi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Periodi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Period[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        bool real;
        uint8_t base;
      } u_Diri;
      u_Diri.real = this->Dir[i];
      *(outbuffer + offset + 0) = (u_Diri.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Dir[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        bool real;
        uint8_t base;
      } u_Eni;
      u_Eni.real = this->En[i];
      *(outbuffer + offset + 0) = (u_Eni.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->En[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_calib;
      u_calib.real = this->calib;
      *(outbuffer + offset + 0) = (u_calib.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->calib);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Periodi;
      u_Periodi.base = 0;
      u_Periodi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Periodi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Periodi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Periodi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Period[i] = u_Periodi.real;
      offset += sizeof(this->Period[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        bool real;
        uint8_t base;
      } u_Diri;
      u_Diri.base = 0;
      u_Diri.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Dir[i] = u_Diri.real;
      offset += sizeof(this->Dir[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        bool real;
        uint8_t base;
      } u_Eni;
      u_Eni.base = 0;
      u_Eni.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->En[i] = u_Eni.real;
      offset += sizeof(this->En[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_calib;
      u_calib.base = 0;
      u_calib.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->calib = u_calib.real;
      offset += sizeof(this->calib);
     return offset;
    }

    virtual const char * getType() override { return "rover_arm/vitesse_moteur_msg"; };
    virtual const char * getMD5() override { return "885790d6c4fd237460d5105af99e556c"; };

  };

}
#endif
