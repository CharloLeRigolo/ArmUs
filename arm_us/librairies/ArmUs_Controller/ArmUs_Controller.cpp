#include "ros/ros.h"
#include "ArmUs_Controller.hpp"

void Controller::Joystick::set(float v, float h)
{
    Vertical = v;
    Horizontal = h;
}

void Controller::DirectionPad::set(int v, int h)
{
    Vertical = v;
    Horizontal = h;
}

void Controller::Trigger::set(int l, int r)
{
    Left = l;
    Right = r;
}

void Controller::Button::set(int b1, int b2, int b3, int b4)
{
    Button1 = b1;
    Button2 = b2;
    Button3 = b3;
    Button4 = b4;
}

void Controller::DisplayControllerInputs()
{
        ROS_INFO("----------------------------------------");
        ROS_INFO("Joy Left (H, V) = %f, %f", JoyLeft.Horizontal, JoyLeft.Vertical);
        ROS_INFO("Joy Right (H, V) = %f, %f", JoyRight.Horizontal, JoyRight.Vertical);
        ROS_INFO("Direction Pad (H, V) = %d, %d", Pad.Horizontal, Pad.Vertical);
        ROS_INFO("Buttons (1, 2, 3, 4) = %d, %d, %d, %d", Buttons.Button1, Buttons.Button2, Buttons.Button3, Buttons.Button4);
        ROS_INFO("Bumpers (L, R) = %d, %d", Bumpers.Left, Bumpers.Right);
        ROS_INFO("Triggers (L, R) = %d, %d", Triggers.Left, Triggers.Right);
}