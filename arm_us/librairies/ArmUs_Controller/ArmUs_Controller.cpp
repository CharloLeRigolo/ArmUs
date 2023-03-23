#include "ros/ros.h"
#include "ArmUs_Controller.hpp"

void Joystick::set(float v, float h)
{
    Vertical = v;
    Horizontal = h;
}

void DirectionPad::set(int v, int h)
{
    Vertical = v;
    Horizontal = h;
}

void Trigger::set(int l, int r)
{
    Left = l;
    Right = r;
}

void Button::set(int b1, int b2, int b3, int b4)
{
    Button1 = b1;
    Button2 = b2;
    Button3 = b3;
    Button4 = b4;
}

void Controller::DisplayControllerInputs()
{
        ROS_WARN("----------------------------------------");
        ROS_WARN("Joy Left (H, V) = %f, %f", JoyLeft.Horizontal, JoyLeft.Vertical);
        ROS_WARN("Joy Right (H, V) = %f, %f", JoyRight.Horizontal, JoyRight.Vertical);
        ROS_WARN("Direction Pad (H, V) = %d, %d", Pad.Horizontal, Pad.Vertical);
        ROS_WARN("Buttons (1, 2, 3, 4) = %d, %d, %d, %d", Buttons.Button1, Buttons.Button2, Buttons.Button3, Buttons.Button4);
        ROS_WARN("Bumpers (L, R) = %d, %d", Bumpers.Left, Bumpers.Right);
        ROS_WARN("Triggers (L, R) = %d, %d", Triggers.Left, Triggers.Right);
}