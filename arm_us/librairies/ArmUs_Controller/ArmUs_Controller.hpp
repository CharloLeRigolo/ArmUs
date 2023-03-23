#pragma once

struct Joystick
{
    void set(float v, float h);

    float Vertical;
    float Horizontal;
};

struct DirectionPad
{
    void set(int v, int h);

    int Vertical;
    int Horizontal;
};

struct Trigger
{
    void set(int l, int r);

    int Left;
    int Right;
};

struct Button
{
    void set(int b1, int b2, int b3, int b4);

    int Button1; 
    int Button2; 
    int Button3; 
    int Button4; 
};

struct Controller
{
    void DisplayControllerInputs();

    Joystick JoyLeft;
    Joystick JoyRight;
    DirectionPad Pad;
    Button Buttons;
    Trigger Bumpers;
    Trigger Triggers;
};