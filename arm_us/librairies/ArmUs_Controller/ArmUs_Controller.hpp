#pragma once

struct Joystick
{
    void set(float v, float h);

    float Vertical = 0.f;
    float Horizontal = 0.f;
};

struct DirectionPad
{
    void set(int v, int h);

    int Vertical = 0;
    int Horizontal = 0;
};

struct Trigger
{
    void set(int l, int r);

    int Left = 0;
    int Right = 0;
};

struct Button
{
    void set(int b1, int b2, int b3, int b4);

    int Button1 = 0;
    int Button2 = 0;
    int Button3 = 0;
    int Button4 = 0;
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