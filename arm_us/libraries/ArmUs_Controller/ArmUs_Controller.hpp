/**
 * @file ArmUs_Controller.hpp
 * @author Mikael St-Arnaud et Philippe Michaud (stam1001, micp1402)
 * @brief This class holds information about the state of the controller used to control the arm
 * @version 0.1 
 * @date 2023-04-13
 *
 * @copyright Copyright (c) 2023 - See ARM_US licence
 */

#pragma once

/**
 * @brief Data structure that holds information about the state of a controller.
 * Is used of ease of use when receiving messages of type sensor_msgs::Joy so that
 * the information is accessible with more intuitive names.
 */
struct Controller
{
    /**
     * @brief Data structure that holds information about a joystick.
     * Vertical axis is a float value between -1.0 and 1.0
     * Horizontal axis is a float value between -1.0 and 1.0
     */
    struct Joystick
    {
        /**
         * @brief Set vertical axis value and horizontal axis value, in this order
         * 
         * @param v Vertical axis value between -1.0 and 1.0
         * @param h Horizontal axis value between -1.0 and 1.0
         */
        void set(float v, float h);

        float Vertical = 0.f; // Vertical axis value
        float Horizontal = 0.f; // Horizontal axis value
    };
    
    /**
     * @brief Data structure that holds information about a direction pad.
     * Vertical axis is an int value of -1, 0 or 1
     * Horizontal axis is an int value of -1, 0 or 1
     */
    struct DirectionPad
    {
        /**
         * @brief Set vertical axis value and horizontal axis value, in this order
         * 
         * @param v Vertical axis value is -1, 0 or 1
         * @param h Horizontal axis value is -1, 0 or 1
         */
        void set(int v, int h);

        int Vertical = 0; // Vertical axis value
        int Horizontal = 0; // Horizontal axis value
    };

    /**
     * @brief Data structure that holds information about a trigger.
     * Vertical axis is an int value of 0 or 1
     * Horizontal axis is an int value of 0 or 1
     */
    struct Trigger
    {
        /**
         * @brief Set left trigger value and right trigger value, in this order
         * 
         * @param l Left trigger value is 0 or 1
         * @param r Right trigger value is 0 or 1
         */
        void set(int l, int r);

        int Left = 0; // Left trigger value
        int Right = 0; // Right trigger value
    };

    /**
     * @brief Data structure that holds information about 4 buttons.
     * Buttons have an int value of 0 or 1
     */
    struct Button
    {
        /**
         * @brief Set the 4 buttons' values
         * 
         * @param b1 // Button 1 (Left)
         * @param b2 // Button 2 (Down)
         * @param b3 // Button 3 (Right)
         * @param b4 // Button 4 (Up)
         */
        void set(int b1, int b2, int b3, int b4);

        int Button1 = 0; // Button 1 (Left)
        int Button2 = 0; // Button 2 (Down)
        int Button3 = 0; // Button 3 (Right)
        int Button4 = 0; // Button 4 (Up)
    };

    /**
     * @brief Displays the current state of the controller
     */
    void DisplayControllerInputs();

    Joystick JoyLeft;   // Left joystick
    Joystick JoyRight;  // Right joystick
    DirectionPad Pad;   // Direction pad
    Button Buttons;     // Buttons
    Trigger Bumpers;    // Bumpers
    Trigger Triggers;   // Triggers
};