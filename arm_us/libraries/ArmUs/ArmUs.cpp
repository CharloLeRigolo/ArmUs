#include "ArmUs.hpp"

/**
 * Arm Us Info object is pointer of virtual base class Arm Us Info that is created
 * as either derived class Arm Us Info Real or Arm Us Info Simul depending on a Rosparam
 * named "/master_node/control_mode" that is passed by command line with launch file.
 * 
 * Initialize() is needed before the instantiation of m_arm_us_info, because it reads
 * the Rosparam that indicates which control mode is wanted.
 */
ArmUs::ArmUs()
{
    Initalize();
    
    if (m_controlMode == ControlMode::Real)
    {
        m_arm_us_info = std::make_unique<ArmUsInfoReal>(std::bind(&ArmUs::call_inv_kin_calc_service, this, std::placeholders::_1, std::placeholders::_2));
    }
    else if (m_controlMode == ControlMode::Simulation)
    {
        m_arm_us_info = std::make_unique<ArmUsInfoSimul>(std::bind(&ArmUs::call_inv_kin_calc_service, this, std::placeholders::_1, std::placeholders::_2));
    }
}

/**
 * Calculates motor commands for each motor depending on the control mode (real or simulation)
 * and the movement mode (joint or cartesian). If in cartesian, calls a service to calculate
 * inverse kinematics. Sends the motor commands to the motor controller, an interface that manages
 * the joint limits and then sends modified commands to motors.
 * 
 * Sends messages to dashboard and 3d graph
 * 
 * Checks callbacks to receive new messages and update information
 */
void ArmUs::Run()
{
    ros::Rate loop_rate(ROS_RATE);

    while (ros::ok())
    {
        // Calculate motor velocities depending if in joint or cartesian mode
        // m_arm_us_info is an instance of the derived class Arm Us Info Real or Arm Us Info Simul
        // and calls the right version of the function (implementation is different in each)
        // It is a virtual void function in base class Arm Us Info
        m_arm_us_info->calculate_motor_velocities();

        if (!ros::ok())
        {
            // Set all motor velocities to 0
            send_cmd_motor_stop();
        }
        else 
        {
            // Send motor velocities to motor translator that checks joint limits and publishes joint angles
            send_cmd_motor();
        }

        // Publish info to dashboard : current movement mode (joint or cartesian) and current joint controlled (1 to 5)
        send_gui_info();
        // Send info to graph for real time representation of arm in Rviz : current joint angles
        send_3d_graph_info();

        // Check callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Set all motor velocities to 0
    send_cmd_motor_stop();
    ros::shutdown();
}

/**
 * m_sub_input subscribes to the controller messages and updates the Controller object.
 * m_sub_joint_angles subscribes to the motor translator and receives the angles of each joint
 * based on their current positions.
 * 
 * m_pub_motor_interface publishes motor commands to translator.
 * m_pub_gui publishes current movement mode and current joint controlled to dashboard.
 * m_pub_3d_graph publishes angles received from motor translator to a 3d graph in Rviz to
 * visualize in real time the position of the arm.
 * 
 * m_client_inv_kin_calc calls a service to find the motor commands needed to move in cartesian.
 * The service receives the current angles of the first 3 joints and the velocity wanted in x, y and z.
 * It sends back the velocities of each of the first 3 joints needed to achieve the cartesian movement,
 * and a boolean that expresses if a singular matrix was encountered.
 * 
 * Calls setParam() that gets all the Rosparams
 */
void ArmUs::Initalize()
{
    m_sub_input =           m_nh.subscribe("joy", 1, &ArmUs::subControllerCallback, this);
    m_sub_joint_angles =    m_nh.subscribe("angles_joint_state", 1, &ArmUs::sub_join_angles_callback, this);
    
    m_pub_motor_interface = m_nh.advertise<sensor_msgs::JointState>("raw_desired_joint_states", 10);
    m_pub_gui =             m_nh.advertise<arm_us_msg::GuiInfo>("gui_info", 10);
    m_pub_3d_graph =        m_nh.advertise<arm_us_msg::GraphInfo>("graph_info", 10);

    m_client_inv_kin_calc = m_nh.serviceClient<arm_us_msg::InverseKinematicCalc>("inverse_kinematic_calc_service");

    setParams();
}

/**
 * Constants are used to indicate which index is which part of the controller
 * Varies depending on the controller
 * The mapping is made in /config/controller_config.yaml
 * 
 * The information in the configuration file is loaded in the launchfile as Rosparams
 * thats are attributed to the constants used as indexes in the code in the initialization.
 * 
 * Change control mode with Button 3 (right button)
 * 
 * Change current joint controlled with Buttons 2 and 4 (down and up buttons respectively)
 * 
 * Get joystick position for motor velocity :
 *  - Joint mode :
 *      - Current joint velocity is vertical axis of left joystick
 *  - Cartesian mode :
 *      - X velocity is vertical axis of left joystick
 *      - Y velocity is horizontal axis of left joystick
 *      - Z velocity is vertical axis of right joystick
 */
void ArmUs::subControllerCallback(const sensor_msgs::Joy::ConstPtr &data)
{
    // Set the message of type sensor_msgs::Joy received from the controller
    // to the Controller object m_controller for ease of use with appropriate names
    m_controller.JoyLeft.set(data->axes[LEFT_JOY_VERT], data->axes[LEFT_JOY_HORI]);
    m_controller.JoyRight.set(data->axes[RIGHT_JOY_VERT], data->axes[RIGHT_JOY_HORI]);
    m_controller.Bumpers.set(data->buttons[LEFT_BUMP], data->buttons[RIGHT_BUMP]);
    m_controller.Triggers.set(data->buttons[LEFT_TRIG], data->buttons[RIGHT_TRIG]);

    // Switch between the movement modes (joint and cartesian) when the button 3 is pressed (Button right)
    if (data->buttons[BUTTON_3] == 1 && m_controller.Buttons.Button3 == 0)
    {
        // If was to cartesian, switch to joint
        if (m_arm_us_info->MoveMode == ArmUsInfo::MovementMode::Cartesian)
        {
            m_arm_us_info->MoveMode = ArmUsInfo::MovementMode::Joint;
        }
        else
        {
            // If was to joint, switch to cartesian
            m_arm_us_info->MoveMode = ArmUsInfo::MovementMode::Cartesian;
        }
    }
    
    // If the current movement mode is in joint
    if (m_arm_us_info->MoveMode == ArmUsInfo::MovementMode::Joint)
    {
        // Change joint controlled with controller
        // Go up one joint if button 4 is pressed (Button up)
        if (data->buttons[BUTTON_4] == 1 && m_controller.Buttons.Button4 == 0)
        {
            m_arm_us_info->JointControlled++;
            if (m_arm_us_info->JointControlled > 5)
            {
                m_arm_us_info->JointControlled = 1;
            }
        }
        // Go down one joint if button 2 is pressed (Button down)
        if (data->buttons[BUTTON_2] == 1 && m_controller.Buttons.Button2 == 0)
        {
            m_arm_us_info->JointControlled--;
            if (m_arm_us_info->JointControlled < 1)
            {
                m_arm_us_info->JointControlled = 5;
            }
        }

        // Get joint command from left joystick (set velocity)
        // In joint mode, only one joint moves at one time, so only one joystick direction is needed to describe velocity
        m_arm_us_info->JointCommand = m_controller.JoyLeft.Vertical * MAX_VEL;
    }

    // If the current movement mode is in joint
    else if (m_arm_us_info->MoveMode == ArmUsInfo::MovementMode::Cartesian)
    {
        // Get cartesian command (Velocity wanted in X, Y and Z, to send to motor translator)
        // X axis is vertical axis of the left joystick
        // Y axis is horizontal axis of the left joystick
        // Z axis is vertical axis of the right joystick
        m_arm_us_info->CartesianCommand.x = m_controller.JoyLeft.Vertical * MAX_VEL;
        m_arm_us_info->CartesianCommand.y = -m_controller.JoyLeft.Horizontal * MAX_VEL;
        m_arm_us_info->CartesianCommand.z = m_controller.JoyRight.Vertical * MAX_VEL;

        // Joints 4 and 5 are controlled by the bumpers and triggers of the controller, respectively
        // Cartesian mode is 3 directions (x, y, z) and 3 joints (1, 2, 3)
        m_arm_us_info->MotorVelocities.m4 = (-m_controller.Bumpers.Left + m_controller.Bumpers.Right) * MAX_VEL;
        m_arm_us_info->MotorVelocities.m5 = (-m_controller.Triggers.Left + m_controller.Triggers.Right) * MAX_VEL;
    }

    // Set state of buttons so that logic executes only when the buttons are first pressed
    m_controller.Buttons.set(data->buttons[BUTTON_1], data->buttons[BUTTON_2], data->buttons[BUTTON_3], data->buttons[BUTTON_4]);
}

/**
 * Receives the angles of the joints from the motor translator and saves them in m_arm_us_info->JointAngles
 */
void ArmUs::sub_join_angles_callback(const sensor_msgs::JointState::ConstPtr &data)
{
    m_arm_us_info->JointAngles.set(data->position[0], data->position[1], data->position[2], data->position[3], data->position[4]);
}

/**
 * Get Rosparams for the control mode. The control mode is Real mode by default but can be changed
 * by adding the argument "simulation" to the command line when launching with roslaunch
 * to set control mode to simulation ("simulation:=0" is real mode, "simulation:=1" is simulation mode)
 * 
 * Get Rosparams from config/controller_config.yaml for the mapping of the indexes of the controller's
 * joysticks and buttons
 */
void ArmUs::setParams()
{
    int controlMode = -1;
    m_nh.getParam("/master_node/control_mode", controlMode);

    switch (controlMode)
    {
        // Real
        case 0:
        {
            m_controlMode = ControlMode::Real;
            // ROS_INFO("Started master node in real control mode");
            break;
        }
        // Simulation
        case 1:
        {
            m_controlMode = ControlMode::Simulation;
            // ROS_INFO("Started master node in simulation control mode");
            break;
        }
    }

    m_nh.getParam("/master_node/left_joy_hori", LEFT_JOY_HORI);
    m_nh.getParam("/master_node/left_joy_vert", LEFT_JOY_VERT);
    
    m_nh.getParam("/master_node/right_joy_hori", RIGHT_JOY_HORI);
    m_nh.getParam("/master_node/right_joy_vert", RIGHT_JOY_VERT);

    m_nh.getParam("/master_node/pad_hori", PAD_HORI);
    m_nh.getParam("/master_node/pad_vert", PAD_VERT);


    m_nh.getParam("/master_node/button_1", BUTTON_1);
    m_nh.getParam("/master_node/button_2", BUTTON_2);
    m_nh.getParam("/master_node/button_3", BUTTON_3);
    m_nh.getParam("/master_node/button_4", BUTTON_4);

    m_nh.getParam("/master_node/left_bump", LEFT_BUMP);
    m_nh.getParam("/master_node/right_bump", RIGHT_BUMP);

    m_nh.getParam("/master_node/left_trig", LEFT_TRIG);
    m_nh.getParam("/master_node/right_trig", RIGHT_TRIG);
}

/**
 * Send the motor commands to the motor translator by publishing a message of type
 * sensor_msgs::JointState in which the name and velocity attributes are set.
 */
void ArmUs::send_cmd_motor()
{
    sensor_msgs::JointState msg;
    msg.name = { "motor1", "motor2" , "motor3", "motor4", "motor5" };
    msg.velocity = m_arm_us_info->MotorVelocities.get();
    m_pub_motor_interface.publish(msg);
}

/**
 * Same as send_cmd_motor(), but instead of sending the motor commands calculated in 
 * m_arm_us_info->calculate_motor_velocities(), send zeros to all motors
 */
void ArmUs::send_cmd_motor_stop()
{
    sensor_msgs::JointState msg;
    msg.name = { "motor1", "motor2", "motor3", "motor4", "motor5" };
    msg.velocity = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    m_pub_motor_interface.publish(msg);
}

/**
 * The current control mode msg.current_mode is a int8 with a value of 0 if in joint mode
 * and a value of 1 if in cartesian mode.
 * 
 * The current joint controlled msg.current_joint is a int8 with a value between 1 and 5
 * representing the joints 1 to 5.
 */
void ArmUs::send_gui_info()
{
    arm_us_msg::GuiInfo gui_info_msg;

    // Current joint controlled
    gui_info_msg.current_joint = m_arm_us_info->JointControlled;

    // Current movement mode
    if (m_arm_us_info->MoveMode == ArmUsInfo::MovementMode::Joint)
    {
        gui_info_msg.current_mode = 0;
    }
    else if (m_arm_us_info->MoveMode == ArmUsInfo::MovementMode::Cartesian)
    {
        gui_info_msg.current_mode = 1;
    }

    m_pub_gui.publish(gui_info_msg);
}

/**
 * Get the angles of each joints calculated by the motor translator
 */
void ArmUs::send_3d_graph_info()
{
    arm_us_msg::GraphInfo msg;
    msg.angle = m_arm_us_info->JointAngles.get();
    m_pub_3d_graph.publish(msg);
}

/**
 * This function calls the service that calculates the motor commands needed for each joint [J1, J2, J3]
 * to move in cartesian axis [X, Y, Z].
 * 
 * The service message used is ArmUsMsg::InverseKinematicCalc
 * 
 * The service is called with two arguments :
 *  - The cartesian movement wanted in X, Y, Z (float64[3] -> [X, Y, Z])
 *  - The current angles of the first 3 joints in degrees (float64[3] -> [q1, q2, q3])
 * 
 * The service returns two arguments :
 *  - The motor velocities of each joints needed to satisfy the cartesian commands sent (float64[3] -> [J1, J2, J3])
 *  - A boolean that expresses if a singular matrix was encountered when calculating the inverse kinematics of the arm
 * 
 * The arguments are used only to propagate the result of the service, they are meant to be called 
 * with a Vector3f filled with zeros ([0, 0, 0]) and a false boolean
 * 
 * If a singular matrix is encountered, or the service could not be called, the Vector3f velocities paramter is filled with zeros.
 * Otherwise, it gives the velocities of each joints calculated by the service.
 * 
 * This function is situated in ArmUs and not in ArmUsInfo because a ros::NodeHandle is necessary to call a service
 * This function is used by Arm Us Info Real and Arm Us Info Simul by being passed as a std::function when the object is created
 */
bool ArmUs::call_inv_kin_calc_service(ArmUsInfo::Vector3f &velocities, int &singularMatrix)
{
    arm_us_msg::InverseKinematicCalc srv;

    // Send current angles of first 3 joints to service
    ArmUsInfo::Vector5f angles = m_arm_us_info->JointAngles;
    // Send cartesian command to service (Desired velocities in x, y, z)
    ArmUsInfo::Vector3f commands = m_arm_us_info->CartesianCommand;

    // Create service request
    srv.request.angles = { angles.m1, angles.m2, angles.m3 };
    srv.request.commands = { commands.x, commands.y, commands.z };

    // Receive service response
    // If call to service is successful
    if (m_client_inv_kin_calc.call(srv))
    {
        // Returns velocities of first 3 motors
        velocities = { static_cast<float>(srv.response.velocities[0]), 
                       static_cast<float>(srv.response.velocities[1]), 
                       static_cast<float>(srv.response.velocities[2]) };
        // Returns if matrix is singular
        singularMatrix = srv.response.singularMatrix;
        return true;
    }
    // If call is unsuccessful
    else 
    {
        velocities = { 0.f, 0.f, 0.f };
        singularMatrix = 0;
        return false;
    }
}
