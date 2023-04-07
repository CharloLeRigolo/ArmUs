#include "ArmUs.hpp"

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
 * @brief Main loop of program
 * 
 */
void ArmUs::Run()
{
    ros::Rate loop_rate(ROS_RATE);

    while (ros::ok())
    {
        // Calculate motor velocities depending if in joint or cartesian mode
        m_arm_us_info->calculate_motor_velocities();

        if (!ros::ok())
        {
            // Set all motor velocities to 0
            send_cmd_motor_stop();
        }
        else 
        {
            // Send motor velocities to interface that checks joint limits and returns joint angles
            send_cmd_motor();
        }

        // Send info to GUI : Motor velocities, positions, etc
        send_gui_info();
        // Send info to graph for real time representation of arm in Rviz : Joint angles
        send_3d_graph_info();

        // Check callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }

    // St all motor velocities to 0
    send_cmd_motor_stop();
    ros::shutdown();
}

/**
 * @brief Initialize publishers, subscribers, services, get RosParams
 * 
 */
void ArmUs::Initalize()
{
    m_sub_input =         m_nh.subscribe("joy", 1, &ArmUs::subControllerCallback, this);
    m_sub_gui =           m_nh.subscribe("gui_arm_us_chatter", 1, &ArmUs::sub_gui_callback, this);
    // if (m_controlMode == ControlMode::Real)
    // {
    //     m_sub_joint_states =  m_nh.subscribe("angle_joint_state", 1, &ArmUs::sub_joint_states_callback, this);
    // }
    m_sub_joint_angles = m_nh.subscribe("angles_joint_state", 1, &ArmUs::sub_join_angles_callback, this);
    
    m_pub_motor_interface =         m_nh.advertise<sensor_msgs::JointState>("raw_desired_joint_states", 10);
    m_pub_gui =           m_nh.advertise<arm_us_msg::GuiInfo>("gui_info", 10);
    m_pub_3d_graph =      m_nh.advertise<arm_us_msg::GraphInfo>("graph_info", 10);

    m_client_inv_kin_calc = m_nh.serviceClient<arm_us_msg::InverseKinematicCalc>("inverse_kinematic_calc_service");

    setParams();
}

/**
 * @brief Controller callback, get joysticks, buttons, bumpers, triggers
 * 
 * @param data 
 */
void ArmUs::subControllerCallback(const sensor_msgs::Joy::ConstPtr &data)
{
    m_controller.JoyLeft.set(data->axes[LEFT_JOY_VERT], data->axes[LEFT_JOY_HORI]);
    m_controller.JoyRight.set(data->axes[RIGHT_JOY_VERT], data->axes[RIGHT_JOY_HORI]);
    m_controller.Bumpers.set(data->buttons[LEFT_BUMP], data->buttons[RIGHT_BUMP]);

    /*
    m_controller.Pad.set(data->axes[PAD_VERT], data->axes[PAD_HORI]);
    m_controller.Buttons.set(data->buttons[BUTTON_1], data->buttons[BUTTON_2], data->buttons[BUTTON_3], data->buttons[BUTTON_4]);
    m_controller.Bumpers.set(data->buttons[LEFT_BUMP], data->buttons[RIGHT_BUMP]);
    m_controller.Triggers.set(data->buttons[LEFT_TRIG], data->buttons[RIGHT_TRIG]);
    m_controller.DisplayControllerInputs();
    */

    // Switch modes (between cartesian and joint) with controller
    if (data->buttons[BUTTON_3] == 1 && m_controller.Buttons.Button3 == 0)
    {
        if (m_arm_us_info->MoveMode == MovementMode::Cartesian)
        {
            m_arm_us_info->MoveMode = MovementMode::Joint;
            ROS_INFO("Joint");
        }
        else
        {
            m_arm_us_info->MoveMode = MovementMode::Cartesian;
            ROS_INFO("Cartesian");
        }
    }
    
    /********** Joint **********/
    if (m_arm_us_info->MoveMode == MovementMode::Joint)
    {
        // Change joint controlled with controller
        // Go up one joint
        if (data->buttons[BUTTON_4] == 1 && m_controller.Buttons.Button4 == 0)
        {
            m_arm_us_info->JointControlled++;
            if (m_arm_us_info->JointControlled > 5)
            {
                m_arm_us_info->JointControlled = 1;
            }
            ROS_INFO("Joint controlled : %d", m_arm_us_info->JointControlled);
        }
        // Go down one joint
        if (data->buttons[BUTTON_2] == 1 && m_controller.Buttons.Button2 == 0)
        {
            m_arm_us_info->JointControlled--;
            if (m_arm_us_info->JointControlled < 1)
            {
                m_arm_us_info->JointControlled = 5;
            }
            ROS_INFO("Joint controlled : %d", m_arm_us_info->JointControlled);
        }

        // Get joint command from left joystick (set velocity)
        m_arm_us_info->JointCommand = m_controller.JoyLeft.Vertical * MAX_VEL;

    }
    /********** Cartesian **********/
    else if (m_arm_us_info->MoveMode == MovementMode::Cartesian)
    {
        // Command is velocity of end effector wanted
        m_arm_us_info->CartesianCommand.x = m_controller.JoyLeft.Vertical * MAX_VEL;
        m_arm_us_info->CartesianCommand.y = -m_controller.JoyLeft.Horizontal * MAX_VEL;
        m_arm_us_info->CartesianCommand.z = m_controller.JoyRight.Vertical * MAX_VEL;

        m_arm_us_info->MotorVelocities.m4 = (-m_controller.Bumpers.Left + m_controller.Bumpers.Right) * MAX_VEL;
        m_arm_us_info->MotorVelocities.m5 = (-m_controller.Triggers.Left + m_controller.Triggers.Right) * MAX_VEL;

        // ROS_INFO("x = %f, y = %f, z = %f, a = %f, b = %f", m_arm_us_info->CartesianCommand.x, 
        //                                                       m_arm_us_info->CartesianCommand.y, 
        //                                                       m_arm_us_info->CartesianCommand.z, 
        //                                                       m_arm_us_info->MotorVelocities.m4,
        //                                                       m_arm_us_info->MotorVelocities.m5);
    }

    m_controller.Buttons.set(data->buttons[BUTTON_1], data->buttons[BUTTON_2], data->buttons[BUTTON_3], data->buttons[BUTTON_4]);
}

/**
 * @brief GUI callback
 * 
 * @param data 
 */
void ArmUs::sub_gui_callback(const arm_us_msg::GuiFeedback::ConstPtr &data)
{
    // Switch modes (between cartesian and joint) with GUI
    if (data->joint)
    {
        m_arm_us_info->MoveMode = MovementMode::Joint;
    }
    else if (data->cartesian)
    {
        m_arm_us_info->MoveMode = MovementMode::Cartesian;
    }

    m_arm_us_info->JointControlled = data->joint_controlled;
}

void ArmUs::sub_join_angles_callback(const sensor_msgs::JointState::ConstPtr &data)
{
    m_arm_us_info->JointAngles.set(data->position[0], data->position[1], data->position[2], data->position[3], data->position[4]);
}

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
            ROS_INFO("Started master node in real control mode");
            break;
        }
        // Simulation
        case 1:
        {
            m_controlMode = ControlMode::Simulation;
            ROS_INFO("Started master node in simulation control mode");
            break;
        }
        // Default
        default:
        {
            m_controlMode = ControlMode::Real;
            ROS_ERROR("Control mode parameter not read from launch file");
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
 * @brief Send motor velocities calculated to interface that checks joint limits
 * 
 */
void ArmUs::send_cmd_motor()
{
    sensor_msgs::JointState msg;
    msg.name = { "motor1", "motor2" , "motor3", "motor4", "motor5" };
    msg.velocity = m_arm_us_info->MotorVelocities.get();

    // if (!m_arm_us_info->MotorVelocities.checkIfNull())
    // {
    //     ROS_INFO("Motor velocities sent to interface :");
    //     m_arm_us_info->MotorVelocities.print();
    // }

    m_pub_motor_interface.publish(msg);
}

/**
 * @brief Set all motor velocities to 0
 * 
 */
void ArmUs::send_cmd_motor_stop()
{
    sensor_msgs::JointState msg;
    msg.name = { "motor1", "motor2", "motor3", "motor4", "motor5" };
    msg.velocity = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    m_pub_motor_interface.publish(msg);
    ROS_ERROR("All motors stopped");
}

void ArmUs::send_gui_info()
{
    arm_us_msg::GuiInfo msg;

    //msg.position = m_arm_us_info->MotorPositions.get();
    msg.velocity = m_arm_us_info->MotorVelocities.get();
    //msg.connected = m_arm_us_info->MotorConnections.get();
    //msg.limit_reached = m_arm_us_info->MotorLimits.get();
    m_pub_gui.publish(msg);
}

void ArmUs::send_3d_graph_info()
{
    arm_us_msg::GraphInfo msg;
    msg.angle = m_arm_us_info->JointAngles.get();
    m_pub_3d_graph.publish(msg);
}

/**
 * @brief Call inverse kinematic calculation service to calculate velocities of first 3 motors to move in cartesian mode
 * 
 * @param velocities Motor velocities calculated by service, returns [0, 0, 0] if service call failed or singular matrix detected
 * @param singularMatrix 1 of singular matrix detected, 0 otherwise
 * @return true if call to service was successful
 * @return false if call to service was unsuccessful
 */
bool ArmUs::call_inv_kin_calc_service(Vector3f &velocities, int &singularMatrix)
{
    arm_us_msg::InverseKinematicCalc srv;

    // Send current angles of first 3 joints to service
    Vector5f angles = m_arm_us_info->JointAngles;
    // Send cartesian command to service (Desired velocities in x, y, z)
    Vector3f commands = m_arm_us_info->CartesianCommand;

    srv.request.angles = { angles.m1, angles.m2, angles.m3 };
    srv.request.commands = { commands.x, commands.y, commands.z };

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
