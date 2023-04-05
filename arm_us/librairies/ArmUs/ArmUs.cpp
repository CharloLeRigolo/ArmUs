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

void ArmUs::Run()
{
    ros::Rate loop_rate(ROS_RATE);

    while (ros::ok())
    {
        m_arm_us_info->calculate_motor_velocities();
        m_arm_us_info->calculate_joint_angles();

        if (!ros::ok())
        {
            send_cmd_motor_stop();
        }
        else 
        {
            send_cmd_motor();
        }

        send_gui_info();
        send_3d_graph_info();

        ros::spinOnce();
        loop_rate.sleep();
    }

    send_cmd_motor_stop();
    ros::shutdown();
}

void ArmUs::Initalize()
{
    m_sub_input =         m_nh.subscribe("joy", 1, &ArmUs::subControllerCallback, this);
    m_sub_gui =           m_nh.subscribe("gui_arm_us_chatter", 1, &ArmUs::sub_gui_callback, this);
    if (m_controlMode == ControlMode::Real)
    {
        m_sub_joint_states =  m_nh.subscribe("joint_states", 1, &ArmUs::sub_joint_states_callback, this);
    }
    
    m_pub_motor =         m_nh.advertise<sensor_msgs::JointState>("desired_joint_states", 10);
    m_pub_gui =           m_nh.advertise<arm_us_msg::GuiInfo>("gui_info", 10);
    m_pub_3d_graph =      m_nh.advertise<arm_us_msg::GraphInfo>("graph_info", 10);

    m_client_inv_kin_calc = m_nh.serviceClient<arm_us_msg::InverseKinematicCalc>("inverse_kinematic_calc_service");

    setParams();
}

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

    // Switch modes (cartesian and joint) with controller
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
        if (data->buttons[BUTTON_4] == 1 && m_controller.Buttons.Button4 == 0)
        {
            m_arm_us_info->JointControlled++;
            if (m_arm_us_info->JointControlled > 5)
            {
                m_arm_us_info->JointControlled = 1;
            }
            ROS_INFO("Joint controlled : %d", m_arm_us_info->JointControlled);
        }

        if (data->buttons[BUTTON_2] == 1 && m_controller.Buttons.Button2 == 0)
        {
            m_arm_us_info->JointControlled--;
            if (m_arm_us_info->JointControlled < 1)
            {
                m_arm_us_info->JointControlled = 5;
            }
            ROS_INFO("Joint controlled : %d", m_arm_us_info->JointControlled);
        }

        // Set speed of joint
        m_arm_us_info->JointCommand = m_controller.JoyLeft.Vertical * MAX_VEL;

    }
    /********** Cartesian **********/
    else if (m_arm_us_info->MoveMode == MovementMode::Cartesian)
    {
        // Command is velocity of end effector wanted
        m_arm_us_info->CartesianCommand.x = m_controller.JoyLeft.Vertical * MAX_VEL;
        m_arm_us_info->CartesianCommand.y = -m_controller.JoyLeft.Horizontal * MAX_VEL;
        m_arm_us_info->CartesianCommand.z = m_controller.JoyRight.Vertical * MAX_VEL;
        m_arm_us_info->CartesianCommand.a = (-m_controller.Bumpers.Left + m_controller.Bumpers.Right) * MAX_VEL;

        // ROS_INFO("x = %f, y = %f, z = %f, a = %f", m_arm_us_info->CartesianCommand.x, m_arm_us_info->CartesianCommand.y, m_arm_us_info->CartesianCommand.z, m_arm_us_info->CartesianCommand.a);
    }

    m_controller.Buttons.set(data->buttons[BUTTON_1], data->buttons[BUTTON_2], data->buttons[BUTTON_3], data->buttons[BUTTON_4]);
}

void ArmUs::sub_gui_callback(const arm_us_msg::GuiFeedback::ConstPtr &data)
{
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

void ArmUs::sub_joint_states_callback(const sensor_msgs::JointState::ConstPtr &data)
{
    m_arm_us_info->MotorPositions.set(data->position[0], data->position[1], data->position[2], data->position[3], data->position[4]);
    m_arm_us_info->MotorVelocities.set(data->velocity[0], data->velocity[1], data->velocity[2], data->velocity[3], data->velocity[4]);

    m_arm_us_info->PositionDifference = data->position[0] - data->position[1];
}

void ArmUs::setParams()
{
    int controlMode = 100;
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

void ArmUs::send_cmd_motor()
{
    sensor_msgs::JointState msg;
    msg.name = { "motor1", "motor2" , "motor3", "motor4", "motor5" };
    msg.velocity = m_arm_us_info->MotorVelocities.get();
    m_pub_motor.publish(msg);
}

void ArmUs::send_cmd_motor_stop()
{
    sensor_msgs::JointState msg;
    msg.name = { "motor1", "motor2", "motor3", "motor4", "motor5" };
    msg.velocity = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    m_pub_motor.publish(msg);
    if (verbose)
    {
        ROS_INFO("All motors stopped");
    }
}

void ArmUs::send_gui_info()
{
    arm_us_msg::GuiInfo msg;

    msg.position = m_arm_us_info->MotorPositions.get();
    msg.velocity = m_arm_us_info->MotorVelocities.get();
    msg.connected = m_arm_us_info->MotorConnections.get();
    msg.limit_reached = m_arm_us_info->MotorLimits.get();
    m_pub_gui.publish(msg);
}

void ArmUs::send_3d_graph_info()
{
    arm_us_msg::GraphInfo msg;
    msg.angle = m_arm_us_info->JointAngles.get();
    m_pub_3d_graph.publish(msg);
}

bool ArmUs::call_inv_kin_calc_service(Vector4f &velocities, int &singularMatrix)
{
    arm_us_msg::InverseKinematicCalc srv;

    Vector5f angles = m_arm_us_info->JointAngles;
    Vector4f commands = m_arm_us_info->CartesianCommand;

    srv.request.angles = { angles.m1, angles.m2, angles.m3, angles.m4 };
    srv.request.commands = { commands.x, commands.y, commands.z, commands.a };

    if (m_client_inv_kin_calc.call(srv))
    {
        velocities = { static_cast<float>(srv.response.velocities[0]), 
                       static_cast<float>(srv.response.velocities[1]), 
                       static_cast<float>(srv.response.velocities[2]), 
                       static_cast<float>(srv.response.velocities[3]) };
        singularMatrix = srv.response.singularMatrix;
        return true;
    }
    else 
    {
        return false;
    }
}
