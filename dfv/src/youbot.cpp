#include <dfv/youbot.h>

namespace dfv
{

//***********************//
//        Gripper        //
//***********************//
const std::string Gripper::topic = "arm_1/gripper_controller/position_command";
const float Gripper::open_pos = 0.01f;
const float Gripper::closed_pos = 0.001f;

Gripper::Gripper(ros::NodeHandle& _node_handle):
    node_handle(_node_handle),
    state(Gripper::closed)
{
    this->values.resize(2);
    values[0].joint_uri = "gripper_finger_joint_l";
    values[0].unit = std::string("m");
    values[0].value = Gripper::closed_pos;
    
    values[1].joint_uri = "gripper_finger_joint_r";
    values[1].unit = std::string("m");
    values[1].value = Gripper::closed_pos;
    
    this->publisher = 
        this->node_handle.advertise<brics_actuator::JointPositions>(Gripper::topic, 1000);
}

Gripper::~Gripper()
{
}

void Gripper::Open()
{
    if(this->state == Gripper::closed)
    {
        this->state = Gripper::open;
        values[0].value = Gripper::open_pos;
        values[1].value = Gripper::open_pos;
        this->Publish();
    }
}

void Gripper::Close()
{
    if(this->state == Gripper::open)
    {
        this->state = Gripper::closed;
        values[0].value = Gripper::closed_pos;
        values[1].value = Gripper::closed_pos;
        this->Publish();
    }
}

void Gripper::Publish()
{
    brics_actuator::JointPositions msg;    
    msg.positions = this->values;    
    this->publisher.publish(msg); 
}


//*********************//
//        Joint        //
//*********************//

Joint::Joint(float _min_pos, float _max_pos, float _reset_pos):
    min_pos(_min_pos),
    max_pos(_max_pos),
    reset_pos(_reset_pos),
    state(0.0),
    target(_reset_pos)
{
}

Joint::~Joint()
{
}

void Joint::SetTarget(float target_pos)
{
    if(target_pos < this->min_pos) this->state = this->min_pos;
    else if(target_pos > this->max_pos) this->state = this->max_pos;
    else this->target = target_pos;
}

void Joint::SetTargetRel(float target_pos)
{
    this->SetTarget(target_pos + this->reset_pos);
}

float Joint::GetState() const
{
    return this->state;
}

void Joint::SetState(float state_pos)
{
    this->state = state_pos;
}

//*******************//
//        Arm        //
//*******************//

const std::string Arm::command_topic = "arm_1/arm_controller/position_command";
const std::string Arm::state_topic = "joint_states";

const float Arm::joint_min_pos[] = {0.0100692f, 0.0100692f, -5.02655f, 0.0221239f, 0.110619f};
const float Arm::joint_max_pos[] = {5.84014f, 2.61799f, -0.015708f, 3.4292f, 5.64159f};
const float Arm::joint_offset[] = {2.959675f, 1.144533f, -2.57124f, 1.811086f, 2.91237f};

Arm::Arm(ros::NodeHandle& _node_handle):
    node_handle(_node_handle)
{
    this->command_publisher = 
        _node_handle.advertise<brics_actuator::JointPositions>(Arm::command_topic, 1000);
    
    this->state_subscriber = 
        this->node_handle.subscribe(Arm::state_topic, 
                                    1,
                                    &Arm::StateCallback,
                                    this);
                                    
    for(unsigned int i = 0; i < 5; i++)
    {
        this->joints.push_back(Joint(Arm::joint_min_pos[i], 
                                     Arm::joint_max_pos[i],
                                     Arm::joint_offset[i]));
    }
    
    this->joint_values.resize(5);
    for(unsigned int i = 0; i < this->joints.size(); ++i)
    {
        std::stringstream ss;
        ss << "arm_joint_" << (i+1);
        joint_values[i].joint_uri = ss.str();
        joint_values[i].unit = std::string("rad");
        joint_values[i].value = 0.0;
    }
}

Arm::~Arm()
{
}

Arm& Arm::SetPos(std::vector<float> joint_pos)
{
    assert(joint_pos.size() == 5);
    for(unsigned int i = 0; i < this->joints.size(); i++)
    {
        this->joints[i].SetTargetRel(joint_pos[i]);
    }
    this->Publish();
    return *this;
}

Arm& Arm::Wait(float time)
{
    ros::Duration(time).sleep();
    return *this;
}

void Arm::StateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (unsigned int i = 0; i < this->joints.size(); i++)
    {
        this->joints[i].SetState(msg->position[i]);
    }
}

void Arm::Publish()
{
    while(this->command_publisher.getNumSubscribers() == 0)
    {        
        ros::Duration(0.02f).sleep();
        ros::spinOnce();        
    }
    
    brics_actuator::JointPositions msg;    
    for(unsigned int i = 0; i < this->joints.size(); ++i)
    {
        this->joint_values[i].value = this->joints[i].target;
    }    
    msg.positions = this->joint_values;
    this->command_publisher.publish(msg);
    ros::Duration(0.02f).sleep();
    ros::spinOnce();
    ROS_INFO("Setting arm position.");
}

//********************//
//        Base        //
//********************//

const std::string Base::topic = "cmd_vel";

Base::Base(ros::NodeHandle& _node_handle):
    node_handle(_node_handle),
    linear_vel(0.f),
    side_vel(0.f),
    angular_vel(0.f)
{
    this->publisher = 
        this->node_handle.advertise<geometry_msgs::Twist>(Base::topic, 1000);
}

Base::~Base()
{
}

Base& Base::Move(float linear_vel, float side_vel, float angular_vel)
{
    this->linear_vel = linear_vel;
    this->side_vel = side_vel;
    this->angular_vel = angular_vel;
    this->Publish();
    //ros::spinOnce();
    return *this;
}

Base& Base::MoveFor(float linear_vel, float side_vel, float angular_vel, float time)
{
    ros::Time::waitForValid();
    ros::Time start = ros::Time::now();
    ros::Time end = start + ros::Duration(time);
    this->linear_vel = linear_vel;
    this->side_vel = side_vel;
    this->angular_vel = angular_vel;
    while(ros::Time::now() < end)
    { 
        this->Publish();
    }
    return *this;
}

Base& Base::Stop()
{
    this->linear_vel = 0.0;
    this->side_vel = 0.0;
    this->angular_vel = 0.0;
    this->Publish();
    return *this;
}

void Base::Publish()
{
    while(this->publisher.getNumSubscribers() == 0)
    {        
        ros::Duration(0.02f).sleep();
        ros::spinOnce();        
    }
    geometry_msgs::Twist msg;
    msg.linear.x = this->linear_vel;
    msg.linear.y = this->side_vel;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = this->angular_vel;
    this->publisher.publish(msg);
    ros::Duration(0.02f).sleep();
    ros::spinOnce();
}

//**********************//
//        Youbot        //
//**********************//


YoubotNew::YoubotNew(ros::NodeHandle& _node_handle):
    arm(Arm(_node_handle)),
    base(Base(_node_handle)),
    gripper(Gripper(_node_handle)),
    node_handle(_node_handle)
{
    
}

YoubotNew::~YoubotNew()
{
}


//------------------------------
const float Youbot::joint_min_pos[] = {0.0100692f, 0.0100692f, -5.02655f, 0.0221239f, 0.110619f};
const float Youbot::joint_max_pos[] = {5.84014f, 2.61799f, -0.015708f, 3.4292f, 5.64159f};
const float Youbot::joint_ini_pos[] = {2.959675f, 1.144533f, -2.57124f, 1.811086f, 2.91237f};
const dfv::Vector3 Youbot::r[] = {dfv::Vector3(-0.034f, 0.f, 0.075f), 
                                  dfv::Vector3(0.f, 0.f, 0.155f),
                                  dfv::Vector3(0.f, 0.f, 0.135f),
                                  dfv::Vector3(0.f, 0.f, 0.113f),
                                  dfv::Vector3(0.f, 0.f, 0.105f)};

Youbot::Youbot(ros::NodeHandle& node_handle_, 
               std::string arm_topic_name_, 
               std::string gripper_topic_name,
               std::string joint_states_topic_name,
               std::string cmd_vel_topic_name):
    node_handle(node_handle_),
    gripper_state(closed)
{
    this->arm_publisher = 
        this->node_handle.advertise<brics_actuator::JointPositions>(arm_topic_name_, 1000);
    this->gripper_publisher = 
        this->node_handle.advertise<brics_actuator::JointPositions>(gripper_topic_name, 1000);
    this->cmd_vel_publisher = 
        this->node_handle.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 1000);
    
    this->v_joint_values.resize(5);
    for(int i = 0; i < 5; ++i)
    {
        std::stringstream ss;
        ss << "arm_joint_" << (i+1);
        v_joint_values[i].joint_uri = ss.str();
        v_joint_values[i].unit = std::string("rad");
        v_joint_values[i].value = 0.0;
    }
    
    this->v_gripper_values.resize(2);
    v_gripper_values[0].joint_uri = "gripper_finger_joint_l";
    v_gripper_values[0].unit = std::string("m");
    v_gripper_values[0].value = 0.001;
    
    v_gripper_values[1].joint_uri = "gripper_finger_joint_r";
    v_gripper_values[1].unit = std::string("m");
    v_gripper_values[1].value = 0.001;
    
    this->gripper_positions[0] = 0.01;    
    this->gripper_positions[1] = 0.01;
        
    this->joint_states.resize(5);    
    this->joint_states_subscriber = 
        this->node_handle.subscribe(joint_states_topic_name, 
                                    1,
                                    &Youbot::JointStatesCallback,
                                    this);
        
}

Youbot::~Youbot()
{
    
}

dfv::Quaternion Youbot::GetJointLocalQuaternion(int index) const
{
    if (index == 0)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::k,
            this->joint_positions[0] - this->joint_ini_pos[0]);
    }
    else if (index == 1)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::j,
            this->joint_positions[1] - this->joint_ini_pos[1]);
    }
    else if (index == 2)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::j,
            this->joint_positions[2] - this->joint_ini_pos[2]);
    }
    else if (index == 3)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::j,
            this->joint_positions[3] - this->joint_ini_pos[3]);
    }
    else if (index == 4)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::k,
            this->joint_positions[4] - this->joint_ini_pos[4]);
    }
    else
    {
        return dfv::Quaternion(0, 0, 0, 0);
    }
}

dfv::Quaternion Youbot::GetJointLocalQuatFromAngle(int index, float angle) const
{
    if (index == 0)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::k,
            angle - this->joint_ini_pos[0]);
    }
    else if (index == 1)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::j,
            angle - this->joint_ini_pos[1]);
    }
    else if (index == 2)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::j,
            angle - this->joint_ini_pos[2]);
    }
    else if (index == 3)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::j,
            angle - this->joint_ini_pos[3]);
    }
    else if (index == 4)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::k,
            angle - this->joint_ini_pos[4]);
    }
    else
    {
        return dfv::Quaternion(0, 0, 0, 0);
    }
}

dfv::Vector3 Youbot::GetJointPosition(int index) const
{
    if (index == 0)
    {
        return dfv::Vector3(0, 0, 0);
    }
    else
    {
        dfv::Vector3 rn = this->r[index - 1];
        for (int i = index - 1; i >= 0; i--)
        {
            rn.Rotate(this->GetJointLocalQuaternion(i));
        }        
        return rn + this->GetJointPosition(index - 1);
    }
}

void Youbot::ResetArmPosition()
{
    for (unsigned int i = 0; i < 5; i++)
    {
        this->joint_positions[i] = Youbot::joint_ini_pos[i];
    }
    this->PublishArmPosition();
}

dfv::Vector3 Youbot::GetJointPosFromAngles(unsigned int index, const std::vector<float>& joint_angles) const
{
    if (joint_angles.size() == 5)
    {
        if (index == 0)
        {
            return dfv::Vector3(0, 0, 0);
        }
        else
        {
            dfv::Vector3 rn = this->r[index - 1];
            for (int i = index - 1; i >= 0; i--)
            {
                rn.Rotate(this->GetJointLocalQuatFromAngle(i, joint_angles[i]));
            }        
            return rn + this->GetJointPosFromAngles(index - 1, joint_angles);
        }
    }
    else
    {
        return dfv::Vector3(0, 0, 0);
    }
}

void Youbot::PublishArmPosition()
{
    brics_actuator::JointPositions msg;    
    for(int i = 0; i < 5; ++i)
    {
        v_joint_values[i].value = this->joint_positions[i];
    }    
    msg.positions = v_joint_values;
    this->arm_publisher.publish(msg);    
}

void Youbot::PublishPlatformVel()
{
    geometry_msgs::Twist msg;
    msg.linear.x = this->linear_vel.x;
    msg.linear.y = this->linear_vel.y;
    msg.linear.z = this->linear_vel.z;
    msg.angular.x = this->angular_vel.x;
    msg.angular.y = this->angular_vel.y;
    msg.angular.z = this->angular_vel.z;
    this->cmd_vel_publisher.publish(msg);
    
}

void Youbot::OpenGripper()
{
    if (this->gripper_state == closed)
    {
        brics_actuator::JointPositions gripper_msg;
        v_gripper_values[0].value = 0.01f;
        v_gripper_values[1].value = 0.01f;    
        gripper_msg.positions = v_gripper_values;
        
        this->gripper_publisher.publish(gripper_msg); 
        this->gripper_state = open;
    }
}

void Youbot::CloseGripper()
{
    if (this->gripper_state == open)
    {
        brics_actuator::JointPositions gripper_msg;
        v_gripper_values[0].value = 0.001f;
        v_gripper_values[1].value = 0.001f;    
        gripper_msg.positions = v_gripper_values;
        
        this->gripper_publisher.publish(gripper_msg);
        this->gripper_state = closed;
    }
}

std::vector<float> Youbot::FindAnglesForPos(dfv::Vector3& target_pos)
{
    // cylindrical coordinates
    float theta_0 = atan2(target_pos.y, target_pos.x);
    float r = sqrt(target_pos.x*target_pos.x + target_pos.y*target_pos.y) + Youbot::r[0].x;
    float h = target_pos.z - Youbot::r[0].z;
    
    float l = sqrt(r*r + h*h);
    float x = (l*l + Youbot::r[2].z*Youbot::r[2].z - Youbot::r[1].z*Youbot::r[1].z) / (2*l);
    float hh = sqrt(Youbot::r[2].z*Youbot::r[2].z - x*x);
    float a0 = asin(hh / Youbot::r[1].z);
    float a1 = asin(hh / Youbot::r[2].z);
    float g0 = atan2(h, r);
    float g1 = atan2(r, h);
    float theta_1 = 3.1415926535f/2.f - a0 - g0;
    float theta_2 = a1 + g1 - theta_1;
    
    std::vector<float> res(3);
    res[0] = theta_0;
    res[1] = theta_1;
    res[2] = theta_2;
    
    return res; 
}

void Youbot::JointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (unsigned int i = 0; i < this->joint_states.size(); i++)
    {
        this->joint_states[i] = msg->position[i];
    }
}

Youbot& Youbot::MovePlatform(double linear_vel, double side_vel, double angular_vel, double time)
{
    ros::Time::waitForValid();
    ros::Time start = ros::Time::now();
    ros::Time end = start + ros::Duration(time);
    this->linear_vel.x = linear_vel;
    this->linear_vel.y = side_vel;
    this->linear_vel.z = 0.0;
    this->angular_vel.x = 0.0;
    this->angular_vel.y = 0.0;
    this->angular_vel.z = angular_vel;
    while(ros::Time::now() < end)
    { 
        this->PublishPlatformVel();
        ros::Duration(0.02f).sleep();
        ros::spinOnce();
    }
    return *this;
}

Youbot& Youbot::StopPlatform()
{
    this->linear_vel.x = 0.0;
    this->linear_vel.y = 0.0;
    this->linear_vel.z = 0.0;
    this->angular_vel.x = 0.0;
    this->angular_vel.y = 0.0;
    this->angular_vel.z = 0.0;
    this->PublishPlatformVel();    
    return *this;
}

/*void Youbot::Draw(sf::RenderWindow& window) const
{
    const float scale = 200.f;
    const float center_off = 0.7f;
    
    // background
    sf::Shape bg = sf::Shape::Rectangle(this->position.x, 
                                        this->position.y,
                                        this->position.x + this->size.x,
                                        this->position.y + this->size.y,
                                        sf::Color(235, 215, 255)
                                        );
    window.Draw(bg);
    
    // upper view
    dfv::Vector3 arm_orig(this->position.x + center_off * scale, 
                          this->position.y + center_off * scale,
                          0.f);
                          
    std::vector<dfv::Vector3> curr_joint_pos(6);
    for (unsigned int i = 0; i < curr_joint_pos.size(); i++)
    {
        dfv::Vector3 pos_flipped = this->GetJointPosition(i) * scale;
        pos_flipped.x = -pos_flipped.x;
        curr_joint_pos[i] = pos_flipped + arm_orig;
    }
    
    sf::Color line_color(180, 180, 180);
    
    // joints
    for (unsigned int i = 0; i < curr_joint_pos.size(); i++)
    {
        sf::Shape c_joint = 
            sf::Shape::Circle(curr_joint_pos[i].x, curr_joint_pos[i].y,
                              8.f, sf::Color(0, 0, 0, 0), 1.f, line_color);
        window.Draw(c_joint);
    }
    
    dfv::Vector3 gripper_pos = this->GetJointPosition(5);
    float radius = sqrt(gripper_pos.x*gripper_pos.x + 
                        gripper_pos.y*gripper_pos.y) * scale;
    sf::Shape circ = sf::Shape::Circle(curr_joint_pos[0].x, curr_joint_pos[0].y, radius, sf::Color(0, 0, 0, 0), 
                                       1.f, sf::Color(0, 0, 0));
    window.Draw(circ);
    
    for (unsigned int i = 0; i < 5; i++)
    {
        sf::Shape s = sf::Shape::Line(curr_joint_pos[i].x, curr_joint_pos[i].y,
                                      curr_joint_pos[i + 1].x, curr_joint_pos[i + 1].y,
                                      5.f, sf::Color(255, 128, 0));
        window.Draw(s);
    }    
    
    // side view
    arm_orig += dfv::Vector3(250, 100, 0);
    for (unsigned int i = 0; i < curr_joint_pos.size(); i++)
    {
        dfv::Vector3 pos = this->GetJointPosition(i) * scale * 2.0;
        float r = sqrt(pos.x*pos.x + pos.y*pos.y);
        float h = pos.z;
        curr_joint_pos[i] = dfv::Vector3(r, -h, 0.f) + arm_orig;
    }
    
    // joints
    for (unsigned int i = 0; i < curr_joint_pos.size(); i++)
    {
        sf::Shape c_joint = 
            sf::Shape::Circle(curr_joint_pos[i].x, curr_joint_pos[i].y,
                              8.f, sf::Color(0, 0, 0, 0), 1.f, line_color);
        window.Draw(c_joint);
    }
    
    // dimension lines
    float d_offset = 20.f;
    
    // horizontal
    sf::Shape l = sf::Shape::Line(curr_joint_pos[0].x, curr_joint_pos[0].y, 
                                  curr_joint_pos[0].x, curr_joint_pos[0].y + d_offset,
                                  1.0, sf::Color(0, 0, 0));
    window.Draw(l);
    l = sf::Shape::Line(curr_joint_pos[0].x, curr_joint_pos[0].y + d_offset, 
                        curr_joint_pos[5].x, curr_joint_pos[0].y + d_offset,
                        1.0, sf::Color(0, 0, 0));
    window.Draw(l);
    l = sf::Shape::Line(curr_joint_pos[5].x, curr_joint_pos[0].y + d_offset, 
                        curr_joint_pos[5].x, curr_joint_pos[5].y,
                        1.0, sf::Color(0, 0, 0));
    window.Draw(l);
    
    std::stringstream ss;
    dfv::Vector3 pos = this->GetJointPosition(5);
    ss << floor(sqrt(pos.x*pos.x + pos.y*pos.y) * 1000.f) << " mm";
    sf::String str_r;
    str_r.SetText(ss.str());
    str_r.SetPosition(floor(curr_joint_pos[5].x - 5.f), floor(curr_joint_pos[0].y + 22.f));
    str_r.SetColor(sf::Color(0, 0, 0));
    str_r.SetFont(this->str_radius.GetFont());
    str_r.SetSize(12.f);
    window.Draw(str_r);
    
    
    // vertical
    l = sf::Shape::Line(curr_joint_pos[0].x, curr_joint_pos[0].y, 
                        curr_joint_pos[5].x + d_offset, curr_joint_pos[0].y,
                        1.0, sf::Color(0, 0, 0));
    window.Draw(l);
    l = sf::Shape::Line(curr_joint_pos[5].x + d_offset, curr_joint_pos[0].y, 
                        curr_joint_pos[5].x + d_offset, curr_joint_pos[5].y,
                        1.0, sf::Color(0, 0, 0));
    window.Draw(l);
    l = sf::Shape::Line(curr_joint_pos[5].x + d_offset, curr_joint_pos[5].y, 
                        curr_joint_pos[5].x, curr_joint_pos[5].y,
                        1.0, sf::Color(0, 0, 0));
    window.Draw(l);
    
    ss.str(std::string(""));
    ss << floor(pos.z * 1000.f) << " mm";
    str_r.SetText(ss.str());
    str_r.SetPosition(floor(curr_joint_pos[5].x + 25.f), floor(curr_joint_pos[5].y - 10.f));
    window.Draw(str_r);
    
    
    for (unsigned int i = 0; i < 5; i++)
    {
        sf::Shape s = sf::Shape::Line(curr_joint_pos[i].x, curr_joint_pos[i].y,
                                      curr_joint_pos[i + 1].x, curr_joint_pos[i + 1].y,
                                      5.f, sf::Color(255, 128, 0));
        window.Draw(s);
    } 
    
}

void Youbot::HandleEvent(std::list<std::string>& responses, const sf::Event& event)
{
}

void Youbot::SetFont(const sf::Font& font)
{
    this->str_radius.SetFont(font);
    this->str_height.SetFont(font);
}*/

}



