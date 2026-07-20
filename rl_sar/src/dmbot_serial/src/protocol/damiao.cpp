#include "dmbot_serial/protocol/damiao.h"
#include <signal.h>
#include <iostream>
#include <boost/bind/bind.hpp>
#include <iomanip>
namespace damiao
{

Limit_param limit_param[Num_Of_Motor]=
{
        {12.566, 50, 5},   // DM3507         check
        {12.5, 30, 10},   // DM4310          check
        {12.5, 50, 10},   // DM4310_48V
        {12.5, 10, 28},   // DM4340          check
        {12.5, 20, 28},   // DM4340_48V      check
        {12.5, 45, 12},   // DM6006          check
        {12.566, 20, 120},   // DM6248P       check
        {12.5, 45, 20},   // DM8006          check
        {12.5, 45, 54},   // DM8009          check
        {12.5, 25, 200},  // DM10010L        check
        {12.5, 20, 200},  // DM10010         check
        {12.5, 280, 1},   // DMH3510         check
        {12.5, 45, 10},   // DMH6215
        {12.5, 2000, 2},    // DMS3519         check
        {12.5, 45, 10}    // DMG6220         check
};

Motor::Motor(DM_Motor_Type motor_type, Control_Mode ctrl_mode,uint16_t can_id, uint16_t master_id)
        :  Motor_Type(motor_type),mode(ctrl_mode),Master_id(master_id), Can_id(can_id){
    this->limit_param = damiao::limit_param[motor_type];
}

void Motor::updateTimeInterval()
{
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt = now - last_time_;
    last_time_ = now;

    delta_time_ = dt.count(); // in seconds
}

double Motor::getTimeInterval()
{
    return delta_time_;
}

void Motor::receive_data(float q, float dq, float tau)
{
    this->state_q = q;
    this->state_dq = dq;
    this->state_tau = tau;
}

void Motor::set_param(int key, float value)
{
    ValueType v{};
    v.value.floatValue = value;
    v.isFloat = true;
    param_map[key] = v;
}

void Motor::set_param(int key, uint32_t value)
{
    ValueType v{};
    v.value.uint32Value = value;
    v.isFloat = false;
    param_map[key] = v;
}

float Motor::get_param_as_float(int key) const
{
    auto it = param_map.find(key);
    if (it != param_map.end())
    {
        if (it->second.isFloat)
        {
            return it->second.value.floatValue;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}

uint32_t Motor::get_param_as_uint32(int key) const
{
    auto it = param_map.find(key);
    if (it != param_map.end()) {
        if (!it->second.isFloat) {
            return it->second.value.uint32Value;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}

bool Motor::is_have_param(int key) const
{
    return param_map.find(key) != param_map.end();
}

/******one CAN bus, one Motor_Control**********************/
Motor_Control::Motor_Control(std::string bus_name,std::vector<DmActData> *data_ptr,Can_control_Mode can_mode)
    :  data_ptr_(data_ptr) ,current_can_mode_(can_mode)
{
    uint16_t motor_mode;
    for (auto it = data_ptr_->begin(); it != data_ptr_->end(); ++it)
    {
    //  std::cout << "Current motor mode: " << static_cast<int>(it->mode) << std::endl;
     motor_mode = static_cast<int>(it->mode);
     //Iterate over all motors on this bus
     std::shared_ptr<Motor> motor = std::make_shared<Motor>(it->motorType,it->mode,it->can_id, it->mst_id);
     addMotor(motor);
    }
    int thread_priority=95;
    while (!socket_can_.open(bus_name, boost::bind(&Motor_Control::canframeCallback, this, boost::placeholders::_1), thread_priority))

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    enable_all(motor_mode);//Enable all motors on this interface
    //usleep(1000000);//1s
    std::cout<<"Motor_Control init success!"<<std::endl;
}

Motor_Control::~Motor_Control()
{
    std::cout<<"enter ~Motor_Control"<<std::endl;
    disable_all();//Disable all motors on this interface
}

/**
 * @brief add motor to class Add motor
 * @param DM_Motor : motor object motor object
 */
void Motor_Control::addMotor(std::shared_ptr<Motor> DM_Motor)
{
    motors.insert({DM_Motor->GetCanId(), DM_Motor});
    motors.insert({DM_Motor->GetMasterId(), DM_Motor});
}

void Motor_Control::enable_all(uint16_t mode_)
{
    for(auto& it : motors)
    {
        switch (mode_)
        {
            case 0:switchControlMode(*(it.second),MIT);
                break;
            case 256:switchControlMode(*(it.second),POS_VEL);
                break;
            case 512:switchControlMode(*(it.second),VEL);
                break;
            case 768:switchControlMode(*(it.second),POS_FORCE);
                break;
            default:
                break;
        }
       for(int j=0;j<5;j++)
       {
        control_cmd(it.second->GetCanId()+it.second->GetMotorMode(),0xFC);
        usleep(2000);
       }
    }
}

void Motor_Control::disable_all()
{
    for(auto& it : motors)
    {
        for(int j=0;j<5;j++)
        {
         control_cmd(it.second->GetCanId()+it.second->GetMotorMode(),0xFD);
         usleep(2000);
        }
    }
}

/*
    * @description: read motor register param Read internal motor register parameters; see the Damiao manual for the register list
    * @param DM_Motor: motor object motor object
    * @param RID: register id register ID  example: damiao::UV_Value
    * @return: motor param Motor parameter; returns 0 if the query fails
    */
float Motor_Control::read_motor_param(Motor &DM_Motor,uint8_t RID)
{
    read_write_save=true;//Send a read-parameter command; the returned data differs from normal feedback and needs separate handling
    uint16_t id = DM_Motor.GetCanId();
    uint8_t id_low = id & 0xff;
    uint8_t id_high = (id >> 8) & 0xff;
    if(current_can_mode_==0){
        can_frame frame{};
        frame.can_id = 0x7FF;
        frame.can_dlc = 8;
        frame.data[0] = id_low;
        frame.data[1]=  id_high;
        frame.data[2] = 0x33;
        frame.data[3] = RID;
        frame.data[4] = 0x00;
        frame.data[5]=  0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        socket_can_.write(&frame);
    }
    else{
        canfd_frame frame{};
        frame.can_id = 0x7FF;
        frame.len = 8;
        frame.flags = CANFD_BRS|CANFD_ESI; //CANFD_BRS|CANFD_FDF|CANFD_ESI;
        frame.data[0] = id_low;
        frame.data[1]=  id_high;
        frame.data[2] = 0x33;
        frame.data[3] = RID;
        frame.data[4] = 0x00;
        frame.data[5]=  0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        socket_can_.write2(&frame);
    }
    return 0;
}

/*
    * @description: save all param to motor flash Save all motor parameters to flash
    * @param DM_Motor: motor object motor object
    * Default motor parameters are not written to flash automatically; a write operation is required
    */
void Motor_Control::save_motor_param(Motor &DM_Motor)
{
    uint16_t id = DM_Motor.GetCanId();
    uint16_t mode=DM_Motor.GetMotorMode();
    control_cmd(id+mode,0xFD);//Disable
    usleep(10000);
    read_write_save=true;//Send a save-parameter command; the returned data differs from normal feedback and needs separate handling
    uint8_t id_low = id & 0xff;
    uint8_t id_high = (id >> 8) & 0xff;
    if(current_can_mode_==0){
        can_frame frame{};
        frame.can_id = 0x7FF;
        frame.can_dlc = 8;
        frame.data[0] = id_low;
        frame.data[1]=  id_high;
        frame.data[2] = 0xAA;
        frame.data[3] = 0x01;
        frame.data[4] = 0x00;
        frame.data[5]=  0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        socket_can_.write(&frame);
    }
    else{
        canfd_frame frame{};
        frame.can_id = 0x7FF;
        frame.len = 8;
        frame.flags = CANFD_BRS|CANFD_ESI; //CANFD_BRS|CANFD_FDF|CANFD_ESI;
        frame.data[0] = id_low;
        frame.data[1]=  id_high;
        frame.data[2] = 0xAA;
        frame.data[3] = 0x01;
        frame.data[4] = 0x00;
        frame.data[5]=  0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        socket_can_.write2(&frame);
    }
    usleep(100000);
}

//Read motor feedback command
void Motor_Control::refresh_motor_status(Motor& motor)
{
    uint8_t can_low = motor.GetCanId() & 0xff; // id low 8 bit
    uint8_t can_high = (motor.GetCanId() >> 8) & 0xff; //id high 8 bit
    if(current_can_mode_==0){
        can_frame frame{};
        frame.can_id = 0x7FF;
        frame.can_dlc = 4;
        frame.data[0] = can_low;
        frame.data[1] = can_high;
        frame.data[2] = 0xCC;
        frame.data[3] = 0x00;
        socket_can_.write(&frame);
    }
    else{
        canfd_frame frame{};
        frame.can_id = 0x7FF;
        frame.len = 4;
        frame.flags = CANFD_BRS|CANFD_ESI; //CANFD_BRS|CANFD_FDF|CANFD_ESI;
        frame.data[0] = can_low;
        frame.data[1] = can_high;
        frame.data[2] = 0xCC;
        frame.data[3] = 0x00;
        socket_can_.write2(&frame);
    }
}

void Motor_Control::control_cmd(uint16_t id, uint8_t cmd)
{
    if(current_can_mode_==0){
        can_frame frame{};
        frame.can_id = id;
        frame.can_dlc = 8;
        frame.data[0] = 0xff;
        frame.data[1]=  0xff;
        frame.data[2] = 0xff;
        frame.data[3] = 0xff;
        frame.data[4] = 0xff;
        frame.data[5]=  0xff;
        frame.data[6] = 0xff;
        frame.data[7] = cmd;
        socket_can_.write(&frame);
    }
    else{
        canfd_frame frame{};
        frame.can_id = id;
        frame.len = 8;
        frame.flags = CANFD_BRS|CANFD_ESI; //CANFD_BRS|CANFD_FDF|CANFD_ESI;
        frame.data[0] = 0xff;
        frame.data[1]=  0xff;
        frame.data[2] = 0xff;
        frame.data[3] = 0xff;
        frame.data[4] = 0xff;
        frame.data[5]=  0xff;
        frame.data[6] = 0xff;
        frame.data[7] = cmd;
        socket_can_.write2(&frame);
    }
}

void Motor_Control::write_motor_param(Motor &DM_Motor,uint8_t RID,const uint8_t data[4])
{
    read_write_save=true;//Send a write-parameter command; the returned data differs from normal feedback and needs separate handling
    uint16_t id = DM_Motor.GetCanId();
    uint8_t can_low = id & 0xff;
    uint8_t can_high = (id >> 8) & 0xff;
    if(current_can_mode_==0){
        can_frame frame{};
        frame.can_id = 0x7FF;
        frame.can_dlc = 8;
        frame.data[0] = can_low;
        frame.data[1]=  can_high;
        frame.data[2] = 0x55;
        frame.data[3] = RID;
        frame.data[4] = data[0];
        frame.data[5]=  data[1];
        frame.data[6] = data[2];
        frame.data[7] = data[3];
        socket_can_.write(&frame);
    }
    else{
         canfd_frame frame{};
        frame.can_id = 0x7FF;
        frame.len = 8;
        frame.flags = CANFD_BRS|CANFD_ESI; //CANFD_BRS|CANFD_FDF|CANFD_ESI;
        frame.data[0] = can_low;
        frame.data[1]=  can_high;
        frame.data[2] = 0x55;
        frame.data[3] = RID;
        frame.data[4] = data[0];
        frame.data[5]=  data[1];
        frame.data[6] = data[2];
        frame.data[7] = data[3];
        socket_can_.write2(&frame);
    }
}

void Motor_Control::set_zero_position(Motor &DM_Motor)
{
    control_cmd(DM_Motor.GetCanId()+DM_Motor.GetMotorMode(),0xFE);
}

void Motor_Control::control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
{
    // Position, velocity, and torque use linear mapping to convert floating-point data to signed fixed-point data
    static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
        float span = xmax - xmin;
        float data_norm = (x - xmin) / span;
        uint16_t data_uint = data_norm * ((1 << bits) - 1);
        return data_uint;
    };
    uint16_t id = DM_Motor.GetCanId();
    if(motors.find(id) == motors.end())
    {
        std::cerr << "[Error] In control_mit,no motor with id222 " << DM_Motor.GetCanId() << " is registered." << std::endl;
        std::exit(-1);  // Terminate the program; a nonzero return value indicates an error
    }
    auto& m = motors[id];
    uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
    Limit_param limit_param_cmd = m->get_limit_param();
    uint16_t q_uint = float_to_uint(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
    uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.DQ_MAX,limit_param_cmd.DQ_MAX, 12);
    uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);
    if(current_can_mode_==0)
    {
        can_frame frame{};
        frame.can_id = id+MIT_MODE;
        frame.can_dlc = 8;
        frame.data[0] = (q_uint >> 8) & 0xff;
        frame.data[1] = q_uint & 0xff;
        frame.data[2] = dq_uint >> 4;
        frame.data[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
        frame.data[4] = kp_uint & 0xff;
        frame.data[5]= kd_uint >> 4;
        frame.data[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
        frame.data[7] = tau_uint & 0xff;
        socket_can_.write(&frame);
    }
    else
    {
        canfd_frame frame{};
        frame.can_id = id+MIT_MODE;
        frame.len = 8;
        frame.flags = CANFD_BRS|CANFD_ESI; //CANFD_BRS|CANFD_FDF|CANFD_ESI;
        frame.data[0] = (q_uint >> 8) & 0xff;
        frame.data[1] = q_uint & 0xff;
        frame.data[2] = dq_uint >> 4;
        frame.data[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
        frame.data[4] = kp_uint & 0xff;
        frame.data[5]= kd_uint >> 4;
        frame.data[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
        frame.data[7] = tau_uint & 0xff;
        socket_can_.write2(&frame);
    }
}

void Motor_Control::control_pos_vel(Motor &DM_Motor,float pos,float vel)
{
    uint16_t id = DM_Motor.GetCanId();
    if(motors.find(id) == motors.end())
    {
        std::cerr << "[Error] In control_pos_vel,no motor with id333 " << DM_Motor.GetCanId() << " is registered." << std::endl;
        std::exit(-1);  // Terminate the program; a nonzero return value indicates an error
    }
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&pos;
    vbuf=(uint8_t*)&vel;
    if(current_can_mode_==0)
    {
        can_frame frame{};
        frame.can_id = id+POS_VEL_MODE;
        frame.can_dlc = 8;
        frame.data[0] = *pbuf;
        frame.data[1] = *(pbuf+1);
        frame.data[2] = *(pbuf+2);
        frame.data[3] = *(pbuf+3);
        frame.data[4] = *vbuf;
        frame.data[5]=  *(vbuf+1);
        frame.data[6] = *(vbuf+2);
        frame.data[7] = *(vbuf+3);
        socket_can_.write(&frame);
    }
    else{
        canfd_frame frame{};
        frame.can_id = id+POS_VEL_MODE;
        frame.len = 8;
        frame.flags = CANFD_BRS|CANFD_ESI; //CANFD_BRS|CANFD_FDF|CANFD_ESI;
        frame.data[0] = *pbuf;
        frame.data[1] = *(pbuf+1);
        frame.data[2] = *(pbuf+2);
        frame.data[3] = *(pbuf+3);
        frame.data[4] = *vbuf;
        frame.data[5]=  *(vbuf+1);
        frame.data[6] = *(vbuf+2);
        frame.data[7] = *(vbuf+3);
        socket_can_.write2(&frame);
    }
}

void Motor_Control::control_vel(Motor &DM_Motor,float vel)
{
    uint16_t id =DM_Motor.GetCanId();
    if(motors.find(id) == motors.end())
    {
        std::cerr << "[Error] In control_vel,no motor with id444 " << DM_Motor.GetCanId() << " is registered." << std::endl;
        std::exit(-1);  // Terminate the program; a nonzero return value indicates an error
    }
    uint8_t *vbuf;
    vbuf=(uint8_t*)&vel;
    if(current_can_mode_==0)
    {
        can_frame frame{};
        frame.can_id = id+VEL_MODE;
        frame.can_dlc = 4;
        frame.data[0] = *vbuf;
        frame.data[1]=  *(vbuf+1);
        frame.data[2] = *(vbuf+2);
        frame.data[3] = *(vbuf+3);
        socket_can_.write(&frame);
    }
    else{
        canfd_frame frame{};
        frame.can_id = id+VEL_MODE;
        frame.len = 4;
        frame.flags = CANFD_BRS|CANFD_ESI; //CANFD_BRS|CANFD_FDF|CANFD_ESI;
        frame.data[0] = *vbuf;
        frame.data[1]=  *(vbuf+1);
        frame.data[2] = *(vbuf+2);
        frame.data[3] = *(vbuf+3);
        socket_can_.write2(&frame);
    }
}

void Motor_Control::receive_param(uint8_t* data)
{
    // uint16_t canID = (uint16_t(data[1]) << 8) | data[0];
    uint16_t canID = data[0]&0x0F;
    uint8_t RID = data[3];
    if (motors.find(canID) == motors.end())
    {
        // std::cerr << "[Error] In receive_param,no motor with id " << canID << " is registered." << std::endl;
        // std::exit(-1);  // Terminate the program; a nonzero return value indicates an error
        return;
    }
    if(is_in_ranges(RID))
    {
        uint32_t data_uint32 = (uint32_t(data[7]) << 24) | (uint32_t(data[6]) << 16) | (uint32_t(data[5]) << 8) | data[4];
        motors[canID]->set_param(RID, data_uint32);
        if(RID==10)
        {
            if(data_uint32==1)
            {
                motors[canID]->set_mode(MIT_MODE);
            }
            else if(data_uint32==2)
            {
                motors[canID]->set_mode(POS_VEL_MODE);
            }
            else if(data_uint32==3)
            {
                motors[canID]->set_mode(VEL_MODE);
            }
            else if(data_uint32==4)
            {
                motors[canID]->set_mode(POS_FORCE_MODE);
            }
        }
    }
    else
    {
        float data_float = uint8_to_float(data + 4);
        motors[canID]->set_param(RID, data_float);
    }
}


/*
    * @description: switch control mode Switch motor control mode
    * @param DM_Motor: motor object motor object
    * @param mode: control mode control mode like:damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
    */
bool Motor_Control::switchControlMode(Motor &DM_Motor,Control_Mode_Code mode)
{
    uint8_t write_data[4]={(uint8_t)mode, 0x00, 0x00, 0x00};
    uint8_t RID = 10;
    write_motor_param(DM_Motor,RID,write_data);
    if (motors.find(DM_Motor.GetCanId()) == motors.end())
    {
        std::cerr << "[Error] In switchControlMode,no motor with id " << DM_Motor.GetCanId() << " is registered." << std::endl;
        std::exit(-1);  // Terminate the program
        return false;
    }
    return true;
}

/*
    * @description: change motor param Modify internal motor register parameter see the Damiao manual for the register list
    * @param DM_Motor: motor object motor object
    * @param RID: register id register ID
    * @param data: param data Parameter data. Most values are float; for uint32 values, integer input is also accepted and handled internally
    * @return: bool true or false  Whether the modification succeeded
    */
bool Motor_Control::change_motor_param(Motor &DM_Motor,uint8_t RID,float data)
{
    if(is_in_ranges(RID)) {
        //Handle integer-range inputs if they are passed in
        uint32_t data_uint32 = float_to_uint32(data);
        uint8_t *data_uint8;
        data_uint8=(uint8_t*)&data_uint32;
        write_motor_param(DM_Motor,RID,data_uint8);
    }
    else
    {
        //is float
        uint8_t *data_uint8;
        data_uint8=(uint8_t*)&data;
        write_motor_param(DM_Motor,RID,data_uint8);
    }
    if (motors.find(DM_Motor.GetCanId()) == motors.end())
    {
        std::cerr << "[Error] In change_motor_param,no motor with id666 " << DM_Motor.GetCanId() << " is registered." << std::endl;
        std::exit(-1);  // Terminate the program; a nonzero return value indicates an error
        return false;
    }
    return true;
}

/*
    * @description: change motor limit param Modify motor limit parameters. This changes the motor limits, not the internal motor register parameters
    * @param DM_Motor: motor object motor object
    * @param P_MAX: position max position maximum
    * @param Q_MAX: velocity max velocity maximum
    * @param T_MAX: torque max torque maximum
    */
void Motor_Control::changeMotorLimit(Motor &DM_Motor,float P_MAX,float Q_MAX,float T_MAX)
{
    limit_param[DM_Motor.GetMotorType()]={P_MAX,Q_MAX,T_MAX};
}

void Motor_Control::canframeCallback(const canfd_frame& frame)
{
    // Use std::lock_guard to manage mutex_ locking and unlocking automatically for thread safety
    // When the guard object is created, it locks mutex_ automatically; when the guard object is destroyed, such as when leaving scope, it unlocks mutex_ automatically
    //std::cerr<<"666"<<std::endl;
  std::lock_guard<std::mutex> guard(mutex_);
  //CanFrameStamp can_frame_stamp{ .frame = frame, .stamp = std::chrono::system_clock::now() };
  CanFrameStamp can_frame_stamp{ .frame = frame };
  read_buffer_.push_back(can_frame_stamp);
   // Note: because the std::lock_guard scope is inside the function body, when frameCallback returns,
    // the guard object is destroyed and unlocks mutex_ automatically, so no manual unlock is needed
    static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
        float span = xmax - xmin;
        float data_norm = float(x) / ((1 << bits) - 1);
        float data = data_norm * span + xmin;
        return data;
    };

  for (const auto& frame_stamp : read_buffer_)
  {
    //std::cerr<<"6www"<<std::endl;
    canfd_frame frame = frame_stamp.frame;
    // uint16_t canID = (uint16_t(frame.data[1]) << 8) | frame.data[0];
    uint16_t canID = frame.data[0]&0x0F;
    // std::cout << "CAN ID: 0x" << std::hex << std::setw(3) << std::setfill('0') << frame.can_id << std::dec;

    // // Check whether this is an extended frame
    // if (frame.can_id & CAN_EFF_FLAG) {
    //     std::cout << " (extended frame)";
    // } else {
    //     std::cout << " (standard frame)";
    // }

    // // Check whether this is a remote frame
    // if (frame.can_id & CAN_RTR_FLAG) {
    //     std::cout << " [remote request]";
    // }

    // // Print data length (DLC)
    // std::cout << " DLC: " << static_cast<int>(frame.can_dlc) << " data: ";

    // // Print data bytes (hex)
    // for (int i = 0; i < frame.can_dlc; ++i) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0')
    //               << static_cast<int>(frame.data[i]) << " " << std::dec;
    // }

    // std::cout << std::endl;
    if(read_write_save==true&& motors.find(canID) == motors.end())
    {//This is the response for save, write, or read parameter commands
        if(frame.data[2]==0x33 || frame.data[2]==0x55 || frame.data[2]==0xAA)
        {//A read-parameter or write-parameter command was sent; the response contains the corresponding register parameter
            if(frame.data[2]==0x33 || frame.data[2]==0x55)
            {
                receive_param(&frame.data[0]);
            }
            read_write_save==false;
        }

    }
    else
    {//This is normal position, velocity, and torque feedback
        //std::cerr<<"id:"<<frame.can_id<<std::endl;
        uint16_t q_uint = (uint16_t(frame.data[1]) << 8) | frame.data[2];
        uint16_t dq_uint = (uint16_t(frame.data[3]) << 4) | (frame.data[4] >> 4);
        uint16_t tau_uint = (uint16_t(frame.data[4] & 0xf) << 8) | frame.data[5];

        if(motors.find(frame.can_id) == motors.end())
        {
            return;
        }
        auto m = motors[frame.can_id];
        Limit_param limit_param_receive = m->get_limit_param();
        float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
        float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
        float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
        m->receive_data(receive_q, receive_dq, receive_tau);
       // m->frequency = 1. / (frame_stamp.stamp -  m->stamp).toSec();

        //m->stamp = frame_stamp.stamp;
        m->updateTimeInterval();

       double interval=m->getTimeInterval() ;
       //std::cerr<<"motor id is: "<<canID<<": "<<interval<<std::endl;
    }

  }
  read_buffer_.clear();

}
}
