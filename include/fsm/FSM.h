#pragma once

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <iostream>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include "common/Types.h"
#include "fsm/BaseState.h"

using namespace unitree_hg::msg::dds_;

StateID stringToStateID(std::string id);

class FSM {
public:
    FSM(std::string networkInterface);
    ~FSM();

    void start();  
    void update(); 

private:
    void LowStateHandler(const void* message);

    StateID _first_state_id;
    std::map<StateID, std::shared_ptr<BaseState>> _state_map;
    std::shared_ptr<BaseState> _current_state;

    unitree::robot::ChannelPublisherPtr<LowCmd_> _lowcmd_pub;
    unitree::robot::ChannelSubscriberPtr<LowState_> _lowstate_sub;
    
    std::unique_ptr<LowCmd_> _lowcmd_msg;
    
    std::vector<float> _default_kp; 
    std::vector<float> _default_kd;

    void InitLowCmd();
};
