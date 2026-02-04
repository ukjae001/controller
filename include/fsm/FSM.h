#pragma once

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <iostream>

// Unitree SDK2 DDS 관련 헤더
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include "common/Types.h"
#include "fsm/BaseState.h"

// 네임스페이스 별칭 설정
using namespace unitree_hg::msg::dds_;

// 🔥 헬퍼 함수 선언 (FSM.cpp에서 정의한 함수를 헤더에서도 알 수 있게 함)
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
    
    std::vector<float> _default_kp; // YAML에서 읽어온 기본 게인 저장소
    std::vector<float> _default_kd;

    void InitLowCmd();
};