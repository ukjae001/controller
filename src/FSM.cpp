#include "fsm/FSM.h"
#include "fsm/BaseState.h"
#include <unitree/robot/channel/channel_factory.hpp>
#include <yaml-cpp/yaml.h>

// 상태 헤더 포함
#include "states/State_FixStand.h"
#include "states/State_Squat_Down.h"
#include "states/State_Squat_Up.h"
#include "states/State_Hello.h"
#include "states/State_Passive.h"

// 정적 변수 정의
LowCmd_t* BaseState::lowcmd = nullptr;
const LowState_t* BaseState::lowstate = nullptr;

// 헬퍼 함수
StateID stringToStateID(std::string id) {
    if (id == "FixStand") return StateID::FixStand;
    if (id == "SquatDown") return StateID::SquatDown;
    if (id == "SquatUp") return StateID::SquatUp;
    if (id == "Hello") return StateID::Hello;
    if (id == "Passive") return StateID::Passive;
    return StateID::Passive;
}

FSM::FSM(std::string networkInterface) 
    : _current_state(nullptr), _first_state_id(StateID::Passive) {
    
    // 1. 커맨드 메시지 초기화
    _lowcmd_msg = std::make_unique<LowCmd_>();
    BaseState::lowcmd = _lowcmd_msg.get();
    InitLowCmd();

    // 2. 통신 초기화
    unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);
    _lowcmd_pub.reset(new unitree::robot::ChannelPublisher<LowCmd_>("rt/lowcmd"));
    _lowcmd_pub->InitChannel();
    _lowstate_sub.reset(new unitree::robot::ChannelSubscriber<LowState_>("rt/lowstate"));
    _lowstate_sub->InitChannel(std::bind(&FSM::LowStateHandler, this, std::placeholders::_1), 1);

    // 3. YAML 로드 (여기서 딱 한 번만 수행)
    try {
        YAML::Node config = YAML::LoadFile("config.yaml");
        
        // 기본 게인 먼저 파싱
        _default_kp = config["default_gain"]["kp"].as<std::vector<float>>();
        _default_kd = config["default_gain"]["kd"].as<std::vector<float>>();

        const auto& seq = config["sequence"];
        for (const auto& node : seq) {
            std::string id_str = node["id"].as<std::string>();
            double dur = node["duration"] ? node["duration"].as<double>() : 0.0;
            std::string next_str = node["next"] ? node["next"].as<std::string>() : "Passive";
            
            YAML::Node params;
            if (node["params"]) {params = node["params"];}
             else {params = YAML::Load("{}");}

            StateID id = stringToStateID(id_str);
            StateID next = stringToStateID(next_str);

            // 🔥 State 생성 시 _default_kp, _default_kd를 인자로 넘겨줌
            if (id == StateID::FixStand) 
                _state_map[id] = std::make_shared<State_FixStand>(id, dur, next, _default_kp, _default_kd, params);
            else if (id == StateID::SquatDown) 
                _state_map[id] = std::make_shared<State_Squat_Down>(id, dur, next, _default_kp, _default_kd, params);
            else if (id == StateID::SquatUp) 
                _state_map[id] = std::make_shared<State_Squat_Up>(id, dur, next, _default_kp, _default_kd, params);
            else if (id == StateID::Hello) 
                _state_map[id] = std::make_shared<State_Hello>(id, dur, next, _default_kp, _default_kd, params);
            else if (id == StateID::Passive) 
                _state_map[id] = std::make_shared<State_Passive>(id);

            if (_state_map.size() == 1) _first_state_id = id;
        }
    } catch (const std::exception& e) {
        std::cerr << "[FSM] Config Load Error: " << e.what() << std::endl;
        // 실패 시 기본값 강제 할당 (안전장치)
        _default_kp.assign(G1_NUM_MOTOR, 60.0f);
        _default_kd.assign(G1_NUM_MOTOR, 2.0f);
        _state_map[StateID::Passive] = std::make_shared<State_Passive>(StateID::Passive);
        _first_state_id = StateID::Passive;
    }
}

FSM::~FSM() {}

void FSM::start() {
    if (_state_map.count(_first_state_id)) {
        _current_state = _state_map[_first_state_id];
        _current_state->enter();
    }
}

void FSM::update() {
    if (BaseState::lowstate == nullptr || !_current_state) return;
    _current_state->run();
    if (_current_state->isFinished()) {
        StateID next_id = _current_state->getNextState();
        if (_state_map.count(next_id)) {
            _current_state->exit();
            _current_state = _state_map[next_id];
            _current_state->enter();
        }
    }
    _lowcmd_pub->Write(*_lowcmd_msg);
}

void FSM::LowStateHandler(const void* message) {
    const LowState_t* current_lowstate = static_cast<const LowState_t*>(message);
    BaseState::lowstate = current_lowstate;
     
    static float last_p = 0.0;
    float current_p = current_lowstate->imu_state().rpy()[1];

    if (std::abs(current_p - last_p) > 0.3 && std::abs(current_p) <= 0.02) {
        std::cout << "[FSM] Reset Detected! Jumping to first state..." << std::endl;
        if (_current_state) _current_state->exit();
        _current_state = _state_map[_first_state_id];
        _current_state->enter();
    }
    last_p = current_p;
}


void FSM::InitLowCmd() {
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        auto& m = _lowcmd_msg->motor_cmd()[i];
        m.mode() = 0x01; m.q() = 0.0; m.dq() = 0.0; m.kp() = 0.0; m.kd() = 0.0; m.tau() = 0.0;
    }
}