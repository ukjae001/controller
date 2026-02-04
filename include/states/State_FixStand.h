#pragma once 
#include "fsm/TimedFSMState.h"
#include <vector>

class State_FixStand : public TimedFSMState {
public:
    // 🔥 에러 지점: 인자를 5개(id, dur, next, kp, kd)로 늘려야 합니다.
    State_FixStand(StateID id, double dur, StateID next,
                   const std::vector<float>& kp, const std::vector<float>& kd,
                   YAML::Node params) 
        : TimedFSMState(id, "FixStand", dur, next, kp, kd) {} // 부모에게 5개 전달

    void enter() override {
        TimedFSMState::enter();
        std::cout << "[FSM] >>> Entering FixStand State (Posture Maintenance)" << std::endl;
        // 이제 여기서 YAML 읽는 코드가 있다면 다 지워버리세요!
    }

    void run() override {
        if (!BaseState::lowcmd || !BaseState::lowstate) return;

        // 1. 부모의 GainSet 호출 (전신 게인 초기화)
        GainSet();

        auto& cmd = *(BaseState::lowcmd);

        // 2. 발목 강성 유지 (특정 관절 오버라이드)
        for (int idx : {4, 5, 10, 11}) {
            cmd.motor_cmd().at(idx).kp() = 800.0f;
            cmd.motor_cmd().at(idx).kd() = 40.0f;
        }

        // 3. 차렷 자세 유지 로직
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            if (i == LeftElbow || i == RightElbow) {
                cmd.motor_cmd().at(i).q() = 1.6f;
            } 
            else if (i == LeftAnklePitch || i == RightAnklePitch) {
                cmd.motor_cmd().at(i).q() = 0.01f;
            } 
            else {
                cmd.motor_cmd().at(i).q() = 0.0f;
            }
        }
    }

    void exit() override {
        std::cout << "[FSM] <<< Exiting FixStand State" << std::endl;
    }
};