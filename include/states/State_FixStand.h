#pragma once 
#include "fsm/TimedFSMState.h"
#include <vector>

class State_FixStand : public TimedFSMState {
public:
    State_FixStand(StateID id, double dur, StateID next,
                   const std::vector<float>& kp, const std::vector<float>& kd,
                   YAML::Node params) 
        : TimedFSMState(id, "FixStand", dur, next, kp, kd) {} // 부모에게 5개 전달

    void enter() override {
        TimedFSMState::enter();
        std::cout << "[FSM] >>> Entering FixStand State (Posture Maintenance)" << std::endl;
    }

    void run() override {
        if (!BaseState::lowcmd || !BaseState::lowstate) return;

        GainSet();

        auto& cmd = *(BaseState::lowcmd);

        for (int idx : {4, 5, 10, 11}) {
            cmd.motor_cmd().at(idx).kp() = 800.0f;
            cmd.motor_cmd().at(idx).kd() = 40.0f;
        }

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
