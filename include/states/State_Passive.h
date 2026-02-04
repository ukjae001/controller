#pragma once 
#include "fsm/BaseState.h"
#include <iostream>
#include <vector>

class State_Passive : public BaseState {
public:
    State_Passive(StateID id) 
        : BaseState(id, "Passive") {}

    void enter() override {
        std::cout << "[FSM] >>> Entering Passive State: Zero Gains Applied" << std::endl;
    }

    void run() override {
        if (!BaseState::lowcmd) return;
        auto& cmd = *(BaseState::lowcmd);

        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            cmd.motor_cmd().at(i).q() = 0.0;
            cmd.motor_cmd().at(i).dq() = 0.0;
            cmd.motor_cmd().at(i).kp() = 0.0;
            cmd.motor_cmd().at(i).kd() = 0.0;
            cmd.motor_cmd().at(i).tau() = 0.0;
        }
    }

    void exit() override {
        std::cout << "[FSM] <<< Exiting Passive State" << std::endl;
    }

    bool isFinished() const override { return false; }
    StateID getNextState() const override { return state_id; }
};
