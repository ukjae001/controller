#pragma once 
#include "fsm/TimedFSMState.h"
#include <vector>
#include <cmath>
#include <algorithm> // std::clamp 사용

class State_Squat_Down : public TimedFSMState {
public:
    State_Squat_Down(StateID id, double dur, StateID next,
                     const std::vector<float>& kp, const std::vector<float>& kd,
                     YAML::Node params) 
        : TimedFSMState(id, "SquatDown", dur, next, kp, kd) {} 

    void enter() override {
        TimedFSMState::enter();
        std::cout << "[FSM] >>> Entering SquatDown State (Memory Optimized)" << std::endl;
        _logical_time = 0.0;
    }

    void run() override {
        if (!BaseState::lowcmd || !BaseState::lowstate) return;

        auto& cmd = *(BaseState::lowcmd);
        const auto& ms = *(BaseState::lowstate);

        _logical_time += 0.002; 

        double ramp_dur = 0.5;
        double ramp_ratio = std::clamp(_logical_time / ramp_dur, 0.0, 1.0);
        double ratio = std::clamp((_logical_time - ramp_dur) / (_duration - ramp_dur), 0.0, 1.0);

        GainSet();

        for (int idx : {4, 5, 10, 11}) { // 발목
            cmd.motor_cmd().at(idx).kp() = 600.0 * ramp_ratio;
        }
        for (int idx : {3, 9}) { // 무릎
            cmd.motor_cmd().at(idx).kp() = 200.0 * ramp_ratio;
        }

        for (int i = 0; i < G1_NUM_MOTOR; ++i) { 
            double sit_q = 0.0;
            if (i == LeftElbow || i == RightElbow) {
                sit_q = 1.6;
                cmd.motor_cmd().at(i).q() = (1.0 - ratio) * sit_q;
            } 
            else if (i == LeftKnee || i == RightKnee) {
                // 무릎 중력 보상 
                cmd.motor_cmd().at(i).tau() = 9.8 * std::sin(ms.motor_state().at(i).q()); 
                sit_q = 1.9;
                cmd.motor_cmd().at(i).q() = sit_q * std::pow(ratio, 0.77);
            } 
            else if (i == LeftHipPitch || i == RightHipPitch) {
                sit_q = -1.4;
                cmd.motor_cmd().at(i).q() = sit_q * std::pow(ratio, 0.48);
            } 
            else if (i == LeftAnklePitch || i == RightAnklePitch) {
                sit_q = -0.9;
                cmd.motor_cmd().at(i).q() = std::pow(ratio, 1.0) * sit_q;
            } 
            else {
                cmd.motor_cmd().at(i).q() = 0.0;
            }
        }

        if (static_cast<int>(_logical_time * 10) % 20 == 0) {
            printf("[FSM] SquatDown: %.1f%% (Ramp: %.3fs)\n", ratio * 100.0, _logical_time);
        }
    }

    void exit() override {
        std::cout << "[FSM] <<< Exiting SquatDown State" << std::endl;
    }

private:
    double _logical_time;
};
