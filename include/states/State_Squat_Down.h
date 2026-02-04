#pragma once 
#include "fsm/TimedFSMState.h"
#include <vector>
#include <cmath>
#include <algorithm> // std::clamp 사용

class State_Squat_Down : public TimedFSMState {
public:
    // FSM에서 전달하는 kp, kd 리스트를 생성자 인자로 추가합니다.
    State_Squat_Down(StateID id, double dur, StateID next,
                     const std::vector<float>& kp, const std::vector<float>& kd,
                     YAML::Node params) 
        : TimedFSMState(id, "SquatDown", dur, next, kp, kd) {} // 부모 생성자로 전달

    void enter() override {
        // 부모의 enter() 호출 (시간 측정 시작)
        TimedFSMState::enter();
        std::cout << "[FSM] >>> Entering SquatDown State (Memory Optimized)" << std::endl;
        _logical_time = 0.0;
        // 더 이상 YAML을 여기서 파싱하지 않습니다.
    }

    void run() override {
        // lowcmd나 lowstate가 초기화되지 않았으면 실행하지 않음
        if (!BaseState::lowcmd || !BaseState::lowstate) return;

        auto& cmd = *(BaseState::lowcmd);
        const auto& ms = *(BaseState::lowstate);

        _logical_time += 0.002; // 500Hz 루프 기준

        // 1. Soft Start 및 동작 비율 계산
        double ramp_dur = 0.5;
        double ramp_ratio = std::clamp(_logical_time / ramp_dur, 0.0, 1.0);
        double ratio = std::clamp((_logical_time - ramp_dur) / (_duration - ramp_dur), 0.0, 1.0);

        // 2. 전신 기본 초기화 (부모 클래스의 kp_list, kd_list 참조 사용)
        GainSet();

        // 3. 하체 강화 (Ramp 적용)
        for (int idx : {4, 5, 10, 11}) { // 발목
            cmd.motor_cmd().at(idx).kp() = 600.0 * ramp_ratio;
        }
        for (int idx : {3, 9}) { // 무릎
            cmd.motor_cmd().at(idx).kp() = 200.0 * ramp_ratio;
        }

        // 4. 동작 궤적 계산
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