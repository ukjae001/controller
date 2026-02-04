#pragma once 
#include "fsm/TimedFSMState.h"
#include <vector>
#include <cmath>
#include <algorithm>

class State_Squat_Up : public TimedFSMState {
public:
    // 1. 생성자에서 FSM으로부터 게인 리스트 참조를 받도록 수정
    State_Squat_Up(StateID id, double dur, StateID next,
                   const std::vector<float>& kp, const std::vector<float>& kd,
                  YAML::Node params) 
        : TimedFSMState(id, "SquatUp", dur, next, kp, kd) {}

    void enter() override {
        // TimedFSMState::enter()에서 _start_time이 초기화됨
        TimedFSMState::enter();
        std::cout << "[FSM] >>> Entering SquatUp State (Real-time mode)" << std::endl;
    }

    void run() override {
        if (!BaseState::lowcmd || !BaseState::lowstate) return;

        auto& cmd = *(BaseState::lowcmd);
        const auto& ms = *(BaseState::lowstate);

        // 실제 경과 시간 사용
        double currentTime = getElapsedSeconds();
        double ratio = std::clamp(currentTime / _duration, 0.0, 1.0);

        // 2. 전신 기본 초기화 (kp_list_ -> 부모 클래스의 kp_list 참조 사용)
        GainSet();

        // 3. 발목 및 무릎 강성 스케줄링 (일어날 때 지지력 강화)
        double kpd_ratio = std::clamp(currentTime / 3.0, 0.0, 1.0);
        for (int idx : {4, 5, 10, 11}) { // 발목
            cmd.motor_cmd().at(idx).kp() = 600.0 + 200.0 * kpd_ratio;
            cmd.motor_cmd().at(idx).kd() = 2.0 + 38.0 * kpd_ratio;
        }
        for (int idx : {3, 9}) { // 무릎
            cmd.motor_cmd().at(idx).kp() = 200.0 * (1.0 - kpd_ratio * 0.2);
        }

        // 4. 동작 궤적 계산
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            double stand_q = 0.0;

            if (i == LeftElbow || i == RightElbow) {
                stand_q = 1.55;
                cmd.motor_cmd().at(i).q() = std::pow(ratio, 0.75) * stand_q;
            } 
            else if (i == LeftKnee || i == RightKnee) {
                stand_q = 1.9;
                cmd.motor_cmd().at(i).q() = (1.0 - std::pow(ratio, 1.55)) * stand_q;

                // 일어설 때 중력 보상 토크 (피드포워드 제어)
                double gravity_k = 15.0; 
                cmd.motor_cmd().at(i).tau() = gravity_k * std::sin(ms.motor_state().at(i).q());
            } 
            else if (i == LeftAnklePitch || i == RightAnklePitch) {
                stand_q = -0.9;
                cmd.motor_cmd().at(i).q() = (1.0 - ratio) * stand_q + ratio * (0.01);
            } 
            else if (i == LeftHipPitch || i == RightHipPitch) {
                // 상체 지연 해제 로직 (중심을 뒤에 두다가 나중에 일어남)
                stand_q = -1.55;
                double delayed_ratio = std::clamp((ratio - 0.6) / 0.4, 0.0, 1.0);
                cmd.motor_cmd().at(i).q() = (1.0 - std::pow(delayed_ratio, 0.9)) * stand_q;
            } 
            else {
                cmd.motor_cmd().at(i).q() = 0.0;
            }
        }

        if (static_cast<int>(currentTime * 10) % 20 == 0) {
            printf("[FSM] SquatUp: %.1f%% (Real Time: %.3fs)\n", ratio * 100.0, currentTime);
        }
    }

    void exit() override {
        std::cout << "[FSM] <<< Exiting SquatUp State" << std::endl;
    }
};