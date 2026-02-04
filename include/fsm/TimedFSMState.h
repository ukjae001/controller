#pragma once
#include "BaseState.h"
#include <chrono>
#include <vector>

class TimedFSMState : public BaseState {
public:
    // 생성자에서 kp, kd 참조를 추가로 받습니다.
    TimedFSMState(StateID id, std::string name, double duration, StateID next,
                  const std::vector<float>& kp, const std::vector<float>& kd)
        : BaseState(id, name), _duration(duration), _next_state(next), 
          kp_list(kp), kd_list(kd) {} // 여기서 참조 복사

    void enter() override {
        _start_time = std::chrono::steady_clock::now();
        std::cout << "[FSM] >>> Entering State: " << state_name << std::endl;
    }

    bool isFinished() const override { return getElapsedSeconds() >= _duration; }
    StateID getNextState() const override { return _next_state; }

protected:
    double getElapsedSeconds() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - _start_time).count();
    }

    double _duration;
    StateID _next_state;
    std::chrono::steady_clock::time_point _start_time;

    void GainSet() {
        if (!lowcmd) return;
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            BaseState::lowcmd->motor_cmd().at(i).kp() = kp_list.at(i);
            BaseState::lowcmd->motor_cmd().at(i).kd() = kd_list.at(i);
            BaseState::lowcmd->motor_cmd().at(i).dq() = 0.0;
            BaseState::lowcmd->motor_cmd().at(i).tau() = 0.0;
        }
    }
    
    // 🔥 중요: 실제 vector가 아니라 '참조'입니다.
    const std::vector<float>& kp_list; 
    const std::vector<float>& kd_list;
};