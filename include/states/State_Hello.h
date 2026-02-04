#pragma once 
#include "fsm/TimedFSMState.h"
#include <vector>
#include <cmath>
#include <algorithm>

class State_Hello : public TimedFSMState {
public:
    State_Hello(StateID id, double dur, StateID next,
                const std::vector<float>& kp, const std::vector<float>& kd, 
                YAML::Node params) 
        : TimedFSMState(id, "Hello", dur, next, kp, kd) {
        ankle_kp = params["ankle_kp"].as<double>(900.0);
        ankle_kd = params["ankle_kd"].as<double>(40.0);
        elbow_q = params["elbow_q"].as<double>(1.6);
        hip_q = params["hip_q"].as<double>(-0.25);
        knee_q = params["knee_q"].as<double>(0.1);
        waist_pitch = params["waist_pitch"].as<double>(-0.175);
        waist_roll = params["waist_roll"].as<double>(-0.6); 
        shoulder_roll = params["shoulder_roll"].as<double>(-2.3); 
        shoulder_yaw = params["shoulder_yaw"].as<double>(-1.57); 
        ankle_roll = params["ankle_roll"].as<double>(-0.08); 
        ankle_pitch = params["ankle_pitch"].as<double>(0.01); 
        }

    void enter() override {
        TimedFSMState::enter();
        std::cout << "[FSM] >>> Entering State: Hello" << std::endl;
    }

    void run() override {
        if (!BaseState::lowcmd || !BaseState::lowstate) return;

        auto& cmd = *(BaseState::lowcmd);
        double currentTime = getElapsedSeconds();

        GainSet();

        for (int idx : {4, 5, 10, 11}) {
            cmd.motor_cmd().at(idx).kp() = ankle_kp; //900.0f;
            cmd.motor_cmd().at(idx).kd() = ankle_kd; //40.0f;
        }

        double ratio = std::clamp(currentTime / _duration, 0.0, 1.0);
        double smooth_ratio = (1.0 - std::cos(ratio * M_PI)) / 2.0;

        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            double hello_q = 0.0;
            
            if (i == LeftElbow || i == RightElbow) {
                cmd.motor_cmd().at(i).q() = elbow_q; //1.6f;
            } 
            else if (i == LeftHipPitch || i == RightHipPitch) {
                //double hello_q = -0.25 * std::sin(smooth_ratio * M_PI);
                cmd.motor_cmd().at(i).q() = hip_q * std::sin(smooth_ratio * M_PI); 
            } 
            else if (i == LeftKnee || i == RightKnee) {
                //double hello_q = 0.1 * std::sin(smooth_ratio * M_PI);
                cmd.motor_cmd().at(i).q() = knee_q * std::sin(smooth_ratio * M_PI);
            } 
            else if (i == WaistPitch) {
                //hello_q = -0.175; 
                cmd.motor_cmd().at(i).q() = waist_pitch * std::sin(smooth_ratio * M_PI);
            }
            else if (i == WaistRoll) {
                //hello_q = -0.6; 
                cmd.motor_cmd().at(i).q() = waist_roll * std::sin(smooth_ratio * M_PI); 
            }
            else if (i == RightShoulderRoll) {
                //hello_q = -2.3;
                cmd.motor_cmd().at(i).q() = shoulder_roll * std::sin(smooth_ratio * M_PI); 
            }
            else if (i == RightShoulderYaw) {
                //hello_q = -1.57; 
                cmd.motor_cmd().at(i).q() = shoulder_yaw * std::sin(smooth_ratio * M_PI);
            }
            else if (i == LeftAnkleRoll || i == RightAnkleRoll) {
                //hello_q = -0.08; 
                //double hello_q = -0.08 * std::sin(smooth_ratio * M_PI);
                cmd.motor_cmd().at(i).q() = ankle_roll * std::sin(smooth_ratio * M_PI);
            }
            else if (i == LeftAnklePitch || i == RightAnklePitch) {
                //hello_q = 0.01;
                cmd.motor_cmd().at(i).q() = ankle_pitch * smooth_ratio;
            }
            else {
                cmd.motor_cmd().at(i).q() = 0.0f;
            }
        }

        if (static_cast<int>(currentTime * 10) % 20 == 0) {
            printf("[FSM] Hello: %.1f%% (Real Time: %.3fs)\n", ratio * 100.0, currentTime);
        }
    }

    void exit() override {
        std::cout << "[FSM] <<< Hello Motion Finished" << std::endl;
    }

private:
    double ankle_kp, ankle_kd;
    double elbow_q, hip_q, knee_q;
    double waist_pitch, waist_roll;
    double shoulder_roll, shoulder_yaw;
    double ankle_roll, ankle_pitch;
};
