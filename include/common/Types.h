#pragma once
#include <string>
#include <vector>

enum class StateID {
    Passive = 0,
    FixStand,
    SquatDown,
    SquatUp,
    Hello
};

enum G1JointIndex {
    LeftHipPitch = 0, 
    LeftHipRoll = 1, 
    LeftHipYaw = 2, 
    LeftKnee = 3, 
    LeftAnklePitch = 4, 
    LeftAnkleRoll = 5,
    RightHipPitch = 6, 
    RightHipRoll = 7, 
    RightHipYaw = 8, 
    RightKnee = 9, 
    RightAnklePitch = 10, 
    RightAnkleRoll = 11,
    WaistYaw = 12, 
    WaistRoll = 13, 
    WaistPitch = 14,
    LeftShoulderPitch = 15, 
    LeftShoulderRoll = 16, 
    LeftShoulderYaw = 17, 
    LeftElbow = 18,
    LeftWristRoll = 19, 
    LeftWristPitch = 20, 
    LeftWristYaw = 21,
    RightShoulderPitch = 22, 
    RightShoulderRoll = 23, 
    RightShoulderYaw = 24, 
    RightElbow = 25,
    RightWristRoll = 26, 
    RightWristPitch = 27, 
    RightWristYaw = 28
};

const int G1_NUM_MOTOR = 29;