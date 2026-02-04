#pragma once
#include <iostream>
#include <string>
#include <memory>
#include "common/Types.h"

// Forward declarations for Unitree SDK2
namespace unitree_hg { namespace msg { namespace dds_ { 
    class LowCmd_; class LowState_; 
}}}

using LowCmd_t = unitree_hg::msg::dds_::LowCmd_;
using LowState_t = unitree_hg::msg::dds_::LowState_;

class BaseState {
public:
    BaseState(StateID id, std::string name) : state_id(id), state_name(name) {}
    virtual ~BaseState() = default;

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual bool isFinished() const = 0;
    virtual StateID getNextState() const = 0;

    const StateID state_id;
    const std::string state_name;

    static LowCmd_t* lowcmd;
    static const LowState_t* lowstate;
};

