/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GO2W_FSM_HPP
#define GO2W_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

namespace go2w_fsm
{

inline constexpr const char *kGo2WHandstandConfig = "robot_lab_handstand";
inline constexpr const char *kGo2WLineRunConfig = "robot_lab_line_run";
inline constexpr const char *kGo2WRLLocomotionState = "RLFSMStateRLLocomotion";
inline constexpr const char *kGo2WRLHandstandState = "RLFSMStateRLHandstand";
inline constexpr const char *kGo2WRLLineRunState = "RLFSMStateRLLineRun";

inline std::string ResolveGo2WConfigAlias(const std::string& requested)
{
    if (requested.empty())
    {
        return kGo2WHandstandConfig;
    }
    if (requested == "handstand" || requested == "go2w_handstand" || requested == kGo2WHandstandConfig)
    {
        return kGo2WHandstandConfig;
    }
    if (requested == "line_run" || requested == "go2w_line_run" || requested == kGo2WLineRunConfig)
    {
        return kGo2WLineRunConfig;
    }
    return requested;
}

inline std::string ResolveGo2WModeStateNameIfKnown(const std::string &requested)
{
    const std::string resolved = ResolveGo2WConfigAlias(requested);
    if (resolved == kGo2WHandstandConfig)
    {
        return kGo2WRLHandstandState;
    }
    if (resolved == kGo2WLineRunConfig)
    {
        return kGo2WRLLineRunState;
    }
    return "";
}

inline bool IsGo2WHandstandSwitch(Input::Keyboard keyboard, Input::Gamepad gamepad)
{
    return keyboard == Input::Keyboard::Num2 || gamepad == Input::Gamepad::RB_DPadRight;
}

inline bool IsGo2WLineRunSwitch(Input::Keyboard keyboard, Input::Gamepad gamepad)
{
    return keyboard == Input::Keyboard::Num3 || gamepad == Input::Gamepad::RB_DPadLeft;
}

inline bool LoadGo2WPolicy(RLFSMState &state, const std::string &requested_config_name)
{
    state.rl.episode_length_buf = 0;
    state.rl.requested_config_name = requested_config_name;
    state.rl.config_name = ResolveGo2WConfigAlias(state.rl.requested_config_name);

    std::string robot_config_path = state.rl.robot_name + "/" + state.rl.config_name;
    try
    {
        state.rl.InitRL(robot_config_path);
        state.rl.OnPolicyActivated(state.rl.config_name);
        state.rl.now_state = *state.fsm_state;
        return true;
    }
    catch (const std::exception &e)
    {
        std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
        state.rl.rl_init_done = false;
        state.rl.fsm.RequestStateChange("RLFSMStatePassive");
        return false;
    }
}

inline void RunGo2WPolicy(RLFSMState &state)
{
    if (!state.rl.rl_init_done)
    {
        state.rl.rl_init_done = true;
    }

    std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << state.rl.config_name
              << "] x:" << state.rl.control.x << " y:" << state.rl.control.y
              << " yaw:" << state.rl.control.yaw << std::flush;
    state.RLControl();
}

inline std::string CheckGo2WPolicyTransition(
    RL &rl,
    const std::string &state_name,
    bool allow_requested_locomotion_entry = false)
{
    if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
    {
        return "RLFSMStatePassive";
    }
    if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
    {
        return "RLFSMStateGetDown";
    }
    if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
    {
        return "RLFSMStateGetUp";
    }
    if (IsGo2WHandstandSwitch(rl.control.current_keyboard, rl.control.current_gamepad))
    {
        return kGo2WRLHandstandState;
    }
    if (IsGo2WLineRunSwitch(rl.control.current_keyboard, rl.control.current_gamepad))
    {
        return kGo2WRLLineRunState;
    }
    if (allow_requested_locomotion_entry &&
        (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp))
    {
        return kGo2WRLLocomotionState;
    }
    return state_name;
}

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 8;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    std::vector<float> pre_running_pos = {
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 0.00, 0.00, 0.00
    };
    bool stand_from_passive = true;

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        if(stand_from_passive)
        {

            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 1.0f, "Pre Getting up", true)) return;
            if (Interpolate(percent_getup, pre_running_pos, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true)) return;
        }
        else
        {
            if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 1.0f, "Getting up", true)) return;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return kGo2WRLLocomotionState;
            }
            else if (IsGo2WHandstandSwitch(rl.control.current_keyboard, rl.control.current_gamepad))
            {
                return kGo2WRLHandstandState;
            }
            else if (IsGo2WLineRunSwitch(rl.control.current_keyboard, rl.control.current_gamepad))
            {
                return kGo2WRLLineRunState;
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, kGo2WRLLocomotionState) {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;

        const std::string known_mode_state = ResolveGo2WModeStateNameIfKnown(rl.requested_config_name);
        if (!known_mode_state.empty())
        {
            rl.fsm.RequestStateChange(known_mode_state);
            return;
        }

        LoadGo2WPolicy(*this, rl.requested_config_name);
    }

    void Run() override
    {
        RunGo2WPolicy(*this);
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        return CheckGo2WPolicyTransition(rl, state_name_);
    }
};

class RLFSMStateGo2WModeBase : public RLFSMState
{
public:
    RLFSMStateGo2WModeBase(RL *rl, const std::string &name, const std::string &requested_config_name)
        : RLFSMState(*rl, name), requested_config_name_(requested_config_name) {}

    void Enter() override
    {
        LoadGo2WPolicy(*this, requested_config_name_);
    }

    void Run() override
    {
        RunGo2WPolicy(*this);
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

protected:
    std::string requested_config_name_;
};

class RLFSMStateRLLineRun : public RLFSMStateGo2WModeBase
{
public:
    RLFSMStateRLLineRun(RL *rl)
        : RLFSMStateGo2WModeBase(rl, kGo2WRLLineRunState, kGo2WLineRunConfig) {}

    std::string CheckChange() override
    {
        return CheckGo2WPolicyTransition(rl, state_name_);
    }
};

class RLFSMStateRLHandstand : public RLFSMStateGo2WModeBase
{
public:
    RLFSMStateRLHandstand(RL *rl)
        : RLFSMStateGo2WModeBase(rl, kGo2WRLHandstandState, kGo2WHandstandConfig) {}

    std::string CheckChange() override
    {
        return CheckGo2WPolicyTransition(rl, state_name_);
    }
};

} // namespace go2w_fsm

class Go2WFSMFactory : public FSMFactory
{
public:
    Go2WFSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<go2w_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<go2w_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<go2w_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == go2w_fsm::kGo2WRLLocomotionState)
            return std::make_shared<go2w_fsm::RLFSMStateRLLocomotion>(rl);
        else if (state_name == go2w_fsm::kGo2WRLLineRunState)
            return std::make_shared<go2w_fsm::RLFSMStateRLLineRun>(rl);
        else if (state_name == go2w_fsm::kGo2WRLHandstandState)
            return std::make_shared<go2w_fsm::RLFSMStateRLHandstand>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "go2w"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            go2w_fsm::kGo2WRLLocomotionState,
            go2w_fsm::kGo2WRLLineRunState,
            go2w_fsm::kGo2WRLHandstandState
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(Go2WFSMFactory, "RLFSMStatePassive")

#endif // GO2W_FSM_HPP
