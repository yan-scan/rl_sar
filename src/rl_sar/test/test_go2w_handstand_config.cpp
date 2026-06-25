#include <cassert>
#include <iostream>
#include <string>
#include <vector>

#include "fsm_go2w.hpp"
#include "rl_sdk.hpp"

namespace
{
class DummyRL final : public RL
{
public:
    std::vector<float> Forward() override
    {
        return {};
    }

    void GetState(RobotState<float> *state) override
    {
        (void)state;
    }

    void SetCommand(const RobotCommand<float> *command) override
    {
        (void)command;
    }
};

void ExpectEqual(const std::vector<int> &actual, const std::vector<int> &expected, const std::string &label)
{
    if (actual != expected)
    {
        std::cerr << label << " mismatch.\nactual:   " << actual << "\nexpected: " << expected << std::endl;
        std::abort();
    }
}

void ExpectEqual(const std::vector<std::string> &actual, const std::vector<std::string> &expected, const std::string &label)
{
    if (actual != expected)
    {
        std::cerr << label << " mismatch." << std::endl;
        std::cerr << "actual:   [";
        for (size_t i = 0; i < actual.size(); ++i)
        {
            std::cerr << actual[i] << (i + 1 == actual.size() ? "" : ", ");
        }
        std::cerr << "]\nexpected: [";
        for (size_t i = 0; i < expected.size(); ++i)
        {
            std::cerr << expected[i] << (i + 1 == expected.size() ? "" : ", ");
        }
        std::cerr << "]" << std::endl;
        std::abort();
    }
}
} // namespace

void VerifyConfig(
    const std::string &config_path,
    const std::vector<std::string> &expected_observations,
    const std::vector<int> &expected_dims,
    int expected_num_observations,
    bool expect_root_pos_rel_xy_b)
{
    DummyRL rl;
    rl.ReadYaml(config_path, "config.yaml");
    rl.InitJointNum(rl.params.Get<int>("num_of_dofs"));
    rl.InitObservations();

    const auto observations = rl.params.Get<std::vector<std::string>>("observations");
    ExpectEqual(observations, expected_observations, "observations");

    const int configured_num_observations = rl.params.Get<int>("num_observations");
    if (configured_num_observations != expected_num_observations)
    {
        std::cerr << "num_observations mismatch. actual: " << configured_num_observations
                  << " expected: " << expected_num_observations << std::endl;
        std::abort();
    }

    rl.obs.lin_vel = {0.1f, -0.2f, 0.3f};
    rl.obs.ang_vel = {0.4f, -0.5f, 0.6f};
    rl.obs.commands = {0.0f, 0.0f, 0.0f};
    if (expect_root_pos_rel_xy_b)
    {
        rl.obs.root_pos_rel_xy_b = {0.02f, -0.03f};
    }

    const std::vector<float> obs = rl.ComputeObservation();
    ExpectEqual(rl.obs_dims, expected_dims, "obs_dims");
    assert(static_cast<int>(obs.size()) == expected_num_observations);
}

int main()
{
    assert(go2w_fsm::ResolveGo2WConfigAlias("") == "robot_lab_handstand");
    assert(go2w_fsm::ResolveGo2WConfigAlias("go2w_handstand") == "robot_lab_handstand");
    assert(go2w_fsm::ResolveGo2WConfigAlias("go2w_line_run") == "robot_lab_line_run");
    assert(go2w_fsm::ResolveGo2WModeStateNameIfKnown("robot_lab_handstand") == "RLFSMStateRLHandstand");
    assert(go2w_fsm::ResolveGo2WModeStateNameIfKnown("robot_lab_line_run") == "RLFSMStateRLLineRun");
    assert(go2w_fsm::ResolveGo2WModeStateNameIfKnown("custom_mode").empty());
    assert(go2w_fsm::IsGo2WHandstandSwitch(Input::Keyboard::Num2, Input::Gamepad::None));
    assert(go2w_fsm::IsGo2WHandstandSwitch(Input::Keyboard::None, Input::Gamepad::RB_DPadRight));
    assert(go2w_fsm::IsGo2WLineRunSwitch(Input::Keyboard::Num3, Input::Gamepad::None));
    assert(go2w_fsm::IsGo2WLineRunSwitch(Input::Keyboard::None, Input::Gamepad::RB_DPadLeft));

    VerifyConfig(
        "go2w/robot_lab_handstand",
        {"lin_vel", "ang_vel", "gravity_vec", "commands", "dof_pos", "dof_vel", "actions", "root_pos_rel_xy_b"},
        {3, 3, 3, 3, 16, 16, 16, 2},
        62,
        true
    );

    VerifyConfig(
        "go2w/robot_lab_line_run",
        {"ang_vel", "gravity_vec", "commands", "dof_pos", "dof_vel", "actions"},
        {3, 3, 3, 16, 16, 16},
        57,
        false
    );

    std::cout << "Go2W handstand and line-run configs load independently, including aliases." << std::endl;
    return 0;
}
