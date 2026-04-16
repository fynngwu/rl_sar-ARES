#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <sstream>
#include <filesystem>

#include "inference_runtime.hpp"
#include "observation_buffer.hpp"

static std::string format_vec(const std::vector<float>& v, int max_n = 12)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4) << "[";
    int n = std::min((int)v.size(), max_n);
    for (int i = 0; i < n; ++i)
    {
        if (i > 0) oss << ", ";
        oss << v[i];
    }
    if ((int)v.size() > max_n) oss << ", ...";
    oss << "](" << v.size() << ")";
    return oss.str();
}

int main(int argc, char** argv)
{
    std::string policy_dir = std::string(POLICY_DIR) + "/ares_himloco";
    std::string model_path = policy_dir + "/himloco/policy.onnx";

    if (argc >= 2) model_path = argv[1];

    std::cout << "=== ARES HIMloco ONNX Smoke Test ===" << std::endl;
    std::cout << "Model path: " << model_path << std::endl;

    if (!std::filesystem::exists(model_path))
    {
        std::cerr << "ERROR: model not found: " << model_path << std::endl;
        return 1;
    }

    // ---------- Load model ----------
    auto model = InferenceRuntime::ModelFactory::load_model(model_path);
    if (!model)
    {
        std::cerr << "ERROR: failed to load model" << std::endl;
        return 1;
    }
    std::cout << "Model loaded, type: " << model->get_model_type() << std::endl;

    // ---------- Config (mirrors config.yaml) ----------
    int num_dofs = 12;
    float lin_vel_scale = 2.0f;
    float ang_vel_scale = 0.25f;
    float dof_pos_scale = 1.0f;
    float dof_vel_scale = 0.05f;
    float gravity_vec_scale = 1.0f;
    float clip_obs = 100.0f;
    float action_scale = 0.25f;

    std::vector<float> default_dof_pos = {
        0.0f, 0.55f, -0.95f,
        0.0f, 0.55f, -0.95f,
        0.0f, -0.55f, 0.95f,
        0.0f, -0.55f, 0.95f
    };

    std::vector<float> commands_scale = {2.0f, 2.0f, 0.25f};

    std::vector<std::string> observations = {
        "commands", "ang_vel", "gravity_vec", "dof_pos", "dof_vel", "actions"
    };
    std::vector<int> observations_history = {0, 1, 2, 3, 4, 5};

    // ---------- Build default observation (single step = 45 dims) ----------
    // Standing still: zero commands, zero ang_vel, gravity pointing down,
    // dof_pos at default, zero dof_vel, zero actions

    std::vector<float> gravity_vec = {0.0f, 0.0f, -1.0f};
    std::vector<float> commands = {0.0f, 0.0f, 0.0f};
    std::vector<float> ang_vel = {0.0f, 0.0f, 0.0f};
    std::vector<float> dof_vel(num_dofs, 0.0f);
    std::vector<float> actions(num_dofs, 0.0f);

    std::vector<float> scaled_gravity = gravity_vec;
    for (auto& v : scaled_gravity) v *= gravity_vec_scale;

    std::vector<float> single_obs;
    for (const auto& obs_name : observations)
    {
        if (obs_name == "commands")
        {
            std::vector<float> scaled = commands;
            for (size_t i = 0; i < scaled.size() && i < commands_scale.size(); ++i)
                scaled[i] *= commands_scale[i];
            single_obs.insert(single_obs.end(), scaled.begin(), scaled.end());
        }
        else if (obs_name == "ang_vel")
        {
            std::vector<float> scaled = ang_vel;
            for (auto& v : scaled) v *= ang_vel_scale;
            single_obs.insert(single_obs.end(), scaled.begin(), scaled.end());
        }
        else if (obs_name == "gravity_vec")
        {
            single_obs.insert(single_obs.end(), scaled_gravity.begin(), scaled_gravity.end());
        }
        else if (obs_name == "dof_pos")
        {
            for (int i = 0; i < num_dofs; ++i)
                single_obs.push_back((default_dof_pos[i] - default_dof_pos[i]) * dof_pos_scale);
        }
        else if (obs_name == "dof_vel")
        {
            std::vector<float> scaled = dof_vel;
            for (auto& v : scaled) v *= dof_vel_scale;
            single_obs.insert(single_obs.end(), scaled.begin(), scaled.end());
        }
        else if (obs_name == "actions")
        {
            single_obs.insert(single_obs.end(), actions.begin(), actions.end());
        }
    }

    for (auto& v : single_obs)
        v = std::max(-clip_obs, std::min(clip_obs, v));

    std::cout << "Single obs dim: " << single_obs.size() << std::endl;
    std::cout << "Single obs:     " << format_vec(single_obs) << std::endl;

    // ---------- Compute obs dims for buffer ----------
    int history_length = 1;
    if (!observations_history.empty())
        history_length = *std::max_element(observations_history.begin(), observations_history.end()) + 1;

    std::vector<int> obs_dims;
    for (const auto& obs : observations)
    {
        int dim = 0;
        if (obs == "lin_vel") dim = 3;
        else if (obs == "ang_vel") dim = 3;
        else if (obs == "gravity_vec") dim = 3;
        else if (obs == "commands") dim = 3;
        else if (obs == "dof_pos") dim = num_dofs;
        else if (obs == "dof_vel") dim = num_dofs;
        else if (obs == "actions") dim = num_dofs;
        if (dim > 0) obs_dims.push_back(dim);
    }

    int obs_dim = 0;
    for (int d : obs_dims) obs_dim += d;
    std::cout << "Obs dims:       " << format_vec(std::vector<float>(obs_dims.begin(), obs_dims.end())) << std::endl;
    std::cout << "Obs total dim:  " << obs_dim << std::endl;
    std::cout << "History length: " << history_length << std::endl;

    // ---------- Build observation history ----------
    ObservationBuffer buf(1, obs_dims, history_length, "time");
    for (int i = 0; i < history_length; ++i)
        buf.insert(single_obs);

    auto history_obs = buf.get_obs_vec(observations_history);
    std::cout << "History obs dim: " << history_obs.size() << std::endl;

    // ---------- Run inference ----------
    std::cout << "\nRunning inference with all-zero standing obs..." << std::endl;
    auto output = model->forward({history_obs});

    std::cout << "Output size:    " << output.size() << std::endl;
    std::cout << "Output:         " << format_vec(output) << std::endl;

    // ---------- Validate ----------
    bool pass = true;

    if (output.empty())
    {
        std::cerr << "FAIL: output is empty" << std::endl;
        pass = false;
    }

    if ((int)output.size() != num_dofs)
    {
        std::cerr << "FAIL: expected " << num_dofs << " outputs, got " << output.size() << std::endl;
        pass = false;
    }

    for (int i = 0; i < (int)output.size(); ++i)
    {
        if (std::isnan(output[i]) || std::isinf(output[i]))
        {
            std::cerr << "FAIL: NaN/Inf at output[" << i << "] = " << output[i] << std::endl;
            pass = false;
            break;
        }
    }

    if (pass)
    {
        float max_abs = *std::max_element(output.begin(), output.end(),
            [](float a, float b) { return std::abs(a) < std::abs(b); });

        std::cout << "Max |action|:   " << std::fixed << std::setprecision(6) << max_abs << std::endl;

        if (max_abs < 1e-6f)
        {
            std::cerr << "WARN: all outputs near zero — model may not be producing meaningful actions" << std::endl;
        }

        // For standing still with zero input, actions should be small (near zero)
        // since the policy should produce near-default positions.
        // But exactly zero is suspicious for a trained policy.
        bool all_zero = true;
        for (float v : output)
        {
            if (std::abs(v) > 1e-8f) { all_zero = false; break; }
        }
        if (all_zero)
        {
            std::cerr << "WARN: all outputs exactly zero — model output suspicious" << std::endl;
        }
    }

    // ---------- Test with non-zero command ----------
    if (pass)
    {
        std::cout << "\nRunning inference with forward command [0.5, 0, 0]..." << std::endl;

        ObservationBuffer buf2(1, obs_dims, history_length, "time");
        for (int i = 0; i < history_length; ++i)
            buf2.insert(single_obs);

        // Rebuild with non-zero command
        std::vector<float> cmd_obs;
        std::vector<float> fwd_cmd = {0.5f, 0.0f, 0.0f};
        for (const auto& obs_name : observations)
        {
            if (obs_name == "commands")
            {
                std::vector<float> scaled = fwd_cmd;
                for (size_t i = 0; i < scaled.size() && i < commands_scale.size(); ++i)
                    scaled[i] *= commands_scale[i];
                cmd_obs.insert(cmd_obs.end(), scaled.begin(), scaled.end());
            }
            else if (obs_name == "ang_vel")
            {
                std::vector<float> scaled = ang_vel;
                for (auto& v : scaled) v *= ang_vel_scale;
                cmd_obs.insert(cmd_obs.end(), scaled.begin(), scaled.end());
            }
            else if (obs_name == "gravity_vec")
            {
                cmd_obs.insert(cmd_obs.end(), scaled_gravity.begin(), scaled_gravity.end());
            }
            else if (obs_name == "dof_pos")
            {
                for (int i = 0; i < num_dofs; ++i)
                    cmd_obs.push_back(0.0f);
            }
            else if (obs_name == "dof_vel")
            {
                std::vector<float> scaled = dof_vel;
                for (auto& v : scaled) v *= dof_vel_scale;
                cmd_obs.insert(cmd_obs.end(), scaled.begin(), scaled.end());
            }
            else if (obs_name == "actions")
            {
                cmd_obs.insert(cmd_obs.end(), actions.begin(), actions.end());
            }
        }

        for (auto& v : cmd_obs)
            v = std::max(-clip_obs, std::min(clip_obs, v));

        buf2.insert(cmd_obs);
        auto history_obs2 = buf2.get_obs_vec(observations_history);

        auto output2 = model->forward({history_obs2});
        std::cout << "Output:         " << format_vec(output2) << std::endl;

        bool any_diff = false;
        for (size_t i = 0; i < output.size() && i < output2.size(); ++i)
        {
            if (std::abs(output[i] - output2[i]) > 1e-8f) { any_diff = true; break; }
        }
        if (!any_diff)
        {
            std::cerr << "WARN: output unchanged with different command — model may be ignoring input" << std::endl;
        }
        else
        {
            std::cout << "OK: output changed with different command" << std::endl;
        }
    }

    // ---------- Summary ----------
    std::cout << "\n========================================" << std::endl;
    if (pass)
        std::cout << "PASS: ONNX smoke test passed" << std::endl;
    else
        std::cout << "FAIL: ONNX smoke test failed" << std::endl;
    std::cout << "========================================" << std::endl;

    return pass ? 0 : 1;
}
