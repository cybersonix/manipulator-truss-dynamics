#include "satellite_docking_model.h"
#include "scenario.h"
#include "controllers.h"
#include <iostream>

int main()
{
    chrono::SetChronoDataPath(CHRONO_DATA_DIR);

    // 定义控制参数
    std::vector<double> init_angles = {1.5862, 0.0910, 2.1837, 0.1037, -0.9215, 1.4654, 1.8753};
    std::vector<double> end_angles = {0.8413, 1.1435, 2.1885, 2.1069, -0.2202, 0.7031, 0.4938};
    std::vector<double> Kp = {13.43, 13.58, 15.00, 12.53, 14.32, 11.30, 1.40};
    std::vector<double> Kd = {63.43, 57.14, 60.00, 142.25, 138.94, 120.20, 110.45};

    // 创建PD控制器
    std::vector<std::shared_ptr<JointController>> controllers;
    for (size_t i = 0; i < 7; ++i)
    {
        auto pd = std::make_shared<PDController>();
        double target = -end_angles[i] + init_angles[i];
        pd->Initialize({target, Kp[i], Kd[i], 100.0});
        controllers.push_back(pd);
    }

    // 配置模型参数
    ModelConfig model_cfg;
    // 初始条件（可选，也可以在ScenarioConfig中覆盖）
    // model_cfg.base_initial_vel = ChVector3d(0, 0.5, 0.2);

    // 创建模型（传入控制器，在内部设置）
    SatelliteModel satellite;
    try
    {
        satellite = CreateSatelliteModel(model_cfg, controllers);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] 模型创建失败: " << e.what() << std::endl;
        return 1;
    }

    // 配置场景
    ScenarioConfig scenario_cfg;
    scenario_cfg.max_time = 300.0;
    scenario_cfg.output_file = "results/pd_docking.csv";
    // 如果ModelConfig中已设置初始速度，这里不需要重复设置
    // 但可以用SetInitialConditions覆盖

    // 创建运行器并运行
    ScenarioRunner runner(std::move(satellite));
    // 可选：覆盖初始条件
    // runner.SetInitialConditions(ChVector3d(0, 0.5, 0.2), ChVector3d(0, 0, 0));

    runner.Run(scenario_cfg);

    return 0;
}