#ifndef SCENARIO_H
#define SCENARIO_H

#include "satellite_docking_model.h"
#include <string>
#include <memory>

// 数据记录器接口
class DataRecorder
{
public:
    virtual ~DataRecorder() = default;
    virtual void Record(double time, const SatelliteModel &model) = 0;
    virtual void Save(const std::string &filename) = 0;
};

// 场景配置
struct ScenarioConfig
{
    // 初始条件（也可以在ModelConfig中设置）
    ChVector3d base_initial_vel = ChVector3d(0, 0, 0);
    ChVector3d base_initial_ang_vel = ChVector3d(0, 0, 0);

    // 仿真参数
    double timestep = 0.005;
    double max_time = 300.0;
    int record_interval = 1; // 每多少帧记录一次

    // 输出配置
    std::string output_file = "simulation_data.csv";
};

// 场景运行器
class ScenarioRunner
{
public:
    // 构造函数：模型已经包含控制器（由CreateSatelliteModel设置）
    explicit ScenarioRunner(SatelliteModel model);

    // 可选：覆盖初始条件（如果与ModelConfig中不同）
    void SetInitialConditions(const ChVector3d &vel, const ChVector3d &ang_vel);
    void SetDataRecorder(std::shared_ptr<DataRecorder> recorder);

    void Run(const ScenarioConfig &config);

private:
    SatelliteModel model_;
    std::shared_ptr<DataRecorder> recorder_;
};

#endif