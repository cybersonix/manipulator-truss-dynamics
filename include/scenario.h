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
    explicit ScenarioRunner(SatelliteModel model);
    void SetDataRecorder(std::shared_ptr<DataRecorder> recorder);
    void Run(const ScenarioConfig &config);

private:
    SatelliteModel model_;
    std::shared_ptr<DataRecorder> recorder_;
};

#endif