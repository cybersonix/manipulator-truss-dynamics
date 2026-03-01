#include "scenario.h"
#include "chrono_vsg/ChVisualSystemVSG.h"
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace chrono;
using namespace chrono::vsg3d;

class DefaultDataRecorder : public DataRecorder
{
public:
    struct DataPoint
    {
        double time;
        double docking_distance;
        std::vector<double> joint_angles;
        std::vector<double> joint_velocities;
        std::vector<ChVector3d> joint_torques;
    };

    std::vector<DataPoint> data_history;
    std::vector<std::string> joint_names;

    explicit DefaultDataRecorder(const std::vector<std::string> &names)
        : joint_names(names) {}

    void Record(double time, const SatelliteModel &model) override
    {
        DataPoint point;
        point.time = time;

        // 计算对接距离
        point.docking_distance = model.body_star_right->GetPos().x() - model.body_base->GetPos().x() - model.star_half_size.x() - model.base_half_size.x();

        // 记录每个关节的数据
        for (size_t i = 0; i < model.arm_motors.size(); ++i)
        {
            if (model.arm_motors[i])
            {
                point.joint_angles.push_back(model.arm_motors[i]->GetMotorAngle());
                point.joint_velocities.push_back(model.arm_motors[i]->GetMotorAngleDt());

                ChWrenchd wrench_local = model.arm_motors[i]->GetReaction1();
                ChFrame<> frame1_abs = model.arm_motors[i]->GetFrame1Abs();
                ChVector3d torque_global = frame1_abs.GetRot().Rotate(wrench_local.torque);
                point.joint_torques.push_back(torque_global);
            }
            else
            {
                point.joint_angles.push_back(0.0);
                point.joint_velocities.push_back(0.0);
                point.joint_torques.push_back(ChVector3d(0, 0, 0));
            }
        }

        data_history.push_back(point);
    }

    void Save(const std::string &filename) override
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "[ERROR] 无法打开文件: " << filename << std::endl;
            return;
        }

        // 设置精度
        file << std::scientific << std::setprecision(6);

        // 表头
        file << "time,docking_distance";
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            file << "," << joint_names[i] << "_angle";
            file << "," << joint_names[i] << "_velocity";
            file << "," << joint_names[i] << "_tx";
            file << "," << joint_names[i] << "_ty";
            file << "," << joint_names[i] << "_tz";
        }
        file << "\n";

        // 数据
        for (const auto &d : data_history)
        {
            file << d.time << "," << d.docking_distance;
            for (size_t i = 0; i < d.joint_angles.size(); ++i)
            {
                file << "," << d.joint_angles[i]
                     << "," << d.joint_velocities[i]
                     << "," << d.joint_torques[i].x()
                     << "," << d.joint_torques[i].y()
                     << "," << d.joint_torques[i].z();
            }
            file << "\n";
        }

        file.close();
        std::cout << "数据已保存到 " << filename << std::endl;
    }
};

ScenarioRunner::ScenarioRunner(SatelliteModel model)
    : model_(std::move(model)), recorder_(nullptr) {}

void ScenarioRunner::SetInitialConditions(const ChVector3d &vel, const ChVector3d &ang_vel)
{
    // 覆盖初始条件
    if (model_.body_base)
    {
        model_.body_base->SetPosDt(vel);
        model_.body_base->SetAngVelParent(ang_vel);
    }
}

void ScenarioRunner::SetDataRecorder(std::shared_ptr<DataRecorder> recorder)
{
    recorder_ = recorder;
}

void ScenarioRunner::Run(const ScenarioConfig &config)
{
    // 创建可视化系统（VSG）
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(model_.system.get());
    vis->SetWindowSize(ChVector2i(1920, 1080));
    vis->SetWindowTitle("Micro-Satellite Docking Scene");
    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->AddCamera(ChVector3d(84.0, -5.0, 3.0), ChVector3d(84.0, 0, 0));
    vis->Initialize();

    // 如果没有外部记录器，创建默认的
    if (!recorder_)
    {
        recorder_ = std::make_shared<DefaultDataRecorder>(model_.joint_names);
    }

    // 仿真循环
    int frame_count = 0;

    while (vis->Run() && model_.system->GetChTime() < config.max_time)
    {
        double time = model_.system->GetChTime();
        // 记录数据
        if (frame_count % config.record_interval == 0)
        {
            recorder_->Record(time, model_);

            // 实时输出
            auto *default_recorder = dynamic_cast<DefaultDataRecorder *>(recorder_.get());
            if (default_recorder && !default_recorder->data_history.empty())
            {
                const auto &last = default_recorder->data_history.back();
                std::cout << "Model Time: " << time
                          << "s, Docking Distance: " << last.docking_distance << "m" << std::endl;
            }
        }

        vis->Render();
        model_.system->DoStepDynamics(config.timestep);
        frame_count++;
    }

    recorder_->Save(config.output_file);
}