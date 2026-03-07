/*
 * @Author: Sun Hao
 * @Email: sunhaonuaa@outlook.com
 * @Date: 2026-03-01 21:13:36
 * @LastEditTime: 2026-03-07 15:49:07
 * @FilePath: /manipulator-truss-dynamics/src/scenario_runner.cpp
 * @Description: 场景运行器实现文件，负责管理卫星对接仿真的执行流程，
 *               包括可视化系统初始化、仿真循环控制、数据记录和结果保存。
 *               提供DefaultDataRecorder类用于记录关节角度、速度、力矩以及对接距离等关键数据。
 *
 * Copyright (c) 2026 by Sun Hao, All Rights Reserved.
 */

#include <fstream>
#include <iomanip>
#include <iostream>

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "controllers.h"
#include "scenario.h"

using namespace chrono;
using namespace chrono::vsg3d;

class DefaultDataRecorder : public DataRecorder {
 public:
  struct DataPoint {
    double time;
    double docking_distance;
    std::vector<double> joint_angles;
    std::vector<double> joint_velocities;
    std::vector<double> joint_control_torques;
    ChVector3d relative_velocity;  // body_star_right相对于body_base的相对速度
    double relative_speed;         // 相对速度大小
    ChVector3d beam2_mid_position; // beam2中间节点的位置
  };

  std::vector<DataPoint> data_history;
  std::vector<std::string> joint_names;

  explicit DefaultDataRecorder(const std::vector<std::string>& names)
      : joint_names(names) {}

  void Record(double time, const SatelliteModel& model) override {
    DataPoint point;
    point.time = time;

    // 计算对接距离
    point.docking_distance =
        model.body_star_right->GetPos().x() - model.body_base->GetPos().x() -
        model.star_half_size.x() - model.base_half_size.x();

    // 计算相对运动速度（body_star_right相对于body_base）
    ChVector3d vel_right = model.body_star_right->GetPosDt();
    ChVector3d vel_base = model.body_base->GetPosDt();
    point.relative_velocity = vel_right - vel_base;
    point.relative_speed = point.relative_velocity.Length();

    // 获取beam2中间节点相对于初始位置的位移
    if (!model.builder2_nodes.empty()) {
      // 计算中间节点索引
      size_t mid_index = model.builder2_nodes.size() / 2;
      
      // 获取中间节点的当前位置
      ChVector3d current_mid_position = model.builder2_nodes[mid_index]->GetPos();
      
      // 获取中间节点的初始位置
      static ChVector3d initial_mid_position;
      static bool initialized = false;
      if (!initialized) {
        initial_mid_position = current_mid_position;
        initialized = true;
      }
      
      // 计算中间节点相对于初始位置的位移
      point.beam2_mid_position = current_mid_position - initial_mid_position;
    } else {
      point.beam2_mid_position = ChVector3d(0, 0, 0);
    }

    // 记录每个关节的数据
    for (size_t i = 0; i < model.arm_motors.size(); ++i) {
      if (model.arm_motors[i]) {
        point.joint_angles.push_back(model.arm_motors[i]->GetMotorAngle());
        point.joint_velocities.push_back(
            model.arm_motors[i]->GetMotorAngleDt());

        // 计算控制力矩（通过控制器实时计算）
        double control_torque = 0.0;
        if (i < model.controllers.size() && model.controllers[i]) {
          control_torque =
              model.controllers[i]->ComputeTorque(model.arm_motors[i]);
        }
        point.joint_control_torques.push_back(control_torque);
      } else {
        point.joint_angles.push_back(0.0);
        point.joint_velocities.push_back(0.0);
        point.joint_control_torques.push_back(0.0);
      }
    }

    data_history.push_back(point);
  }

  void Save(const std::string& filename) override {
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << "[ERROR] 无法打开文件: " << filename << std::endl;
      return;
    }

    // 设置精度
    file << std::scientific << std::setprecision(6);

    // 表头
    file << "time,docking_distance,relative_vx,relative_vy,relative_vz,"
            "relative_speed,beam2_mid_x,beam2_mid_y,beam2_mid_z";
    for (size_t i = 0; i < joint_names.size(); ++i) {
      file << "," << joint_names[i] << "_angle";
      file << "," << joint_names[i] << "_velocity";
      file << "," << joint_names[i] << "_control_torque";
    }
    file << "\n";

    // 数据
    for (const auto& d : data_history) {
      file << d.time << "," << d.docking_distance << ","
           << d.relative_velocity.x() << "," << d.relative_velocity.y() << ","
           << d.relative_velocity.z() << "," << d.relative_speed << ","
           << d.beam2_mid_position.x() << "," << d.beam2_mid_position.y() << ","
           << d.beam2_mid_position.z();
      for (size_t i = 0; i < d.joint_angles.size(); ++i) {
        file << "," << d.joint_angles[i] << "," << d.joint_velocities[i] << ","
             << d.joint_control_torques[i];
      }
      file << "\n";
    }

    file.close();
    std::cout << "仿真数据已保存到 " << filename << std::endl;
  }
};

ScenarioRunner::ScenarioRunner(SatelliteModel model)
    : model_(std::move(model)), recorder_(nullptr) {}

void ScenarioRunner::SetDataRecorder(std::shared_ptr<DataRecorder> recorder) {
  recorder_ = recorder;
}

void ScenarioRunner::Run(const ScenarioConfig& config) {
  // 创建可视化系统（VSG）
  auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
  vis->AttachSystem(model_.system.get());
  vis->SetWindowSize(ChVector2i(1920, 1080));
  vis->SetWindowTitle("Micro-Satellite Docking Scene");
  vis->SetCameraVertical(CameraVerticalDir::Y);
  vis->AddCamera(ChVector3d(84.0, -5.0, 3.0), ChVector3d(84.0, 0, 0));
  vis->Initialize();

  // 如果没有外部记录器，创建默认的
  if (!recorder_) {
    recorder_ = std::make_shared<DefaultDataRecorder>(model_.joint_names);
  }

  // 仿真循环
  int frame_count = 0;

  while (vis->Run() && model_.system->GetChTime() < config.max_time) {
    double time = model_.system->GetChTime();
    // 记录数据
    if (frame_count % config.record_interval == 0) {
      recorder_->Record(time, model_);

      // 实时输出
      auto* default_recorder =
          dynamic_cast<DefaultDataRecorder*>(recorder_.get());
      if (default_recorder && !default_recorder->data_history.empty()) {
        const auto& last = default_recorder->data_history.back();
        std::cout << "Model Time: " << time
                  << "s, Docking Distance: " << last.docking_distance
                  << "m, Relative Speed: " << last.relative_speed << " m/s"
                  << std::endl;
      }
    }

    vis->Render();
    model_.system->DoStepDynamics(config.timestep);
    frame_count++;
  }

  recorder_->Save(config.output_file);
}