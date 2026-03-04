/*
 * @Author: Sun Hao
 * @Email: sunhaonuaa@outlook.com
 * @Date: 2026-03-02 08:48:34
 * @LastEditTime: 2026-03-04 14:43:26
 * @FilePath: /manipulator-truss-dynamics/scenarios/relative_motion_elimination.cpp
 * /manipulator-truss-dynamics/scenarios/relative_motion_elimination.cpp
 * @Description: 相对运动消除阶段仿真场景
 *              根据报告《机械臂辅助对接场景动力学仿真分析-消除相对运动阶段控制分析》
 *              场景描述：抓捕完成后，对接点相距1.1m，机械臂保持在起始位形，
 *              通过PD控制消除目标航天器相对于巡游飞行器的相对运动。
 *
 * Copyright (c) 2026 by Sun Hao, All Rights Reserved.
 */
#include <cmath>
#include <iostream>

#include "controllers.h"
#include "satellite_docking_model.h"
#include "scenario.h"

int main() {
  chrono::SetChronoDataPath(CHRONO_DATA_DIR);

  // 控制目标：保持在起始位形，即目标角度增量为0
  std::vector<double> target_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // PD控制器参数
  // std::vector<double> Kp = {1343.0, 339.5, 750.0, 313.25, 716.0, 282.5, 565.0};
  // std::vector<double> Kd = {3171.5, 2857.0, 3000.0, 7112.5,
  //                           6947.0, 6010.0, 6010.0};

  std::vector<double> Kp = {13.43, 13.58, 15.00, 12.53, 14.32, 11.30, 1.40};
  std::vector<double> Kd = {63.43, 57.14, 60.00, 142.25, 138.94, 120.20, 110.45};


  // 力矩限幅 ±5 Nm
  double torque_limit = 5.0;

  // 创建PD控制器
  std::vector<std::shared_ptr<JointController>> controllers;
  for (size_t i = 0; i < 7; ++i) {
    auto pd = std::make_shared<PDController>();
    pd->Initialize({target_angles[i], Kp[i], Kd[i], torque_limit});
    controllers.push_back(pd);
  }

  ModelConfig model_cfg;

  // 初始条件：施加在目标航天器（body_star_right + builder2 + body_spacecraft）
  model_cfg.target_spacecraft_linear_vel = ChVector3d(0.02, 0.02, 0.02);
  model_cfg.target_spacecraft_angular_vel = ChVector3d(
      -0.01 * M_PI / 180.0, 0.01 * M_PI / 180.0, 0.01 * M_PI / 180.0);

  // 创建模型
  SatelliteModel satellite;
  try {
    satellite = CreateSatelliteModel(model_cfg, controllers);
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] 模型创建失败: " << e.what() << std::endl;
    return 1;
  }

  ScenarioConfig scenario_cfg;
  scenario_cfg.max_time = 600.0;
  scenario_cfg.output_file = "results/relative_motion_elimination/relative_motion_elimination.csv";

  ScenarioRunner runner(std::move(satellite));
  runner.Run(scenario_cfg);

  return 0;
}