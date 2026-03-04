/*
 * @Author: Sun Hao
 * @Email: sunhaonuaa@outlook.com
 * @Date: 2026-03-01 19:25:14
 * @LastEditTime: 2026-03-04 17:54:28
 * @FilePath: /manipulator-truss-dynamics/src/satellite_docking_model.cpp
 * @Description: 卫星对接场景动力学模型实现
 *              创建包含柔性桁架结构、对接节点星、机械臂基座、航天器平台和7DoF机械臂的动力学系统
 *              支持关节控制器集成，通过ChFunction适配器实现力矩控制
 * 
 * Copyright (c) 2026 by Sun Hao, All Rights Reserved. 
 */
#include "satellite_docking_model.h"
#include "controllers.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/assets/ChVisualShapeFEA.h"

SatelliteModel CreateSatelliteModel(
    const ModelConfig &cfg,
    const std::vector<std::shared_ptr<JointController>> &controllers)
{
    SatelliteModel model;
    model.joint_names = cfg.joint_names;

    // 创建系统
    model.system = chrono_types::make_shared<ChSystemSMC>();
    model.system->SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // 创建FEA网格
    model.mesh = chrono_types::make_shared<ChMesh>();
    model.system->Add(model.mesh);

    // 保存尺寸用于后续计算
    model.star_half_size = ChVector3d(cfg.star_Lx / 2, cfg.star_Wy / 2, cfg.star_Hz / 2);
    model.base_half_size = ChVector3d(cfg.base_Lx / 2, cfg.base_Wy / 2, cfg.base_Hz / 2);

    // ========== 梁截面定义 ==========
    auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();
    msection->SetDensity(cfg.rho);
    msection->SetYoungModulus(cfg.E);
    msection->SetShearModulusFromPoisson(cfg.nu);
    msection->SetRayleighDamping(0.01, 0.005);

    double Area = 5.4428e-4, Iyy = 9.0814e-5, Izz = 9.0814e-5, J = 1.8163e-4;
    msection->SetArea(Area);
    msection->SetIyy(Iyy);
    msection->SetIzz(Izz);
    msection->SetJ(J);

    // ========== 构建第一根梁 ==========
    ChBuilderBeamEuler builder1;
    ChVector3d beam1_end_B(cfg.beam_length, 0, 0);
    builder1.BuildBeam(model.mesh, msection, cfg.beam_elements,
                       ChVector3d(0, 0, 0), beam1_end_B, ChVector3d(0, 1, 0));
    double beam1_bottom_z = beam1_end_B.z() - cfg.beam_height / 2.0;

    // 保存builder1的节点
    auto nodes1 = builder1.GetLastBeamNodes();

    // ========== 左侧对接星 ==========
    double star_density = cfg.star_mass / (cfg.star_Lx * cfg.star_Wy * cfg.star_Hz);
    model.body_star_left = chrono_types::make_shared<ChBodyEasyBox>(
        cfg.star_Lx, cfg.star_Wy, cfg.star_Hz, star_density, true, false);
    model.body_star_left->SetPos(ChVector3d(-cfg.star_Lx / 2, 0, beam1_bottom_z + cfg.star_Hz / 2));
    model.body_star_left->SetFixed(false);
    model.body_star_left->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.4f, 0.1f)); // 橙色
    model.system->Add(model.body_star_left);

    auto link_star_left = chrono_types::make_shared<ChLinkMateFix>();
    link_star_left->Initialize(nodes1.front(), model.body_star_left);
    model.system->Add(link_star_left);

    // ========== 机械臂基座 ==========
    double base_density = cfg.base_mass / (cfg.base_Lx * cfg.base_Wy * cfg.base_Hz);
    model.body_base = chrono_types::make_shared<ChBodyEasyBox>(
        cfg.base_Lx, cfg.base_Wy, cfg.base_Hz, base_density, true, false);
    model.body_base->SetPos(ChVector3d(beam1_end_B.x() - cfg.base_Lx / 2.0, 0, beam1_bottom_z - cfg.base_Hz / 2.0));
    model.body_base->SetFixed(false);
    model.body_base->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.6f, 0.3f)); // 绿色
    model.system->Add(model.body_base);

    auto link_base = chrono_types::make_shared<ChLinkMateFix>();
    link_base->Initialize(nodes1.back(), model.body_base);
    model.system->Add(link_base);

    // ========== 右侧对接星 ==========
    double body_star_right_center_x = beam1_end_B.x() + cfg.initial_gap + cfg.star_Lx / 2.0;
    double star_right_center_z = beam1_bottom_z + cfg.star_Hz / 2.0;

    model.body_star_right = chrono_types::make_shared<ChBodyEasyBox>(
        cfg.star_Lx, cfg.star_Wy, cfg.star_Hz, star_density, true, false);
    model.body_star_right->SetPos(ChVector3d(body_star_right_center_x, 0, star_right_center_z));
    model.body_star_right->SetFixed(false);
    model.body_star_right->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.4f, 0.1f)); // 橙色
    model.system->Add(model.body_star_right);

    // ========== 构建第二根梁 ==========
    ChBuilderBeamEuler builder2;
    double beam2_start_x = body_star_right_center_x + cfg.star_Lx / 2.0;
    ChVector3d beam2_end_B(beam2_start_x + cfg.beam_length, 0, 0);
    builder2.BuildBeam(model.mesh, msection, cfg.beam_elements,
                       ChVector3d(beam2_start_x, 0, 0), beam2_end_B, ChVector3d(0, 1, 0));

    // 保存builder2的节点
    auto nodes2 = builder2.GetLastBeamNodes();
    model.builder2_nodes = nodes2;  // 用于后续设置初始条件

    auto link_beam2_star = chrono_types::make_shared<ChLinkMateFix>();
    link_beam2_star->Initialize(nodes2.front(), model.body_star_right);
    model.system->Add(link_beam2_star);

    // ========== 航天器平台 ==========
    double sc_density = cfg.sc_mass / (cfg.sc_Lx * cfg.sc_Wy * cfg.sc_Hz);
    model.body_spacecraft = chrono_types::make_shared<ChBodyEasyBox>(
        cfg.sc_Lx, cfg.sc_Wy, cfg.sc_Hz, sc_density, true, false);
    double beam2_bottom_z = beam1_bottom_z;
    model.body_spacecraft->SetPos(ChVector3d(beam2_end_B.x() + cfg.sc_Lx / 2.0, 0, beam2_bottom_z));
    model.body_spacecraft->SetFixed(false);
    model.body_spacecraft->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.2f, 0.8f)); // 蓝色
    model.system->Add(model.body_spacecraft);

    auto link_sc = chrono_types::make_shared<ChLinkMateFix>();
    link_sc->Initialize(nodes2.back(), model.body_spacecraft);
    model.system->Add(link_sc);

    // ========== 创建机械臂 (URDF) ==========
    try
    {
        ChParserURDF urdf_parser(cfg.urdf_file);
        urdf_parser.SetAllJointsActuationType(ChParserURDF::ActuationType::FORCE);

        // 计算安装位置：基座右上角附近
        ChVector3d arm_mount_pos = model.body_base->GetPos() + ChVector3d(
                                                                   cfg.base_Lx / 2.0 - 0.15,
                                                                   -cfg.base_Wy / 2.0,
                                                                   cfg.base_Hz / 2.0 - 0.15);

        ChQuaternion<> rot_x_90 = QuatFromAngleX(CH_PI / 2.0);
        ChFrame<> root_init_pose(arm_mount_pos, rot_x_90);
        urdf_parser.SetRootInitPose(root_init_pose);
        urdf_parser.PopulateSystem(*model.system);

        // 获取根连杆 link1 并固连到基座
        auto link1 = urdf_parser.GetRootChBody();
        auto link1_to_base = chrono_types::make_shared<ChLinkMateFix>();
        link1_to_base->Initialize(link1, model.body_base, ChFrame<>(arm_mount_pos, QUNIT));
        model.system->Add(link1_to_base);

        // 获取末端执行器 link8 并固连到右侧对接星
        model.arm_link8 = urdf_parser.GetChBody("link8");
        auto link8_to_star = chrono_types::make_shared<ChLinkMateFix>();
        ChFrame<> link8_frame(model.arm_link8->GetPos() + ChVector3d(0, 0, 0.05), QUNIT);
        link8_to_star->Initialize(model.arm_link8, model.body_star_right, link8_frame);
        model.system->Add(link8_to_star);

        // 获取电机并设置控制器
        for (size_t i = 0; i < cfg.joint_names.size(); ++i)
        {
            auto motor = urdf_parser.GetChMotor(cfg.joint_names[i]);
            auto rot_motor = std::dynamic_pointer_cast<ChLinkMotorRotation>(motor);
            if (rot_motor)
            {
                model.arm_motors.push_back(rot_motor);

                if (i < controllers.size() && controllers[i])
                {
                    // 保存控制器防止被销毁
                    model.controllers.push_back(controllers[i]);

                    // 创建ChFunction适配器
                    auto ch_function = chrono_types::make_shared<ControllerChFunction>(
                        controllers[i], rot_motor);

                    // 设置到URDF解析器
                    urdf_parser.SetMotorFunction(cfg.joint_names[i], ch_function);
                }
            }
            else
            {
                // 保持占位，确保索引对齐
                model.arm_motors.push_back(nullptr);
                std::cerr << "[WARNING] 关节 " << cfg.joint_names[i] << " 不是旋转电机" << std::endl;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] URDF加载失败: " << e.what() << std::endl;
        throw;
    }

    // 设置初始条件
    if (cfg.target_spacecraft_linear_vel != ChVector3d(0, 0, 0) || 
        cfg.target_spacecraft_angular_vel != ChVector3d(0, 0, 0)) {
        
        // 计算质心位置：builder2轴心，从nodes2.front()向右71.337m
        ChVector3d builder2_start = ChVector3d(beam2_start_x, 0, 0);
        ChVector3d center_of_mass = builder2_start + ChVector3d(71.337, 0, 0);
        
        ChVector3d omega = cfg.target_spacecraft_angular_vel;
        ChVector3d v_target = cfg.target_spacecraft_linear_vel;
        
        // 辅助函数：计算转动线速度 omega × r
        auto compute_rotational_velocity = [&](const ChVector3d& r) -> ChVector3d {
            return ChVector3d(
                omega.y() * r.z() - omega.z() * r.y(),
                omega.z() * r.x() - omega.x() * r.z(),
                omega.x() * r.y() - omega.y() * r.x()
            );
        };
        
        // body_star_right
        {
            ChVector3d r = model.body_star_right->GetPos() - center_of_mass;
            ChVector3d v_rot = compute_rotational_velocity(r);
            model.body_star_right->SetPosDt(v_target + v_rot);
            model.body_star_right->SetAngVelParent(omega);
        }
        
        // builder2柔性梁的节点
        for (auto& node : model.builder2_nodes) {
            ChVector3d node_pos = node->GetPos();
            ChVector3d r = node_pos - center_of_mass;
            ChVector3d v_rot = compute_rotational_velocity(r);
            node->SetPosDt(v_target + v_rot);
            node->SetAngVelParent(omega);
        }
        
        // body_spacecraft
        {
            ChVector3d r = model.body_spacecraft->GetPos() - center_of_mass;
            ChVector3d v_rot = compute_rotational_velocity(r);
            model.body_spacecraft->SetPosDt(v_target + v_rot);
            model.body_spacecraft->SetAngVelParent(omega);
        }
    }
    
    model.system->SetTimestepperType(ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED);
    
    // 求解器设置
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    model.system->SetSolver(solver);
    // solver->SetMaxIterations(1000);
    solver->SetTolerance(1e-8);
    solver->EnableWarmStart(true);
    solver->EnableDiagonalPreconditioner(true);
    
    // FEA可视化配置
    auto mvisualizebeam = chrono_types::make_shared<ChVisualShapeFEA>(model.mesh);
    mvisualizebeam->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizebeam->SetColorscaleMinMax(0.0, 1.0);
    mvisualizebeam->SetSmoothFaces(true);
    mvisualizebeam->SetWireframe(false);
    model.mesh->AddVisualShapeFEA(mvisualizebeam);

    return model;
}