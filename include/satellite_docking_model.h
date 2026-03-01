#ifndef SATELLITE_DOCKING_MODEL_H
#define SATELLITE_DOCKING_MODEL_H

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/fea/ChMesh.h"
#include "chrono_parsers/ChParserURDF.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include <vector>
#include <string>
#include <memory>

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::parsers;

// 前向声明
class JointController;

// 模型配置参数
struct ModelConfig
{
    // 梁参数
    double beam_length = 83.24;
    int beam_elements = 10;
    double beam_height = 0.90603;

    // 材料参数
    double rho = 12140.0;
    double E = 5.9773e10;
    double nu = 0.3;

    // 几何尺寸
    double star_Lx = 1.6, star_Wy = 1.0, star_Hz = 0.9, star_mass = 450.0;
    double base_Lx = 1.0, base_Wy = 1.4, base_Hz = 1.4, base_mass = 400.0;
    double sc_Lx = 1.0, sc_Wy = 2.0, sc_Hz = 2.0, sc_mass = 3935.0;

    // 机械臂
    std::string urdf_file = "assets/urdf/robot.urdf";
    std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

    // 初始间距
    double initial_gap = 1.1;

    // 添加：初始条件
    ChVector3d base_initial_vel = ChVector3d(0, 0, 0);
    ChVector3d base_initial_ang_vel = ChVector3d(0, 0, 0);
};

struct SatelliteModel
{
    std::shared_ptr<ChSystemSMC> system;
    std::shared_ptr<ChMesh> mesh;
    std::shared_ptr<ChBody> body_star_left;
    std::shared_ptr<ChBody> body_base;
    std::shared_ptr<ChBody> body_star_right;
    std::shared_ptr<ChBody> body_spacecraft;
    std::vector<std::shared_ptr<ChLinkMotorRotation>> arm_motors;
    std::shared_ptr<ChBody> arm_link8; // 末端执行器
    ChVector3d star_half_size;
    ChVector3d base_half_size;
    std::vector<std::string> joint_names;
    std::vector<std::shared_ptr<JointController>> controllers; // 保存控制器，防止被销毁（因为ChFunction内部使用）
};

SatelliteModel CreateSatelliteModel(
    const ModelConfig &config = ModelConfig(),
    const std::vector<std::shared_ptr<JointController>> &controllers = {});

#endif