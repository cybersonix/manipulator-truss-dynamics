#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_parsers/ChParserURDF.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::parsers;
using namespace chrono::vsg3d;

class TorqueController : public ChFunction
{
public:
    double target_angle = 0;
    double Kp = 0.0;        // 从表5获取
    double Kd = 0.0;        // 从表5获取
    double max_torque = 100.0;  // ±100 Nm饱和限制
    std::shared_ptr<ChLinkMotorRotation> motor;

    virtual double GetVal(double x) const override
    {
        if (!motor)
            return 0;

        double current_angle = motor->GetMotorAngle();
        double current_vel = motor->GetMotorAngleDt();
        double error = target_angle - current_angle;

        // PD控制：τ = Kp*(qd - qact) + Kd*(0 - qact_dot) = Kp*error - Kd*current_vel
        double torque = Kp * error - Kd * current_vel;

        // 饱和限制 ±100 Nm
        if (torque > max_torque)
            torque = max_torque;
        if (torque < -max_torque)
            torque = -max_torque;

        return torque;
    }

    virtual double GetDer(double x) const override { return 0; }

    virtual ChFunction* Clone() const override
    {
        auto new_func = new TorqueController();
        new_func->target_angle = target_angle;
        new_func->Kp = Kp;
        new_func->Kd = Kd;
        new_func->max_torque = max_torque;
        new_func->motor = motor;
        return new_func;
    }
};


int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[])
{
    chrono::SetChronoDataPath(CHRONO_DATA_DIR);

    // 1. 创建物理系统
    ChSystemSMC sys;

    // 设置无重力环境
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // 2. 创建 FEA 网格
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(my_mesh);

    // 3. 定义梁截面 (Euler-Bernoulli)
    auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    // --- 材料参数 ---
    double rho = 12140.0; // 等效密度 kg/m^3
    double E = 5.9773e10; // 弹性模量 Pa
    double nu = 0.3;      // 泊松比

    msection->SetDensity(rho);
    msection->SetYoungModulus(E);
    msection->SetShearModulusFromPoisson(nu);  // 根据 nu 自动计算 G
    msection->SetRayleighDamping(0.01, 0.005); // 轻微阻尼以保证数值稳定

    // --- 截面几何特性 (核心参数) ---
    double Area = 5.4428e-4; // 截面积 m^2
    double Iyy = 9.0814e-5;  // Y轴惯性矩 m^4
    double Izz = 9.0814e-5;  // Z轴惯性矩 m^4
    double J = 1.8163e-4;    // 扭转常数 m^4

    // 设置截面特性
    msection->SetArea(Area);
    msection->SetIyy(Iyy);
    msection->SetIzz(Izz);
    msection->SetJ(J);

    // 4. 构建梁 (无约束，自由漂浮)
    double beam_L = 83.24; // 梁总长
    int n_elements = 10;   // 单元数量
    // double beam_width_y = 1.0400;      // TKy, Y方向包络
    double beam_height_z = 0.90603; // TKz, Z方向包络

    ChBuilderBeamEuler builder1;
    ChVector3d beam1_end_B(beam_L, 0, 0);
    builder1.BuildBeam(my_mesh,              // 网格对象
                       msection,             // 截面属性
                       n_elements,           // 单元数
                       ChVector3d(0, 0, 0),  // 起点 A
                       beam1_end_B,          // 终点 B (沿X轴放置)
                       ChVector3d(0, 1, 0)); // Y轴参考方向 (定义截面朝向)

    double beam1_bottom_z = beam1_end_B.z() - beam_height_z / 2.0; // 梁下表面 Z 坐标

    // 添加对接节点星
    double star_Lx = 1.6;
    double star_Wy = 1.0;
    double star_Hz = 0.9;
    double star_mass = 450.0;
    double star_density = star_mass / (star_Lx * star_Wy * star_Hz);
    auto body_star_left = chrono_types::make_shared<ChBodyEasyBox>(
        star_Lx, star_Wy, star_Hz, // 尺寸
        star_density,              // 密度
        true,                      // 开启碰撞可视化
        false                      // 关闭碰撞检测
    );

    // 位置计算：
    // X: 对接在左端(0,0,0)的左侧 -> -Lx/2
    // Z: 下表面与梁下表面对齐 -> beam_bottom + Hz/2
    body_star_left->SetPos(ChVector3d(-star_Lx / 2.0, 0, beam1_bottom_z + star_Hz / 2.0));
    body_star_left->SetFixed(false);
    body_star_left->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.4f, 0.1f)); // 设为橙色
    sys.Add(body_star_left);

    // 约束：将左端刚体固定到梁的第一个节点
    auto link_beam1_star = chrono_types::make_shared<ChLinkMateFix>();
    link_beam1_star->Initialize(builder1.GetLastBeamNodes().front(), body_star_left);
    sys.Add(link_beam1_star);

    // 添加机械臂基座
    double base_Lx = 1.0;
    double base_Wy = 1.4;
    double base_Hz = 1.4;
    double base_mass = 400.0;
    double base_density = base_mass / (base_Lx * base_Wy * base_Hz);
    auto body_base = chrono_types::make_shared<ChBodyEasyBox>(
        base_Lx, base_Wy, base_Hz,
        base_density,
        true,
        false);

    // 位置计算：
    // X: 右端面对齐梁右端(L) -> L - Lx/2
    // Z: 上表面对齐梁下表面 -> beam_bottom - Hz/2
    body_base->SetPos(ChVector3d(beam1_end_B.x() - base_Lx / 2.0, 0, beam1_bottom_z - base_Hz / 2.0));
    body_base->SetFixed(false);
    body_base->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.6f, 0.3f)); // 设为绿色
    sys.Add(body_base);

    // 约束：将右端刚体固定到梁的最后一个节点
    auto link_base = chrono_types::make_shared<ChLinkMateFix>();
    link_base->Initialize(builder1.GetLastBeamNodes().back(), body_base);
    sys.Add(link_base);

    auto body_star_right = chrono_types::make_shared<ChBodyEasyBox>(
        star_Lx, star_Wy, star_Hz, // 尺寸
        star_density,              // 密度
        true,                      // 开启碰撞可视化
        false                      // 关闭碰撞检测
    );
    double body_star_right_center_x = beam1_end_B.x() + 1.1 + star_Lx / 2.0;
    double star_right_center_z = beam1_bottom_z + star_Hz / 2.0;
    body_star_right->SetPos(ChVector3d(body_star_right_center_x, 0, star_right_center_z));
    body_star_right->SetFixed(false);
    body_star_right->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.4f, 0.1f)); // 橙色
    sys.Add(body_star_right);

    ChBuilderBeamEuler builder2;
    double beam2_start_x = body_star_right_center_x + star_Lx / 2.0;
    ChVector3d beam2_end_B(beam2_start_x + beam_L, 0, 0);
    builder2.BuildBeam(my_mesh,                         // 网格对象
                       msection,                        // 截面属性
                       n_elements,                      // 单元数
                       ChVector3d(beam2_start_x, 0, 0), // 起点 A
                       beam2_end_B,                     // 终点 B (沿X轴放置)
                       ChVector3d(0, 1, 0));            // Y轴参考方向 (定义截面朝向)

    auto link_beam2_star = chrono_types::make_shared<ChLinkMateFix>();
    link_beam2_star->Initialize(builder2.GetLastBeamNodes().front(), body_star_right);
    sys.Add(link_beam2_star);

    // -----------------------------------------------------------
    // C. 航天器平台 (Spacecraft)
    // -----------------------------------------------------------
    // 位置: 左端面中心 与 新梁底边中点 对齐

    double sc_Lx = 1.0;
    double sc_Wy = 2.0;
    double sc_Hz = 2.0;
    double sc_mass = 3935.0;
    double sc_density = sc_mass / (sc_Lx * sc_Wy * sc_Hz);
    auto body_spacecraft = chrono_types::make_shared<ChBodyEasyBox>(
        sc_Lx, sc_Wy, sc_Hz,
        sc_density,
        true, false);

    double beam2_bottom_z = beam1_bottom_z;
    body_spacecraft->SetPos(ChVector3d(beam2_end_B.x() + sc_Lx / 2.0, 0, beam2_bottom_z));
    body_spacecraft->SetFixed(false);
    body_spacecraft->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.2f, 0.8f)); // 蓝色
    sys.Add(body_spacecraft);

    auto link_sc = chrono_types::make_shared<ChLinkMateFix>();
    link_sc->Initialize(builder2.GetLastBeamNodes().back(), body_spacecraft);
    sys.Add(link_sc);

    // 3. 创建机械臂
    // std::vector<std::shared_ptr<ChLinkMotorRotationTorque>> arm_motors;
    // std::vector<std::shared_ptr<TorqueController>> controllers;
    // URDF文件路径
    std::string urdf_file = "robot.urdf";

    try
    {
        // 1. 创建解析器 (构造函数传入文件名)
        ChParserURDF urdf_parser(urdf_file);

        std::cout << "[INFO] 加载URDF模型: " << urdf_parser.GetModelName() << std::endl;

        // 2. 配置所有关节为力矩控制模式
        urdf_parser.SetAllJointsActuationType(ChParserURDF::ActuationType::FORCE);

        // 3. 设置根链接(link1)的初始位姿
        // 计算安装位置：基座右上角附近
        ChVector3d arm_mount_pos = body_base->GetPos() + ChVector3d(
                                                             base_Lx / 2.0 - 0.15, // X: 右侧面偏左15cm
                                                             -base_Wy / 2.0,       // Y: 下表面
                                                             base_Hz / 2.0 - 0.15  // Z: 上表面偏下15cm
                                                         );

        // 设置根链接初始位姿，沿 x 轴旋转90度
        ChQuaternion<> rot_x_90 = QuatFromAngleX(CH_PI / 2.0);
        ChFrame<> root_init_pose(arm_mount_pos, rot_x_90);
        urdf_parser.SetRootInitPose(root_init_pose);

        // 4. 填充系统 (创建所有连杆和关节)
        urdf_parser.PopulateSystem(sys);

        // std::cout << "[INFO] URDF模型已成功加载到系统" << std::endl;

        // 5. 获取根链接(link1)并固连到基座
        auto link1 = urdf_parser.GetRootChBody();

        // 创建固定关节将根链接连接到body_base
        auto link1_to_base = chrono_types::make_shared<ChLinkMateFix>();
        link1_to_base->Initialize(link1, body_base, ChFrame<>(arm_mount_pos, QUNIT));
        sys.Add(link1_to_base);

        auto link8 = urdf_parser.GetChBody("link8");
        // 创建固定关节将link8连接到body_star_left
        auto link8_to_star = chrono_types::make_shared<ChLinkMateFix>();
        // 使用link8的当前位置作为连接点
        ChFrame<> link8_frame(link8->GetPos() + ChVector3d(0, 0, 0.05), QUNIT);
        link8_to_star->Initialize(link8, body_star_right, link8_frame);
        sys.Add(link8_to_star);

        // 6. 配置关节控制器
        // 关节起始角度
        std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
        std::vector<double> init_angles = {1.5862, 0.0910, 2.1837, 0.1037, -0.9215, 1.4654, 1.8753};

        // 关节目标角度
        std::vector<double> end_angles = {0.8413, 1.1435, 2.1885, 2.1069, -0.2202, 0.7031, 0.4938};
        
        // 关节增益系数
        std::vector<double> Kp_gains = {13.43, 13.58, 15.00, 12.53, 14.32, 11.30, 1.40};
        std::vector<double> Kd_gains = {63.43, 57.14, 60.00, 142.25, 138.94, 120.20, 110.45};

        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            auto motor = urdf_parser.GetChMotor(joint_names[i]);
            if (!motor)
            {
                std::cerr << "[WARNING] 未找到关节: " << joint_names[i] << std::endl;
                continue;
            }

            auto rot_motor = std::dynamic_pointer_cast<ChLinkMotorRotation>(motor);
            if (!rot_motor)
            {
                std::cerr << "[WARNING] " << joint_names[i] << " 不是旋转电机" << std::endl;
                continue;
            }

            // 创建PD控制器
            auto controller = chrono_types::make_shared<TorqueController>();
            controller->target_angle = end_angles[i];    // 目标：终止位姿
            controller->Kp = Kp_gains[i];                // 表5的Kp
            controller->Kd = Kd_gains[i];                // 表5的Kd
            controller->max_torque = 100.0;              // ±100 Nm饱和
            controller->motor = rot_motor;

            // 设置电机的扭矩函数
            urdf_parser.SetMotorFunction(joint_names[i], controller);
        }

        // std::cout << "[INFO] 成功配置 " << arm_motors.size() << " 个关节电机" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] URDF加载失败: " << e.what() << std::endl;
        return 1;
    }

    // 5. 施加初始运动
    // body_base->SetPosDt(ChVector3d(0, 0.5, 0.2));    // 平移初速度
    // body_base->SetAngVelParent(ChVector3d(0.01, 0, 0.05));   // 角速度

    // 6. FEA 可视化配置
    auto mvisualizebeam = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeam->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizebeam->SetColorscaleMinMax(0.0, 1.0);
    mvisualizebeam->SetSmoothFaces(true);
    mvisualizebeam->SetWireframe(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeam);

    /*
    // 添加坐标轴，方便开发调试
    auto axis_body = chrono_types::make_shared<ChBody>();
    axis_body->SetPos(ChVector3d(0, 0, 0));
    axis_body->SetFixed(true);
    axis_body->EnableCollision(false);
    sys.Add(axis_body);
    double axis_len = 5.0;  // 轴长 5米
    double axis_thk = 0.2;  // 轴粗细 0.2米
    // X轴 (红色)
    auto box_x = chrono_types::make_shared<ChVisualShapeBox>(axis_len, axis_thk, axis_thk);
    box_x->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    // 偏移中心，使其从原点出发
    axis_body->AddVisualShape(box_x, ChFrame<>(ChVector3d(axis_len / 2.0, 0, 0)));
    // Y轴 (绿色)
    auto box_y = chrono_types::make_shared<ChVisualShapeBox>(axis_thk, axis_len, axis_thk);
    box_y->SetColor(ChColor(0.0f, 1.0f, 0.0f));
    axis_body->AddVisualShape(box_y, ChFrame<>(ChVector3d(0, axis_len / 2.0, 0)));
    // Z轴 (蓝色)
    auto box_z = chrono_types::make_shared<ChVisualShapeBox>(axis_thk, axis_thk, axis_len);
    box_z->SetColor(ChColor(0.0f, 0.0f, 1.0f));
    axis_body->AddVisualShape(box_z, ChFrame<>(ChVector3d(0, 0, axis_len / 2.0)));
    */

    // 7. VSG 可视化系统设置
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(ChVector2i(1920, 1080));
    vis->SetWindowTitle("Micro-Satellite Docking Scene");
    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->AddCamera(ChVector3d(84.0, -15.0, 3.0), ChVector3d(84.0, 0, 0));
    vis->Initialize();

    // 8. 求解器设置
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(1000);
    solver->SetTolerance(1e-12);
    solver->EnableWarmStart(true);
    solver->EnableDiagonalPreconditioner(true);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // 9. 仿真循环
    while (vis->Run())
    {
        //vis->RenderFrame(ChFrame<>(ChVector3d(0, 0, 0), QUNIT), 10.0);
        vis->Render();
        sys.DoStepDynamics(0.005);
    }

    return 0;
}
