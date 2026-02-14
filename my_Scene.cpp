#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_vsg/ChVisualSystemVSG.h"


using namespace chrono;
using namespace chrono::fea;
using namespace chrono::vsg3d;

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
    double rho = 12140.0;           // 等效密度 kg/m^3
    double E = 5.9773e10;           // 弹性模量 Pa
    double nu = 0.3;                // 泊松比

    msection->SetDensity(rho);
    msection->SetYoungModulus(E);
    msection->SetShearModulusFromPoisson(nu); // 根据 nu 自动计算 G
    msection->SetRayleighDamping(0.01, 0.005);      // 轻微阻尼以保证数值稳定

    // --- 截面几何特性 (核心参数) ---
    double Area = 5.4428e-4;        // 截面积 m^2
    double Iyy = 9.0814e-5;         // Y轴惯性矩 m^4
    double Izz = 9.0814e-5;         // Z轴惯性矩 m^4
    double J = 1.8163e-4;           // 扭转常数 m^4

    // 设置截面特性
    msection->SetArea(Area);
    msection->SetIyy(Iyy);
    msection->SetIzz(Izz);
    msection->SetJ(J);

    // 4. 构建梁 (无约束，自由漂浮)
    double beam_L = 83.24; // 梁总长
    int n_elements = 10;   // 单元数量
    // double beam_width_y = 1.0400;      // TKy, Y方向包络
    double beam_height_z = 0.90603;     // TKz, Z方向包络
    
    ChBuilderBeamEuler builder1;
    ChVector3d beam1_end_B(beam_L, 0, 0);
    builder1.BuildBeam(my_mesh,                  // 网格对象
                      msection,                 // 截面属性
                      n_elements,               // 单元数
                      ChVector3d(0, 0, 0),      // 起点 A
                      beam1_end_B, // 终点 B (沿X轴放置)
                      ChVector3d(0, 1, 0));     // Y轴参考方向 (定义截面朝向)


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
        false
    );

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
    builder2.BuildBeam(my_mesh,                  // 网格对象
                      msection,                 // 截面属性
                      n_elements,               // 单元数
                      ChVector3d(beam2_start_x, 0, 0),      // 起点 A
                      beam2_end_B,              // 终点 B (沿X轴放置)
                      ChVector3d(0, 1, 0));     // Y轴参考方向 (定义截面朝向)

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
        true, false
    );

    double beam2_bottom_z = beam1_bottom_z;
    body_spacecraft->SetPos(ChVector3d(beam2_end_B.x() + sc_Lx / 2.0, 0, beam2_bottom_z));
    body_spacecraft->SetFixed(false);
    body_spacecraft->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.2f, 0.8f)); // 蓝色
    sys.Add(body_spacecraft);
    
    auto link_sc = chrono_types::make_shared<ChLinkMateFix>();
    link_sc->Initialize(builder2.GetLastBeamNodes().back(), body_spacecraft);
    sys.Add(link_sc);

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

    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetModifiedNewton(true);
    mystepper->SetVerbose(false);

    // 9. 仿真循环
    while (vis->Run())
    {
        vis->RenderFrame(ChFrame<>(ChVector3d(0, 0, 0), QUNIT), 10.0);
        vis->Render();
        sys.DoStepDynamics(0.005);
    }

    return 0;
}
