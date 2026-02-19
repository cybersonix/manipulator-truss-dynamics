#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
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

    // =============================================================================
    // D. 8自由度机械臂建模 (修正后)
    // =============================================================================
    
    // 1. 机械臂配置参数 (单位: m)
    // 连杆长度 (改为8个连杆)
    std::vector<double> arm_lengths = {0.05, 0.05, 0.15, 0.863, 0.863, 0.15, 0.05, 0.05}; 
    // 关节类型: 0=Rz(Yaw), 1=Ry(Pitch), 2=Rx(Roll)
    // 关节配置: 固连-Rz-Ry-Rx-Ry-Rx-Ry-Rz
    std::vector<int> joint_types = {-1, 2, 1, 0, 1, 0, 1, 2}; 

    // 初始关节角度
    // 注意：第一个元素对应 joint_types[0] (Fixed)，角度设为0
    std::vector<double> init_angles = {0.0, 1.5862, 0.0910, 2.1837, 0.1037, -0.9215, 1.4654, 1.8753};
    
    // 连杆几何参数，假设为均质圆柱体
    double arm_radius = 0.05; 
    
    // 2. 动力学属性计算 (质量按臂长比例分配)
    double arm_total_mass = 82.4;
    
    // 计算所有连杆的总长度
    double total_arm_len = 0;
    for (double l : arm_lengths) total_arm_len += l;

    // 计算等效密度
    // m_i = rho * V_i = rho * (Area * L_i)
    // 若 rho 为常数，则 m_i 正比于 L_i
    double total_volume = CH_PI * std::pow(arm_radius, 2) * total_arm_len;
    double arm_density = arm_total_mass / total_volume;

    // 3. 确定机械臂安装基点
    auto arm_base_body = body_base;
    ChVector3d arm_mount_pos = arm_base_body->GetPos() + ChVector3d(base_Lx / 2.0 - 0.15, -base_Wy / 2.0, base_Hz / 2.0 - 0.15);
    
    std::shared_ptr<ChBody> prev_body = arm_base_body;
    ChVector3d curr_joint_pos = arm_mount_pos;
    ChQuaternion<> curr_abs_rot = arm_base_body->GetRot();
    
    // 循环创建 8 个连杆和关节
    for (size_t i = 0; i < arm_lengths.size(); ++i) {
        double L = arm_lengths[i];
        int type = joint_types[i];
        double angle = init_angles[i];
        
        // --- A. 计算当前关节产生的相对旋转 ---
        ChQuaternion<> joint_rel_rot = QUNIT; // 相对旋转
        ChVector3d local_axis(0, 1, 0);       // 关节在上一级坐标系中的旋转轴
        if (type == -1) {
            // 固连
        } else if (type == 0) { // Rx
            joint_rel_rot = QuatFromAngleX(angle);
            local_axis = ChVector3d(1, 0, 0);
        } else if (type == 1) { // Ry
            joint_rel_rot = QuatFromAngleY(angle);
            local_axis = ChVector3d(0, 1, 0);
        } else if (type == 2) { // Rz
            joint_rel_rot = QuatFromAngleZ(angle);
            local_axis = ChVector3d(0, 0, 1);
        }
        // 更新当前的绝对姿态：先应用上一级的旋转，再叠加当前关节的旋转
        // 注意：Chrono中连杆刚体通常随关节旋转
        ChQuaternion<> prev_abs_rot = curr_abs_rot; // 保存上一级姿态用于定义关节轴
        curr_abs_rot = curr_abs_rot * joint_rel_rot;
        // --- B. 创建连杆刚体 ---
        // 连杆位置：从当前关节位置，沿新的Z轴方向移动 L/2 (假设圆柱沿Z轴生长)
        ChVector3d link_center = curr_joint_pos + curr_abs_rot.Rotate(ChVector3d(0, 0, L / 2.0));
        auto body_link = chrono_types::make_shared<ChBodyEasyCylinder>(
            ChAxis::Z,      // 几何轴向
            arm_radius,     L, 
            arm_density,    
            true, false
        );
        
        body_link->SetPos(link_center);
        body_link->SetRot(curr_abs_rot); // 设置为计算出的初始姿态
        
        // 颜色交替
        if (i % 2 == 0) body_link->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.4f, 0.1f));
        else            body_link->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.2f, 0.8f));
        
        sys.Add(body_link);
        // --- C. 创建关节 ---
        if (type == -1) {
            // 固连类型
            auto link_fix = chrono_types::make_shared<ChLinkMateFix>();
            // Initialize 会锁定当前的相对位置/姿态
            link_fix->Initialize(prev_body, body_link, ChFrame<>(curr_joint_pos, curr_abs_rot));
            sys.Add(link_fix);
        } else {
            // 旋转关节
            auto link_rev = chrono_types::make_shared<ChLinkLockRevolute>();
            
            // 计算关节坐标系：原点在 curr_joint_pos
            // 关键：关节坐标系的 Z 轴必须对齐物理旋转轴
            // 物理旋转轴在世界坐标系下 = prev_abs_rot.Rotate(local_axis)
            ChVector3d axis_in_world = prev_abs_rot.Rotate(local_axis);
            
            // 构建一个Z轴指向axis_in_world的四元数
            ChQuaternion<> joint_frame_rot;
            
            // --- 修改开始：手动实现从 Z 轴到 axis_in_world 的旋转 ---
            ChVector3d target_dir = axis_in_world;
            target_dir.Normalize(); // 确保目标向量归一化
            // 计算点积。因为起始向量是 (0,0,1)，点积直接等于 z 分量
            double dot = target_dir.z();
            if (dot >= 1.0 - 1e-9) {
                // 向量同向，无需旋转
                joint_frame_rot.SetUnit();
            } else if (dot <= -1.0 + 1e-9) {
                // 向量反向，绕 X 轴旋转 180 度 (PI)
                joint_frame_rot.SetFromAngleAxis(CH_PI, ChVector3d(1, 0, 0));
            } else {
                // 一般情况：计算旋转角和旋转轴
                double angle = std::acos(dot);
                
                // 计算叉积：(0,0,1) x (x,y,z) = (-y, x, 0)
                ChVector3d rot_axis(-target_dir.y(), target_dir.x(), 0);
                rot_axis.Normalize();
                
                joint_frame_rot.SetFromAngleAxis(angle, rot_axis);
            }
            // --- 修改结束 ---
            
            link_rev->Initialize(prev_body, body_link, ChFrame<>(curr_joint_pos, joint_frame_rot));
            sys.Add(link_rev);
        }
        // --- D. 更新状态为下一轮循环做准备 ---
        prev_body = body_link;
        // 下一个关节的位置 = 当前位置 + 沿连杆轴向移动全长
        curr_joint_pos = curr_joint_pos + curr_abs_rot.Rotate(ChVector3d(0, 0, L));
    }
    // // 添加末端抓手 (End Effector) 视觉示意
    // auto body_ee = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, 0.05, 0.2, 100.0, true, false);
    // body_ee->SetPos(curr_joint_pos + ChVector3d(0, 0, 0.1)); // 0.1 是 ee 长度的一半
    // body_ee->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.2f, 0.2f)); // 红色
    // sys.Add(body_ee);
    
    // auto link_ee = chrono_types::make_shared<ChLinkMateFix>();
    // link_ee->Initialize(prev_body, body_ee);
    // sys.Add(link_ee);


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
