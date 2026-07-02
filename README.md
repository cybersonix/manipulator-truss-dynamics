# Manipulator-Truss Dynamics Simulation

[![License](https://img.shields.io/badge/license-GPL%20v3-blue.svg)](LICENSE)

## 项目概述

本项目基于 **Chrono 物理引擎**，对 **7 自由度机械臂** 与 **柔性桁架耦合系统** 进行动力学仿真分析。仿真结果与 ADAMS 多体动力学软件的计算结果进行对比验证。主要研究内容包括机械臂辅助对接场景的两个关键阶段：

1. **对接阶段（Docking Phase）**：机械臂从初始位形运动到目标位形，完成抓捕对接
2. **相对运动消除阶段（Relative Motion Elimination Phase）**：抓捕完成后消除目标航天器相对于巡游飞行器的残余运动

### 核心技术栈

- **物理引擎**: [Chrono](https://projectchrono.org/)（C++ 多体动力学 + 有限元）
- **可视化**: Chrono::VSG（基于 Vulkan Scene Graph 的实时 3D 渲染）
- **对比验证**: ADAMS 仿真数据（`.tab` 文件）
- **编程语言**: C++17
- **数据可视化**: Python (Matplotlib, Pandas, scienceplots)
- **构建系统**: CMake 3.10+
- **机器人模型**: URDF 格式

## 软件架构

```
manipulator-truss-dynamics/
├── include/                              # 头文件
│   ├── controllers.h                     # 关节控制器（PD控制器 + ChFunction适配器）
│   ├── satellite_docking_model.h         # 卫星对接模型定义（ModelConfig, SatelliteModel）
│   └── scenario.h                        # 场景运行器与数据记录器接口
├── src/                                  # 核心源代码
│   ├── satellite_docking_model.cpp       # 动力学模型构建（柔性梁、刚体、URDF机械臂）
│   └── scenario_runner.cpp               # 仿真循环、VSG可视化、CSV数据记录
├── scenarios/                            # 场景可执行文件入口
│   ├── docking.cpp                       # 对接阶段场景
│   └── relative_motion_elimination.cpp   # 相对运动消除阶段场景
├── scripts/                              # 构建脚本与可视化脚本
│   ├── build.sh                          # 项目构建脚本
│   ├── run.sh                            # 仿真运行脚本
│   ├── docking/                          # 对接阶段可视化
│   │   ├── joints_angle_plot.py
│   │   ├── joints_torque_plot.py
│   │   └── joints_vel_plot.py
│   └── relative_motion_elimination/      # 相对运动消除阶段可视化
│       ├── joints_angle_plot.py
│       ├── joints_torque_plot.py
│       ├── joints_vel_plot.py
│       ├── vibration_plot.py
│       ├── rel_vel_plot.py
│       └── combine_plots.py
├── assets/                               # 资源文件
│   ├── data/                             # ADAMS 对比验证数据
│   │   ├── docking/                      # 对接阶段 ADAMS 数据（TC-02, TC-03）
│   │   └── relative_motion_elimination/  # 相对运动消除阶段 ADAMS 数据
│   └── urdf/                             # 机械臂 URDF 模型
│       └── robot.urdf                    # 7-DoF 机械臂定义
├── results/                              # 仿真结果与可视化输出
│   ├── docking/                          # 对接阶段输出
│   └── relative_motion_elimination/      # 相对运动消除阶段输出
├── CMakeLists.txt                        # CMake 构建配置
└── LICENSE                               # GPL-3.0 许可证
```

### 核心模块说明

| 模块 | 文件 | 功能 |
|------|------|------|
| 动力学模型 | [satellite_docking_model.cpp](src/satellite_docking_model.cpp) | 构建包含柔性桁架（Euler-Bernoulli 梁单元）、对接节点星、机械臂基座、航天器平台和 7-DoF 机械臂的耦合系统 |
| 控制器 | [controllers.h](include/controllers.h) | `PDController` 实现独立关节 PD 控制，`ControllerChFunction` 将控制器适配为 Chrono 的 `ChFunction` 接口 |
| 场景运行器 | [scenario_runner.cpp](src/scenario_runner.cpp) | 管理仿真循环、VSG 实时可视化、CSV 数据记录（关节角度/速度/力矩、对接距离、相对速度、桁架振动位移） |
| 场景入口 | [scenarios/](scenarios/) | 每个 `.cpp` 文件编译为独立的可执行文件，配置特定的控制参数和初始条件 |

## 仿真场景描述

### 动力学模型

仿真系统由以下组件构成（沿 X 轴方向排列）：

```
[左侧对接星] -- [柔性梁1] -- [机械臂基座] -- [机械臂] -- [右侧对接星] -- [柔性梁2] -- [航天器平台]
```

- **柔性梁**：采用 Chrono FEA 模块的 Euler-Bernoulli 梁单元，长度 83.24m，10 个单元
- **机械臂**：7 自由度，通过 URDF 文件定义，安装在基座上方
- **末端执行器**（link8）：固连到右侧对接星，初始间距 1.1m

### 对接阶段 (Docking Phase)

- 机械臂从初始位形运动到目标位形，完成对目标航天器的抓捕
- 采用独立关节 PD 控制，力矩限幅 ±100 Nm
- 仿真时长：300s

**初始位形**：

| Joint 1 | Joint 2 | Joint 3 | Joint 4 | Joint 5 | Joint 6 | Joint 7 |
|---------|---------|---------|---------|---------|---------|---------|
| 1.5862  | 0.0910  | 2.1837  | 0.1037  | -0.9215 | 1.4654  | 1.8753  |

**目标位形**：

| Joint 1 | Joint 2 | Joint 3 | Joint 4 | Joint 5 | Joint 6 | Joint 7 |
|---------|---------|---------|---------|---------|---------|---------|
| 0.8413  | 1.1435  | 2.1885  | 2.1069  | -0.2202 | 0.7031  | 0.4938  |

### 相对运动消除阶段 (Relative Motion Elimination Phase)

- 场景初始位置为抓捕完成状态（对接点相距 1.1m）
- 机械臂保持在起始位形（控制目标角度增量为 0）
- 对目标航天器（右侧对接星 + 柔性梁2 + 航天器平台）施加初始扰动
- 仿真时长：600s

**初始扰动**：

- 线速度：X、Y、Z 方向均为 0.02 m/s
- 角速度：绕整体质心，X、Y、Z 方向均为 0.01°/s

## 控制策略

### 对接阶段

采用独立关节 PD 控制，力矩限幅 ±100 Nm：

| 关节 | Kp (Nm/rad) | Kd (Nm·s/rad) |
|------|-------------|----------------|
| 1    | 13.43       | 63.43          |
| 2    | 13.58       | 57.14          |
| 3    | 15.00       | 60.00          |
| 4    | 12.53       | 142.25         |
| 5    | 14.32       | 138.94         |
| 6    | 11.30       | 120.20         |
| 7    | 1.40        | 110.45         |

### 相对运动消除阶段

采用独立关节 PD 控制，力矩限幅 ±5 Nm：

| 关节 | Kp (Nm/rad) | Kd (Nm·s/rad) |
|------|-------------|----------------|
| 1    | 13.43       | 63.43          |
| 2    | 13.58       | 57.14          |
| 3    | 15.00       | 60.00          |
| 4    | 12.53       | 142.25         |
| 5    | 14.32       | 138.94         |
| 6    | 11.30       | 120.20         |
| 7    | 1.40        | 110.45         |

## 仿真结果与验证

### 输出数据

每次仿真运行后，CSV 数据文件保存至 `results/` 目录，包含以下字段：

| 字段 | 说明 |
|------|------|
| `time` | 仿真时间 (s) |
| `docking_distance` | 对接距离 (m) |
| `relative_vx/y/z` | 相对速度分量 (m/s) |
| `relative_speed` | 相对速度大小 (m/s) |
| `beam2_mid_x/y/z` | 柔性梁2 中点振动位移 (m) |
| `joint*_angle` | 各关节角度 (rad) |
| `joint*_velocity` | 各关节角速度 (rad/s) |
| `joint*_control_torque` | 各关节控制力矩 (Nm) |

### 对比验证

仿真结果与 ADAMS 多体动力学软件的计算结果进行对比验证：

- **对接阶段**：对比 TC-02、TC-03 测试用例的关节角度、速度和控制力矩
- **相对运动消除阶段**：对比关节角度、速度、力矩、相对速度和桁架振动位移

## 快速开始

### 环境要求

- **操作系统**: Ubuntu 20.04+ / CentOS 7+ / Fedora 30+
- **编译器**: GCC 9+（支持 C++17）
- **CMake**: 3.10+
- **Chrono**: 9.0.0+（需从源码编译，启用 `PARSE` 和 `VSG` 模块）
- **Python**: 3.8+（用于数据可视化）

### 安装 Chrono 引擎

Chrono 必须从源码编译安装，并启用 URDF 解析和 VSG 可视化模块：

```bash
git clone https://github.com/projectchrono/chrono.git
cd chrono
git checkout develop

mkdir build && cd build

cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_DEMOS=OFF \
         -DBUILD_TESTING=OFF \
         -DENABLE_MODULE_PARSE=ON \
         -DENABLE_MODULE_VSG=ON

make -j$(nproc)
sudo make install
```

参考文档：

- [Chrono 安装指南](https://api.projectchrono.org/9.0.0/tutorial_install_chrono.html)
- [Parser 模块安装](https://api.projectchrono.org/9.0.0/module_parsers_installation.html)
- [VSG 模块安装](https://api.projectchrono.org/9.0.0/module_vsg_installation.html)

### 安装 Python 依赖

```bash
pip install matplotlib numpy pandas scienceplots cairosvg pillow
```

### 构建项目

```bash
# 使用构建脚本
./scripts/build.sh

# 或手动构建
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

构建产物：

- `build/bin/docking` — 对接阶段可执行文件
- `build/bin/relative_motion_elimination` — 相对运动消除阶段可执行文件
- `build/libsatellite_core.a` — 核心静态库

### 运行仿真

```bash
# 使用运行脚本
./scripts/run.sh

# 或直接运行
./build/bin/docking
./build/bin/relative_motion_elimination
```

> **注意**：运行前需确保 Chrono 动态库在 `LD_LIBRARY_PATH` 中，可参考 `scripts/run.sh` 中的环境变量设置。

### 可视化结果

```bash
# 相对运动消除阶段
python3 scripts/relative_motion_elimination/joints_angle_plot.py
python3 scripts/relative_motion_elimination/joints_vel_plot.py
python3 scripts/relative_motion_elimination/joints_torque_plot.py
python3 scripts/relative_motion_elimination/vibration_plot.py
python3 scripts/relative_motion_elimination/rel_vel_plot.py
python3 scripts/relative_motion_elimination/combine_plots.py

# 对接阶段
python3 scripts/docking/joints_angle_plot.py
python3 scripts/docking/joints_torque_plot.py
python3 scripts/docking/joints_vel_plot.py
```

## 结果展示

<p align="center">
  <img src="results/relative_motion_elimination/combined_analysis_results.png" alt="Project Visualization" width="100%">
</p>

## 贡献

欢迎提交 Issue 和 Pull Request 来帮助改进这个项目。

## 许可证

本项目采用 GNU General Public License v3.0 - 查看 [LICENSE](LICENSE) 文件了解更多详情。
