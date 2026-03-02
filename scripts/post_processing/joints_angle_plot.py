from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scienceplots

adams_result_dir = Path('assets/data/docking')

init_angles = np.array([1.5862, 0.0910, 2.1837, 0.1037, -0.9215, 1.4654, 1.8753])
final_angles = np.array([0.8413, 1.1435, 2.1885, 2.1069, -0.2202, 0.7031, 0.4938])

# 创建 7 个子图（4 行 2 列）
plt.style.use(['science', 'ieee', 'no-latex'])
fig, axes = plt.subplots(4, 2, figsize=(8, 6), sharex=True)
axes_flat = axes.flatten()

csv_file_path = Path('results/docking.csv')
df_csv = pd.read_csv(csv_file_path)

for i in range(7):
    tab_file_path = adams_result_dir / f'J{i+1}_angleTC-02.tab'

    # skiprows=7 跳过前7行头部信息
    # header=0 表示第8行（跳过7行后的第1行）作为列名
    # sep='\t' 使用制表符分隔
    # engine='python' 更好地处理复杂格式
    df_tab = pd.read_csv(
        tab_file_path, skiprows=7, header=0, sep='\t', engine='python', quoting=3
    )  # QUOTE_NONE，忽略引号

    # 清理列名（去除引号和多余空格）
    df_tab.columns = df_tab.columns.str.replace('"', '').str.strip()

    # 提取时间和角度数据
    # 根据文件结构，第一列是TIME，第二列是Q（角度）
    time_tab = np.array(df_tab.iloc[:, 0].values)  # 时间列
    angle_tab = np.array(df_tab.iloc[:, 1].values)  # 角度列
    # 转换成弧度
    angle_tab = np.deg2rad(angle_tab)
    angle_tab = angle_tab + init_angles[i]

    # 提取时间和joint_angle列
    time_csv = np.array(pd.to_numeric(df_csv['time'], errors='coerce').values)
    angle_csv = -np.array(
        pd.to_numeric(df_csv[f'joint{i + 1}_angle'], errors='coerce').values
    )

    # 截断到前300秒
    # time_csv = time_csv[:5766]
    # angle_csv = angle_csv[:5766]
    angle_csv = angle_csv + init_angles[i]

    ax = axes_flat[i]
    # 绘制.tab文件的数据（Adams仿真结果）
    ax.plot(
        time_tab,
        angle_tab,
        label='Adams',
        # color='blue',
        # linewidth=2,
        # linestyle='-',
    )

    # 绘制.csv文件的数据（joints_angle）
    ax.plot(
        time_csv,
        angle_csv,
        label='Chrono',
        # color='red',
        # linewidth=2,
        # linestyle='--',
    )

    # 绘制最终角度基准线
    ax.axhline(y=final_angles[i], color='green', linestyle=':', label='Target Angle')

    # 设置图表属性
    ax.set_xlabel('Time (s)')
    ax.set_xlim(0, 300)
    ax.set_ylabel('Angle (rad)')
    ax.set_title(f'Joint {i+1}')
    ax.legend(loc='best')
    # plt.grid(True, linestyle='--', alpha=0.7)

# plt.tight_layout()
for ax in axes.flat:
    ax.tick_params(labelbottom=True)  # 强制所有子图显示 x 轴刻度标签
axes_flat[7].set_visible(False)
plt.tight_layout(rect=(0, 0, 1, 0.95))
plt.savefig(Path('results/joints_angle.svg'), bbox_inches='tight', pad_inches=0.01)

# plt.show()
