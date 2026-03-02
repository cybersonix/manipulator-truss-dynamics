from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scienceplots

adams_result_dir = Path('assets/data/docking')

# 创建 7 个子图（4 行 2 列）
plt.style.use(['science', 'ieee', 'no-latex'])
fig, axes = plt.subplots(4, 2, figsize=(8, 6), sharex=True)
axes_flat = axes.flatten()

csv_file_path = Path('results/docking/docking.csv')
df_csv = pd.read_csv(csv_file_path)

for i in range(7):
    tab_file_path = adams_result_dir / f'VTORQUE_{i+1}TC-02.tab'

    # skiprows=7 跳过前7行头部信息
    # header=0 表示第8行（跳过7行后的第1行）作为列名
    # sep='\t' 使用制表符分隔
    # engine='python' 更好地处理复杂格式
    df_tab = pd.read_csv(
        tab_file_path, skiprows=7, header=0, sep='\t', engine='python', quoting=3
    )  # QUOTE_NONE，忽略引号

    # 清理列名（去除引号和多余空格）
    df_tab.columns = df_tab.columns.str.replace('"', '').str.strip()

    # 提取时间和力矩数据
    time_tab = np.array(df_tab.iloc[:, 0].values)
    torque_tab = np.array(df_tab.iloc[:, 5].values)

    # 提取时间和力矩列
    time_csv = np.array(pd.to_numeric(df_csv['time'], errors='coerce').values)
    torque_csv = -np.array(
        pd.to_numeric(df_csv[f'joint{i + 1}_control_torque'], errors='coerce').values
    )

    ax = axes_flat[i]
    # 绘制.tab文件的数据（Adams仿真结果）
    ax.plot(
        time_tab,
        torque_tab,
        label='Adams',
        # color='blue',
        # linewidth=2,
        # linestyle='-',
    )

    # 绘制.csv文件的数据
    ax.plot(
        time_csv,
        torque_csv,
        label='Chrono',
        # color='red',
        # linewidth=2,
        # linestyle='--',
    )

    # 设置图表属性
    ax.set_xlabel('Time (s)')
    ax.set_xlim(0, 300)
    ax.set_ylabel('Torque (N·m)')
    ax.set_title(f'Joint {i+1}')
    ax.legend(loc='best')
    # plt.grid(True, linestyle='--', alpha=0.7)

# plt.tight_layout()
for ax in axes.flat:
    ax.tick_params(labelbottom=True)  # 强制所有子图显示 x 轴刻度标签
axes_flat[7].set_visible(False)
plt.tight_layout(rect=(0, 0, 1, 0.95))
plt.savefig(Path('results/docking/joints_torque.svg'), bbox_inches='tight', pad_inches=0.01)

# plt.show()
