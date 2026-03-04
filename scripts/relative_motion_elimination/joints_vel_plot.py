from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scienceplots

adams_result_dir = Path('assets/data/relative_motion_elimination')

# 创建 7 个子图（4 行 2 列）
plt.style.use(['science', 'ieee', 'no-latex'])
fig, axes = plt.subplots(4, 2, figsize=(8, 6), sharex=True)
axes_flat = axes.flatten()

csv_file_path = Path('results/relative_motion_elimination/relative_motion_elimination.csv')
df_csv = pd.read_csv(csv_file_path)

# 读取Adams速度数据
tab_file_path = adams_result_dir / 'velocities.tab'
df_tab = pd.read_csv(tab_file_path, skiprows=1, sep='\t')

# 清理列名（去除引号和多余空格）
df_tab.columns = df_tab.columns.str.replace('"', '').str.strip()

# 提取时间数据
time_tab = np.array(df_tab['Time'].values)

for i in range(7):
    # 提取Adams速度数据
    vel_tab = np.array(df_tab[f'J{i+1}_vel'].values)
    
    # 提取Chrono速度数据
    time_csv = np.array(pd.to_numeric(df_csv['time'], errors='coerce').values)
    vel_csv = np.array(
        pd.to_numeric(df_csv[f'joint{i + 1}_velocity'], errors='coerce').values
    )
    
    # 转换为度/秒
    vel_csv_deg = np.rad2deg(vel_csv)

    ax = axes_flat[i]
    # 绘制.tab文件的数据（Adams仿真结果）
    ax.plot(
        time_tab,
        vel_tab,
        label='Adams',
    )

    # 绘制.csv文件的数据（Chrono仿真结果）
    ax.plot(
        time_csv,
        vel_csv_deg,
        label='Chrono',
    )

    # 设置图表属性
    ax.set_xlabel('Time (s)')
    ax.set_xlim(0, 300)
    ax.set_ylabel('Vel (°/s)')
    ax.set_title(f'Joint {i+1}')
    ax.legend(loc='best')

# 调整布局
for ax in axes.flat:
    ax.tick_params(labelbottom=True)
axes_flat[7].set_visible(False)
plt.tight_layout(rect=(0, 0, 1, 0.95))
plt.savefig(Path('results/relative_motion_elimination/joints_vel.svg'), bbox_inches='tight', pad_inches=0.01)