from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scienceplots

adams_result_dir = Path('assets/data/relative_motion_elimination')

# 使用科学样式
plt.style.use(['science', 'ieee', 'no-latex'])

# 创建包含四个子图的图形（总位移 + X/Y/Z方向位移）
fig, axs = plt.subplots(2, 2, figsize=(8, 6))
axs = axs.flatten()

# 读取Adams振动数据
tab_file_path = adams_result_dir / 'vibration.tab'
df_tab = pd.read_csv(tab_file_path, skiprows=1, sep='\t')

# 清理列名（去除引号和多余空格）
df_tab.columns = df_tab.columns.str.replace('"', '').str.strip()

# 提取时间数据和位移数据
time_tab = np.array(df_tab['Time'].values)
x_disp_tab = np.array(df_tab['X'].values)
y_disp_tab = np.array(df_tab['Y'].values)
z_disp_tab = np.array(df_tab['Z'].values)

# 计算总位移
disp_tab = np.sqrt(x_disp_tab**2 + y_disp_tab**2 + z_disp_tab**2)

# 读取Chrono振动数据
csv_file_path = Path(
    'results/relative_motion_elimination/relative_motion_elimination.csv'
)
df_csv = pd.read_csv(csv_file_path)

# 提取时间数据和位移数据
time_csv = np.array(pd.to_numeric(df_csv['time'], errors='coerce').values)
x_disp_csv = np.array(pd.to_numeric(df_csv['beam2_mid_x'], errors='coerce').values)
y_disp_csv = np.array(pd.to_numeric(df_csv['beam2_mid_y'], errors='coerce').values)
z_disp_csv = np.array(pd.to_numeric(df_csv['beam2_mid_z'], errors='coerce').values)

# 计算总位移
disp_csv = np.sqrt(x_disp_csv**2 + y_disp_csv**2 + z_disp_csv**2)

# 绘制总位移对比图
axs[0].plot(time_tab, disp_tab, label='Adams')
axs[0].plot(time_csv, disp_csv, label='Chrono')
axs[0].set_xlabel('Time (s)')
axs[0].set_xlim(0, 300)
axs[0].set_ylabel('Total Displacement (m)')
axs[0].legend(loc='best')
axs[0].set_title('Total Displacement')

# 绘制X方向位移对比图
axs[1].plot(time_tab, x_disp_tab, label='Adams')
axs[1].plot(time_csv, x_disp_csv, label='Chrono')
axs[1].set_xlabel('Time (s)')
axs[1].set_xlim(0, 300)
axs[1].set_ylabel('X Displacement (m)')
axs[1].legend(loc='best')
axs[1].set_title('X Direction Displacement')

# 绘制Y方向位移对比图
axs[2].plot(time_tab, y_disp_tab, label='Adams')
axs[2].plot(time_csv, y_disp_csv, label='Chrono')
axs[2].set_xlabel('Time (s)')
axs[2].set_xlim(0, 300)
axs[2].set_ylabel('Y Displacement (m)')
axs[2].legend(loc='best')
axs[2].set_title('Y Direction Displacement')

# 绘制Z方向位移对比图
axs[3].plot(time_tab, z_disp_tab, label='Adams')
axs[3].plot(time_csv, z_disp_csv, label='Chrono')
axs[3].set_xlabel('Time (s)')
axs[3].set_xlim(0, 300)
axs[3].set_ylabel('Z Displacement (m)')
axs[3].legend(loc='best')
axs[3].set_title('Z Direction Displacement')

# 调整布局并保存
plt.tight_layout()
plt.savefig(
    Path('results/relative_motion_elimination/vibration_displacement.svg'),
    bbox_inches='tight',
    pad_inches=0.01,
)
