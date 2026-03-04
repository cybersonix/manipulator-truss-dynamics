from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scienceplots

adams_result_dir = Path('assets/data/relative_motion_elimination')

# 使用科学样式
plt.style.use(['science', 'ieee', 'no-latex'])
fig, ax = plt.subplots(figsize=(4, 2))

# 读取Adams相对速度数据
tab_file_path = adams_result_dir / 'rel_vel.tab'
df_tab = pd.read_csv(tab_file_path, skiprows=1, sep='\t')

# 清理列名（去除引号和多余空格）
df_tab.columns = df_tab.columns.str.replace('"', '').str.strip()

# 提取时间数据和相对速度数据
time_tab = np.array(df_tab['Time'].values)
rel_vel_tab = np.array(df_tab['relative_vel'].values)

# 读取Chrono相对速度数据
csv_file_path = Path('results/relative_motion_elimination/relative_motion_elimination.csv')
df_csv = pd.read_csv(csv_file_path)

# 提取时间数据和相对速度数据
time_csv = np.array(pd.to_numeric(df_csv['time'], errors='coerce').values)
rel_vel_csv = np.array(pd.to_numeric(df_csv['relative_speed'], errors='coerce').values)

# 绘制对比图
ax.plot(time_tab, rel_vel_tab, label='Adams')
ax.plot(time_csv, rel_vel_csv, label='Chrono')

# 设置图表属性
ax.set_xlabel('Time (s)')
ax.set_xlim(0, 300)
ax.set_ylabel('Relative Velocity (m/s)')
ax.legend(loc='best')

# 调整布局并保存
plt.tight_layout()
plt.savefig(Path('results/relative_motion_elimination/rel_velocity.svg'), bbox_inches='tight', pad_inches=0.01)