import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image
import numpy as np
from pathlib import Path
import io
import cairosvg

# 设置路径
result_dir = Path('results/relative_motion_elimination')

# 定义要组合的SVG文件
svg_files = [
    'joints_angle.svg',
    'joints_vel.svg',
    'joints_torque.svg',
    'rel_velocity_comparison.svg'
]

# 创建2x2子图布局
fig, axes = plt.subplots(2, 2, figsize=(20, 15))
fig.suptitle('Relative Motion Elimination Analysis Results', fontsize=20, fontweight='bold')

# 子图标题
titles = [
    'Joints Angle',
    'Joints Velocity',
    'Joints Torque',
    'Relative Velocity Comparison'
]

# 读取并显示每个SVG文件
for i, (svg_file, title) in enumerate(zip(svg_files, titles)):
    row = i // 2
    col = i % 2
    
    try:
        svg_path = result_dir / svg_file
        
        # 使用cairosvg将SVG转换为PNG内存数据
        png_data = cairosvg.svg2png(url=str(svg_path), output_width=800, output_height=600)
        # 将PNG数据转换为PIL图像
        img = Image.open(io.BytesIO(png_data))
        
        # 显示图像
        axes[row, col].imshow(img)
        axes[row, col].set_title(title, fontsize=16, pad=20)
        axes[row, col].axis('off')
        
    except Exception as e:
        # 如果SVG读取失败，则显示错误信息
        axes[row, col].text(0.5, 0.5, f'Error loading\n{svg_file}\n{str(e)[:50]}...', 
                           ha='center', va='center', transform=axes[row, col].transAxes,
                           fontsize=12, color='red')
        axes[row, col].set_title(title, fontsize=16)
        axes[row, col].axis('off')

# 调整布局
plt.tight_layout()
plt.subplots_adjust(top=0.93)

# 保存为PNG格式
output_path = result_dir / 'combined_analysis_results.png'
plt.savefig(output_path, dpi=1200, bbox_inches='tight', pad_inches=0.1, format='png')

print(f"Combined plot saved to: {output_path}")

# 显示图形
plt.show()