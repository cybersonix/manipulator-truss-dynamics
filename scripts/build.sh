#!/bin/bash

echo "开始构建项目..."

# 检查必要工具
if ! command -v cmake &> /dev/null; then
    echo "错误: 未找到CMake，请先安装CMake"
    exit 1
fi

if ! command -v make &> /dev/null; then
    echo "错误: 未找到make，请先安装make"
    exit 1
fi

rm -rf build
# 创建构建目录
echo "创建构建目录..."
mkdir -p build
cd build

# 配置项目
echo "配置CMake项目..."

cmake .. \
    -DPYTHON_EXECUTABLE=$ENV{HOME}/miniconda3/bin/python3.13 \
    -DPYTHON_INCLUDE_DIR=$ENV{HOME}/miniconda3/include/python3.13 \
    -DPYTHON_LIBRARY=$ENV{HOME}/miniconda3/lib/libpython3.13.so \
    -DPython3_ROOT_DIR=$ENV{HOME}/miniconda3 \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$ENV{HOME}/urdf_install;$ENV{HOME}/vsgFramework/install;/usr/local/chrono" \
    -DCMAKE_EXE_LINKER_FLAGS="-L/home/sun/miniconda3/lib -lpython3.13" \

if [ $? -ne 0 ]; then
    echo "错误: CMake配置失败"
    exit 1
fi

# 编译项目
echo "开始编译项目..."
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

if [ $? -ne 0 ]; then
    echo "错误: 编译失败"
    exit 1
fi

echo ""
echo "构建成功！"
echo ""
echo "编译完成的程序位于: build/bin/"
echo ""
