#!/bin/bash

# 设置退出条件：任何命令失败时退出
set -e

# 定义安装路径
SOURCE_DIR="$(pwd)"
INSTALL_DIR="$(pwd)/3rdparty/vtk"
VTK_VERSION="8.2.0"

# 打印开始信息
echo "开始编译并安装 VTK ${VTK_VERSION} 到 ${INSTALL_DIR}"

# 1. 安装依赖
echo "安装必要的依赖..."
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y git build-essential libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev libxt-dev libxmu-dev libxi-dev libpng-dev libjpeg-dev zlib1g-dev -y

# 2. 检查依赖是否安装
echo "检查依赖..."
for pkg in git libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev libxt-dev libxmu-dev libxi-dev libpng-dev libjpeg-dev zlib1g-dev; do
    dpkg -l | grep -qw $pkg || { echo "依赖 $pkg 未安装！"; exit 1; }
done

# 3. 下载 VTK 源码
echo "下载 VTK ${VTK_VERSION} 源码..."
mkdir -p "${SOURCE_DIR}"
cd "${SOURCE_DIR}"
if [ ! -d "VTK-${VTK_VERSION}" ]; then
    wget https://www.vtk.org/files/release/8.2/VTK-${VTK_VERSION}.tar.gz
    tar -xzf VTK-${VTK_VERSION}.tar.gz
fi
cd VTK-${VTK_VERSION}

# 4. 清空旧构建目录
echo "清空旧构建目录..."
rm -rf build
mkdir -p build && cd build

# 4. 配置编译环境
echo "配置 CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DVTK_RENDERING_BACKEND=OpenGL2 \
    -DVTK_MODULE_ENABLE_VTK_CommonCore=ON \
    -DVTK_MODULE_ENABLE_VTK_CommonDataModel=ON \
    -DVTK_MODULE_ENABLE_VTK_CommonExecutionModel=ON \
    -DVTK_MODULE_ENABLE_VTK_IOImage=ON \
    -DVTK_MODULE_ENABLE_VTK_IOLegacy=ON \
    -DVTK_MODULE_ENABLE_VTK_IOGeometry=ON \
    -DVTK_MODULE_ENABLE_VTK_IOPLY=ON \
    -DVTK_MODULE_ENABLE_VTK_ImagingCore=ON \
    -DVTK_MODULE_ENABLE_VTK_FiltersCore=ON \
    -DVTK_MODULE_ENABLE_VTK_FiltersGeneral=ON \
    -DVTK_MODULE_ENABLE_VTK_RenderingCore=ON \
    -DVTK_MODULE_ENABLE_VTK_RenderingOpenGL2=ON \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    -DCMAKE_VERBOSE_MAKEFILE=ON

# 5. 编译源码
echo "编译 VTK..."
make -j$(nproc)

# 6. 安装 VTK
echo "安装 VTK 到 ${INSTALL_DIR}..."
make install

# 7. 验证安装
echo "验证安装..."
if [ -f "${INSTALL_DIR}/lib/cmake/vtk-8.2/VTKConfig.cmake" ] && \
   [ -f "${INSTALL_DIR}/lib/libvtkCommonCore-8.2.so" ] && \
   [ -f "${INSTALL_DIR}/lib/libvtkIOPLY-8.2.so" ] && \
   [ -f "${INSTALL_DIR}/lib/libvtkIOImage-8.2.so" ]; then
    echo "安装成功！VTKConfig.cmake 位于 ${INSTALL_DIR}/lib/cmake/vtk-8.2"
else
    echo "安装失败，请检查日志！"
    exit 1
fi

# 8. 清除
echo "清除..."
cd ${SOURCE_DIR}
rm -rf VTK-8.2.0 && rm -rf VTK-8.2.0.tar.gz

echo "VTK ${VTK_VERSION} 已成功安装到 ${INSTALL_DIR}"
