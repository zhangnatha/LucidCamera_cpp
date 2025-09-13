#!/bin/bash

# 设置退出条件：任何命令失败时退出
set -e

# 定义安装路径
SOURCE_DIR="$(pwd)"
INSTALL_DIR="$(pwd)/3rdparty/pcl"
VTK_DIR="$(pwd)/3rdparty/vtk"
PCL_VERSION="1.12.0"

# 打印开始信息
echo "开始编译并安装 PCL ${PCL_VERSION} 到 ${INSTALL_DIR}"

# 1. 安装依赖
echo "安装必要的依赖..."
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y git build-essential libflann-dev libeigen3-dev libboost-all-dev libqhull-dev -y

# 2. 检查 VTK 是否存在
if [ ! -f "${VTK_DIR}/lib/cmake/vtk-8.2/VTKConfig.cmake" ]; then
    echo "错误：VTK 未安装或 ${VTK_DIR}/lib/cmake/vtk-8.2/VTKConfig.cmake 缺失！请先运行 build_vtk.sh"
    exit 1
fi

# 3. 下载 PCL 源码
echo "下载 PCL ${PCL_VERSION} 源码..."
mkdir -p "${SOURCE_DIR}"
cd "${SOURCE_DIR}"
if [ ! -d "pcl" ]; then
    git clone https://github.com/PointCloudLibrary/pcl.git
fi
cd pcl
git checkout pcl-${PCL_VERSION}

# 4. 配置编译环境
echo "配置 CMake..."
mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DVTK_DIR="${VTK_DIR}/lib/cmake/vtk-8.2" \
    -DBUILD_visualization=ON \
    -DBUILD_examples=OFF \
    -DBUILD_tools=OFF \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"

# 5. 编译源码
echo "编译 PCL..."
make -j$(($(nproc) / 2))

# 6. 安装 PCL
echo "安装 PCL 到 ${INSTALL_DIR}..."
make install

# 7. 验证安装
echo "验证安装..."
if [ -f "${INSTALL_DIR}/share/pcl-1.12/PCLConfig.cmake" ] && [ -f "${INSTALL_DIR}/lib/libpcl_io.so" ]; then
    echo "安装成功！PCLConfig.cmake 位于 ${INSTALL_DIR}/share/pcl-1.12"
else
    echo "安装失败，请检查日志！"
    exit 1
fi

# 8. 清除
echo "清除..."
cd ${SOURCE_DIR}
rm -rf pcl
echo "PCL ${PCL_VERSION} 已成功安装到 ${INSTALL_DIR}"
