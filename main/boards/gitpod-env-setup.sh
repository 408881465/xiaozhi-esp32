#!/bin/bash

# 设置错误处理：任何命令失败则退出脚本
set -e

# 安装依赖
echo "安装依赖包..."
sudo apt-get install -y git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

# 创建esp目录并克隆代码
echo "创建ESP目录并克隆代码..."
mkdir -p ~/esp
cd ~/esp
if [ ! -d "esp-idf-v5.5" ]; then
    git clone -b v5.5 --recursive https://github.com/espressif/esp-idf.git esp-idf-v5.5
else
    echo "ESP-IDF目录已存在，跳过克隆"
fi

# 安装ESP-IDF
echo "安装ESP-IDF..."
cd ~/esp/esp-idf-v5.5
./install.sh esp32

# 配置环境变量 - 修改为更安全的方式
echo "配置环境变量..."
if ! grep -q "esp-idf-v5.5/export.sh" ~/.bashrc; then
    echo 'alias get_idf=". $HOME/esp/esp-idf-v5.5/export.sh"' >> ~/.bashrc
fi

# 直接在当前shell中设置环境变量，而不是source整个.bashrc
export IDF_PATH=~/esp/esp-idf-v5.5
. $IDF_PATH/export.sh

# 清理并构建项目
echo "清理并构建项目..."
cd /workspace/xiaozhi-esp32
# 使用find命令时添加-safe选项防止权限错误
find . -name "CMakeCache.txt" -o -name "CMakeFiles" -o -name ".cmake" -print0 2>/dev/null | xargs -0 rm -rf 2>/dev/null || true
rm -rf build sdkconfig sdkconfig.old 2>/dev/null || true

# 设置构建环境
export IDF_USE_NINJA=1
export CMAKE_GENERATOR=Ninja

# 设置目标并构建
echo "设置目标esp32s3..."
idf.py set-target esp32s3

echo "请手动完成menuconfig配置，完成后保存退出即可继续构建"
idf.py menuconfig

echo "开始构建项目..."
idf.py build

# 复制并打包二进制文件
echo "复制并打包二进制文件..."
set +e
cp build/bootloader/bootloader.bin \
   build/partition_table/partition-table.bin \
   build/ota_data_initial.bin \
   build/srmodels/srmodels.bin \
   build/xiaozhi.bin . 2>/dev/null
# 重新启用错误退出
set -e

# 打包所有存在的bin文件
if ls *.bin >/dev/null 2>&1; then
    zip -r xiaozhi.zip *.bin
    rm *.bin
    echo "打包完成！"
else
    echo "警告：没有找到任何.bin文件进行打包"
fi

# zip -r xiaozhi.zip *.bin
# rm *.bin

echo "所有操作已完成！"