#!/bin/bash
# STURDeCAM88_CUOAGX カーネル/ドライバビルド用の環境変数を設定する。
# 使い方: source setup_build_env.sh   (実行ではなく source すること)

export TOP_DIR=/home/yy/ME450/camera_driver_build

export TOOL_CHAIN=$TOP_DIR/tool_chain
export RELEASE_PACK_DIR=$TOP_DIR/STURDeCAM88_CUOAGX_FORGE_ORIN_AGX_L4T35.4.1_R05
export L4T_DIR=$TOP_DIR/Linux_for_Tegra
export NVIDIA_SRC=$TOP_DIR/kernel_sources/
export TEGRA_KERNEL_OUT=$TOP_DIR/kernel_out
export LDK_ROOTFS_DIR=$L4T_DIR/rootfs
export INSTALL_MOD_PATH=$L4T_DIR/rootfs

export ARCH=arm64
export CROSS_COMPILE=$TOOL_CHAIN/bin/aarch64-linux-

# クロスコンパイラを PATH の先頭に追加 (重複追加を避ける)
case ":$PATH:" in
    *":$TOOL_CHAIN/bin:"*) ;;
    *) export PATH=$TOOL_CHAIN/bin:$PATH ;;
esac
