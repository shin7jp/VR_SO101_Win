#!/usr/bin/env bash
# Gazebo (ogre v1, ヘッドレス) 用の GL 設定。`source` して使う。
#
# WSL2 の Mesa D3D12 (GPU パススルー) は NVIDIA では動くが、Intel UHD 770 では
# カメラ画像の読み出しが壊れる (全画素 0 のフレームと、行がぐちゃぐちゃに崩れたノイズ画像が
# 交互に来る)。フレーム数や解像度は正常に出るので、画素の中身を見ない限り気付けない。
# NVIDIA 以外では llvmpipe (ソフトウェアレンダリング) に切り替える。ogre v1 では正常に描画できる。
#
#   VRSO101_GL=hw   常に GPU を使う
#   VRSO101_GL=sw   常にソフトウェアレンダリング
#   (未設定)         GL レンダラ名に NVIDIA が含まれれば GPU、それ以外はソフトウェア
_mode="${VRSO101_GL:-auto}"
if [ "$_mode" = auto ]; then
  if command -v glxinfo >/dev/null 2>&1 && glxinfo -B 2>/dev/null | grep -q "renderer string.*NVIDIA"; then
    _mode=hw
  else
    _mode=sw
  fi
fi
if [ "$_mode" = sw ]; then
  export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
  echo "[gl] ソフトウェアレンダリング (llvmpipe)" >&2
else
  echo "[gl] GPU レンダリング" >&2
fi
unset _mode
