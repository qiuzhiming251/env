#!/bin/bash

# 设置默认路径
rm ../png/*
DEFAULT_PATH="../data/datadump/"

# 使用传入的第一个参数或默认路径
INPUT_DIR="${1:-$DEFAULT_PATH}"

# 校验输入目录是否存在
if [ ! -d "$INPUT_DIR" ]; then
  echo "错误: 输入路径不存在: $INPUT_DIR"
  exit 1
fi

# 定义输出目录并创建
OUTPUT_DIR="../png"
mkdir -p "$OUTPUT_DIR"
rm "$OUTPUT_DIR"/*.png

# 遍历目录中的每个 .ppm 文件并进行转换
for ppm_file in "$INPUT_DIR"/*.ppm; do
  # 检查是否存在 .ppm 文件
  if [ ! -f "$ppm_file" ]; then
    echo "警告: 没有找到任何 .ppm 文件在目录: $INPUT_DIR"
    exit 1
  fi
  
  # 获取文件名（没有扩展名）
  base_name=$(basename "$ppm_file" .ppm)
  
  # 定义输出的 .png 文件名
  png_file="$OUTPUT_DIR/$base_name.png"
  
  # 使用 pnmtopng 进行转换
  pnmtopng "$ppm_file" > "$png_file"
  
  echo "转换完成: $ppm_file -> $png_file"
done



####转换成tiff
 #设置默认输入路径
DEFAULT_PATH="../png"

# 使用传入的第一个参数或默认路径
INPUT_DIR="${1:-$DEFAULT_PATH}"

# 校验输入目录是否存在
if [ ! -d "$INPUT_DIR" ]; then
  echo "错误: 输入路径不存在: $INPUT_DIR"
  exit 1
fi

# 定义输出的 .tiff 文件名
OUTPUT_DIR="../mp4"
mkdir -p "$OUTPUT_DIR"
CURRENT_TIME=$(date +"%Y%m%d_%H%M%S")
OUTPUT_FILE="output_${CURRENT_TIME}.mp4"
OUTPUT_FILE_GIF="output_${CURRENT_TIME}.gif"
# 检查目录中是否有 .png 文件
png_files=("$INPUT_DIR"/*.png)
if [ ${#png_files[@]} -eq 0 ]; then
  echo "警告: 没有找到任何 .png 文件在目录: $INPUT_DIR"
  exit 1
fi

# 使用 ImageMagick 的 convert 命令合并 .png 文件到一个 .tiff 文件
echo "${png_files[@]}"
# convert   -delay 5 -loop 0 "${png_files[@]}" "$OUTPUT_DIR/$OUTPUT_FILE"
ffmpeg -framerate 5 -i $INPUT_DIR/%04d.png -c:v libx264 -pix_fmt yuv420p "$OUTPUT_DIR/$OUTPUT_FILE"
echo "转换完成: 所有 .png 文件已合并为 $OUTPUT_FILE"
rm -rf ../data/datadump/*
