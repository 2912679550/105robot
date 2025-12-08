#!/bin/bash
cd ..

# 获取当前时间，格式为：年月日小时分钟
timestamp=$(date +"%Y%m%d%H%M")

# 数据包保存路径
bag_file="locate_data_$timestamp.bag"

# 打印提示信息
echo "开始录制ROS话题，保存为数据包：$bag_file"

# 录制指定话题
rosbag record -O $bag_file \
    /camera/imu \
    /camera/infra1/image_rect_raw \
    /camera/infra2/image_rect_raw \
    /vins_node/odometry

# 提示录制完成
echo "录制完成，数据包已保存为：$bag_file"
