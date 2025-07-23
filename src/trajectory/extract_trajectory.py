#!/usr/bin/env python3

import rosbag
import numpy as np
import tf
import argparse
from tf.transformations import euler_from_quaternion, quaternion_matrix
import os
import sys

# 添加項目根目錄到系統路徑，以便導入其他模塊
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

def extract_trajectory(bag_file, output_file='traj.txt', frame_id='odom', child_frame_id='base_link', 
                      sampling_rate=10.0, include_timestamp=False):
    """
    Extract trajectory data from a ROS bag file and save it to a text file.
    
    Args:
        bag_file (str): Path to the ROS bag file
        output_file (str): Path to the output text file
        frame_id (str): Parent frame ID
        child_frame_id (str): Child frame ID
        sampling_rate (float): Desired sampling rate in Hz
        include_timestamp (bool): Whether to include timestamps in the output
    """
    print(f"Extracting trajectory from {bag_file}")
    print(f"  - Target frames: {frame_id} -> {child_frame_id}")
    print(f"  - Sampling rate: {sampling_rate} Hz")
    print(f"  - Include timestamps: {'Yes' if include_timestamp else 'No'}")
    
    # 確保輸出目錄存在
    output_dir = os.path.dirname(output_file)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    
    # 打開 ROS bag 文件
    bag = rosbag.Bag(bag_file, 'r')
    
    # 獲取 bag 的持續時間和開始時間
    start_time = None
    end_time = None
    
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        for transform in msg.transforms:
            if transform.header.frame_id == frame_id and transform.child_frame_id == child_frame_id:
                if start_time is None or t.to_sec() < start_time:
                    start_time = t.to_sec()
                if end_time is None or t.to_sec() > end_time:
                    end_time = t.to_sec()
    
    if start_time is None or end_time is None:
        print(f"Error: Could not find transforms between {frame_id} and {child_frame_id}")
        return
    
    duration = end_time - start_time
    print(f"Bag duration: {duration:.2f} seconds")
    
    # 根據取樣率計算時間間隔
    time_interval = 1.0 / sampling_rate
    
    # 創建期望的時間戳列表
    desired_timestamps = np.arange(start_time, end_time, time_interval)
    
    # 用於存儲每個時間戳的變換的字典
    transforms = {}
    
    # 讀取所有變換
    print("Reading transforms...")
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        for transform in msg.transforms:
            if transform.header.frame_id == frame_id and transform.child_frame_id == child_frame_id:
                transforms[t.to_sec()] = transform
    
    bag.close()
    
    if not transforms:
        print(f"Error: No transforms found between {frame_id} and {child_frame_id}")
        return
    
    print(f"Found {len(transforms)} transforms")
    
    # 為每個期望的時間戳獲取最接近的變換
    trajectory = []
    times = sorted(transforms.keys())
    
    print("Processing trajectory data...")
    for timestamp in desired_timestamps:
        # 找到最接近的時間
        closest_time = min(times, key=lambda x: abs(x - timestamp))
        
        # 獲取變換
        transform = transforms[closest_time]
        
        # 提取位置和方向
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        
        # 將四元數轉換為 4x4 變換矩陣
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        matrix = quaternion_matrix(quaternion)
        
        # 將平移向量插入矩陣
        matrix[0, 3] = translation.x
        matrix[1, 3] = translation.y
        matrix[2, 3] = translation.z
        
        # 存儲軌跡數據（作為展平的 4x4 矩陣）
        if include_timestamp:
            entry = [timestamp - start_time]
            entry.extend(matrix.flatten())
            trajectory.append(entry)
        else:
            trajectory.append(matrix.flatten())
    
    # 將軌跡寫入文件
    print(f"Writing trajectory to {output_file}")
    with open(output_file, 'w') as f:
        for matrix in trajectory:
            if include_timestamp:
                time_str = f"{matrix[0]:.6f} "
                matrix_str = ' '.join([f"{val:.15e}" for val in matrix[1:]])
                f.write(time_str + matrix_str + '\n')
            else:
                f.write(' '.join([f"{val:.15e}" for val in matrix]) + '\n')
    
    print(f"Successfully extracted {len(trajectory)} trajectory points")
    return output_file

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract trajectory from ROS bag file')
    parser.add_argument('bag_file', help='Path to the ROS bag file')
    parser.add_argument('--output', '-o', default='output/trajectory/traj.txt', help='Output file path (default: output/trajectory/traj.txt)')
    parser.add_argument('--frame-id', default='odom', help='Parent frame ID (default: odom)')
    parser.add_argument('--child-frame-id', default='base_link', help='Child frame ID (default: base_link)')
    parser.add_argument('--rate', '-r', type=float, default=10.0, help='Sampling rate in Hz (default: 10.0)')
    parser.add_argument('--timestamp', '-t', action='store_true', help='Include timestamp in the output')
    
    args = parser.parse_args()
    
    # 確保輸入文件路徑是絕對路徑
    bag_file = args.bag_file
    if not os.path.isabs(bag_file):
        # 如果是相對路徑，嘗試在 data 目錄中查找
        if os.path.exists(os.path.join('data', bag_file)):
            bag_file = os.path.join('data', bag_file)
        elif not os.path.exists(bag_file):
            print(f"Error: Could not find bag file {bag_file}")
            sys.exit(1)
    
    extract_trajectory(bag_file, args.output, args.frame_id, args.child_frame_id, args.rate, args.timestamp)