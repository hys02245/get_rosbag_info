#!/usr/bin/env python3

import rosbag
import cv2
import numpy as np
import os
import sys
import argparse
import time
from datetime import datetime

def extract_sequence_from_bag(bag_file, rgb_topic, depth_topic, output_dir, 
                            frequency=10.0, start_time=None, end_time=None, 
                            save_depth_raw=False, save_depth_vis=True, save_depth_color=False):
    """
    從ROS bag中按照指定頻率提取彩色圖像序列和深度圖序列
    
    Args:
        bag_file (str): ROS bag文件路徑
        rgb_topic (str): RGB圖像話題
        depth_topic (str): 深度圖像話題
        output_dir (str): 輸出目錄
        frequency (float): 提取頻率 (Hz)
        start_time (float): 提取的起始時間 (秒)，None表示從頭開始
        end_time (float): 提取的結束時間 (秒)，None表示到結尾
        save_depth_raw (bool): 是否保存原始深度數據 (.npy)，默認為False
        save_depth_vis (bool): 是否保存可視化深度圖 (.png)，默認為True
        save_depth_color (bool): 是否保存彩色深度圖 (.png)，默認為False
    
    Returns:
        tuple: (rgb_count, depth_count) 提取的彩色圖像數量和深度圖數量
    """
    # 確保輸出目錄存在
    rgb_output_dir = os.path.join(output_dir, 'images')
    depth_output_dir = os.path.join(output_dir, 'depth')
    os.makedirs(rgb_output_dir, exist_ok=True)
    os.makedirs(depth_output_dir, exist_ok=True)
    
    # 打開ROS bag文件
    print(f"打開ROS bag文件: {bag_file}")
    bag = rosbag.Bag(bag_file, 'r')
    
    # 獲取bag的開始和結束時間
    bag_start_time = None
    bag_end_time = None
    
    # 獲取話題信息
    topics = bag.get_type_and_topic_info().topics
    if rgb_topic not in topics:
        print(f"錯誤: 未找到RGB話題 '{rgb_topic}'")
        print(f"可用話題: {', '.join(topics.keys())}")
        bag.close()
        return 0, 0
    if depth_topic not in topics:
        print(f"錯誤: 未找到深度話題 '{depth_topic}'")
        print(f"可用話題: {', '.join(topics.keys())}")
        bag.close()
        return 0, 0
    
    rgb_msg_count = topics[rgb_topic].message_count
    depth_msg_count = topics[depth_topic].message_count
    print(f"找到 {rgb_msg_count} 條RGB消息, {depth_msg_count} 條深度消息")
    
    # 獲取bag的時間範圍
    for topic, msg, t in bag.read_messages():
        if bag_start_time is None or t.to_sec() < bag_start_time:
            bag_start_time = t.to_sec()
        if bag_end_time is None or t.to_sec() > bag_end_time:
            bag_end_time = t.to_sec()
    
    # 設置提取的時間範圍
    if start_time is None:
        start_time = bag_start_time
    else:
        start_time = bag_start_time + start_time
        
    if end_time is None:
        end_time = bag_end_time
    else:
        end_time = bag_start_time + end_time
    
    duration = end_time - start_time
    print(f"Bag時間範圍: {bag_start_time:.2f} - {bag_end_time:.2f} 秒 (總時長: {bag_end_time - bag_start_time:.2f} 秒)")
    print(f"提取時間範圍: {start_time:.2f} - {end_time:.2f} 秒 (提取時長: {duration:.2f} 秒)")
    
    # 計算時間間隔和預計提取的幀數
    interval = 1.0 / frequency
    estimated_frames = int(duration * frequency) + 1
    print(f"提取頻率: {frequency} Hz (時間間隔: {interval:.4f} 秒)")
    print(f"預計提取 {estimated_frames} 幀")
    
    # 創建要提取的時間戳列表
    target_timestamps = np.arange(start_time, end_time + 1e-6, interval)
    
    # 收集RGB圖像的時間戳和消息
    print("收集RGB圖像時間戳...")
    rgb_timestamps = []
    rgb_messages = {}
    
    for _, msg, t in bag.read_messages(topics=[rgb_topic]):
        t_sec = t.to_sec()
        if start_time <= t_sec <= end_time:
            rgb_timestamps.append(t_sec)
            rgb_messages[t_sec] = msg
    
    rgb_timestamps.sort()
    print(f"在時間範圍內找到 {len(rgb_timestamps)} 條RGB消息")
    
    # 收集深度圖的時間戳和消息
    print("收集深度圖時間戳...")
    depth_timestamps = []
    depth_messages = {}
    
    for _, msg, t in bag.read_messages(topics=[depth_topic]):
        t_sec = t.to_sec()
        if start_time <= t_sec <= end_time:
            depth_timestamps.append(t_sec)
            depth_messages[t_sec] = msg
    
    depth_timestamps.sort()
    print(f"在時間範圍內找到 {len(depth_timestamps)} 條深度消息")
    
    # 對每個目標時間戳，找到最接近的RGB和深度消息
    rgb_count = 0
    depth_count = 0
    processed_time = 0
    start_process_time = time.time()
    
    print(f"\n開始提取序列，共 {len(target_timestamps)} 個時間點...")
    
    for i, target_time in enumerate(target_timestamps):
        # 顯示進度
        if i % 10 == 0 or i == len(target_timestamps) - 1:
            progress = (i + 1) / len(target_timestamps) * 100
            elapsed_time = time.time() - start_process_time
            remaining_time = (elapsed_time / (i + 1)) * (len(target_timestamps) - i - 1) if i > 0 else 0
            print(f"進度: {progress:.1f}% ({i+1}/{len(target_timestamps)}), "
                  f"已用時間: {elapsed_time:.1f}秒, 剩餘時間: {remaining_time:.1f}秒")
        
        # 找最接近的RGB時間戳
        if rgb_timestamps:
            closest_rgb_time = min(rgb_timestamps, key=lambda x: abs(x - target_time))
            time_diff = abs(closest_rgb_time - target_time)
            
            # 如果時間差小於間隔的一半，則處理該幀
            if time_diff <= interval / 2:
                msg = rgb_messages[closest_rgb_time]
                frame_time = closest_rgb_time - bag_start_time
                
                try:
                    # 處理RGB圖像
                    height, width = msg.height, msg.width
                    encoding = msg.encoding
                    
                    img_data = None
                    if encoding in ["rgb8", "bgr8"]:
                        img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
                        if encoding == "rgb8":
                            img_data = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)
                    elif encoding == "bgra8":
                        img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
                        img_data = cv2.cvtColor(img_data, cv2.COLOR_BGRA2BGR)
                    elif encoding == "rgba8":
                        img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
                        img_data = cv2.cvtColor(img_data, cv2.COLOR_RGBA2BGR)
                    else:
                        print(f"跳過不支持的RGB編碼: {encoding}")
                        continue
                    
                    # 生成文件名 (使用時間戳，精確到毫秒)
                    timestamp_str = f"{frame_time:.3f}".replace('.', '_')
                    rgb_filename = os.path.join(rgb_output_dir, f"rgb_{timestamp_str}.jpg")
                    
                    # 保存圖像
                    cv2.imwrite(rgb_filename, img_data)
                    rgb_count += 1
                    
                except Exception as e:
                    print(f"處理RGB圖像時出錯: {e}")
        
        # 找最接近的深度時間戳
        if depth_timestamps:
            closest_depth_time = min(depth_timestamps, key=lambda x: abs(x - target_time))
            time_diff = abs(closest_depth_time - target_time)
            
            # 如果時間差小於間隔的一半，則處理該幀
            if time_diff <= interval / 2:
                msg = depth_messages[closest_depth_time]
                frame_time = closest_depth_time - bag_start_time
                
                try:
                    # 處理深度圖
                    height, width = msg.height, msg.width
                    encoding = msg.encoding
                    
                    if encoding == "32FC1":
                        depth_data = np.frombuffer(msg.data, dtype=np.float32).reshape(height, width)
                        
                        # 生成文件名 (使用時間戳，精確到毫秒)
                        timestamp_str = f"{frame_time:.3f}".replace('.', '_')
                        
                        # 保存原始深度數據
                        if save_depth_raw:
                            depth_raw_path = os.path.join(depth_output_dir, f"depth_{timestamp_str}.npy")
                            np.save(depth_raw_path, depth_data)
                        
                        # 創建可視化用的深度圖
                        depth_mask = np.isfinite(depth_data) & (depth_data > 0)
                        if np.any(depth_mask):
                            valid_min = np.min(depth_data[depth_mask])
                            valid_max = np.max(depth_data[depth_mask])
                            
                            # 標準化為8位灰度
                            depth_vis = np.zeros_like(depth_data)
                            if valid_max > valid_min:
                                depth_vis[depth_mask] = 255.0 * (depth_data[depth_mask] - valid_min) / (valid_max - valid_min)
                            
                            depth_vis = depth_vis.astype(np.uint8)
                            
                            # 保存灰度深度圖
                            if save_depth_vis:
                                depth_gray_path = os.path.join(depth_output_dir, f"depth_{timestamp_str}.png")
                                cv2.imwrite(depth_gray_path, depth_vis)
                            
                            # 創建彩色深度圖
                            if save_depth_color:
                                depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                                depth_color_path = os.path.join(depth_output_dir, f"depth_{timestamp_str}_color.png")
                                cv2.imwrite(depth_color_path, depth_color)
                        
                        depth_count += 1
                    else:
                        print(f"跳過不支持的深度編碼: {encoding}")
                        continue
                        
                except Exception as e:
                    print(f"處理深度圖時出錯: {e}")
    
    bag.close()
    
    print(f"\n提取完成! 總共提取了 {rgb_count} 張RGB圖像和 {depth_count} 張深度圖")
    print(f"RGB圖像保存在: {rgb_output_dir}")
    print(f"深度圖保存在: {depth_output_dir}")
    
    return rgb_count, depth_count

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='從ROS bag中按指定頻率提取圖像序列')
    parser.add_argument('bag_file', help='ROS bag文件路徑')
    parser.add_argument('--rgb-topic', default='/zed2i/zed_node/rgb/image_rect_color',
                        help='RGB圖像話題 (默認: /zed2i/zed_node/rgb/image_rect_color)')
    parser.add_argument('--depth-topic', default='/zed2i/zed_node/depth/depth_registered',
                        help='深度圖像話題 (默認: /zed2i/zed_node/depth/depth_registered)')
    parser.add_argument('--output', '-o', default='output',
                        help='輸出目錄 (默認: output)')
    parser.add_argument('--frequency', '-f', type=float, default=10.0,
                        help='提取頻率，單位Hz (默認: 10.0)')
    parser.add_argument('--start-time', type=float, default=None,
                        help='提取的起始時間 (秒)，默認從頭開始')
    parser.add_argument('--end-time', type=float, default=None,
                        help='提取的結束時間 (秒)，默認到結尾')
    parser.add_argument('--no-depth-raw', action='store_false', dest='save_depth_raw',
                        help='不保存原始深度數據 (.npy)')
    parser.add_argument('--no-depth-vis', action='store_false', dest='save_depth_vis',
                        help='不保存可視化深度圖 (.png)')
    parser.add_argument('--no-depth-color', action='store_false', dest='save_depth_color',
                        help='不保存彩色深度圖 (.png)')
    
    args = parser.parse_args()
    
    # 確保輸入文件路徑是絕對路徑
    bag_file = args.bag_file
    if not os.path.isabs(bag_file):
        # 如果是相對路徑，嘗試在 data 目錄中查找
        if os.path.exists(os.path.join('data', bag_file)):
            bag_file = os.path.join('data', bag_file)
        elif not os.path.exists(bag_file):
            print(f"錯誤: 找不到bag文件 {bag_file}")
            sys.exit(1)
    
    # 記錄開始時間
    start_time = time.time()
    
    # 提取圖像序列
    extract_sequence_from_bag(
        bag_file, 
        args.rgb_topic, 
        args.depth_topic, 
        args.output, 
        args.frequency, 
        args.start_time, 
        args.end_time,
        args.save_depth_raw,
        args.save_depth_vis,
        args.save_depth_color
    )
    
    # 計算總運行時間
    elapsed_time = time.time() - start_time
    print(f"總運行時間: {elapsed_time:.2f} 秒 ({elapsed_time/60:.2f} 分鐘)")