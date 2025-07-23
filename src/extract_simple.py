#!/usr/bin/env python3

import rosbag
import cv2
import numpy as np
import os
import sys
import argparse

def extract_from_bag(bag_file, rgb_topic, depth_topic, rgb_output, depth_output, frame_number=0):
    """
    從ROS bag中提取單張彩色圖像和深度圖
    
    Args:
        bag_file (str): ROS bag文件路徑
        rgb_topic (str): RGB圖像話題
        depth_topic (str): 深度圖像話題
        rgb_output (str): RGB圖像輸出路徑
        depth_output (str): 深度圖輸出路徑
        frame_number (int): 要提取的幀號
    """
    # 確保輸出目錄存在
    for path in [rgb_output, depth_output]:
        output_dir = os.path.dirname(path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)
    
    # 打開ROS bag文件
    print(f"打開ROS bag文件: {bag_file}")
    bag = rosbag.Bag(bag_file, 'r')
    
    # 獲取消息數量
    topics = bag.get_type_and_topic_info().topics
    if rgb_topic not in topics:
        print(f"錯誤: 未找到RGB話題 '{rgb_topic}'")
        print(f"可用話題: {', '.join(topics.keys())}")
        return False
    if depth_topic not in topics:
        print(f"錯誤: 未找到深度話題 '{depth_topic}'")
        print(f"可用話題: {', '.join(topics.keys())}")
        return False
    
    rgb_msg_count = topics[rgb_topic].message_count
    depth_msg_count = topics[depth_topic].message_count
    print(f"找到 {rgb_msg_count} 條RGB消息, {depth_msg_count} 條深度消息")
    
    # 檢查幀號是否有效
    if frame_number < 0 or frame_number >= min(rgb_msg_count, depth_msg_count):
        print(f"錯誤: 幀號 {frame_number} 超出範圍 (0-{min(rgb_msg_count, depth_msg_count)-1})")
        return False
    
    # 提取RGB圖像
    print(f"提取RGB幀 {frame_number}...")
    rgb_frame = None
    rgb_count = 0
    for _, msg, _ in bag.read_messages(topics=[rgb_topic]):
        if rgb_count == frame_number:
            try:
                # 直接從消息數據中提取圖像
                height, width = msg.height, msg.width
                encoding = msg.encoding
                print(f"RGB圖像編碼: {encoding}, 尺寸: {width}x{height}")
                
                if encoding in ["rgb8", "bgr8"]:
                    img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
                    if encoding == "rgb8":
                        img_data = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)
                    rgb_frame = img_data
                elif encoding == "bgra8":
                    img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
                    # 從BGRA轉換為BGR
                    rgb_frame = cv2.cvtColor(img_data, cv2.COLOR_BGRA2BGR)
                elif encoding == "rgba8":
                    img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
                    # 從RGBA轉換為BGR
                    rgb_frame = cv2.cvtColor(img_data, cv2.COLOR_RGBA2BGR)
                else:
                    print(f"不支持的RGB編碼: {encoding}")
                    break
                    
                # 保存圖像
                cv2.imwrite(rgb_output, rgb_frame)
                print(f"已保存RGB圖像到 {rgb_output}")
                break
            except Exception as e:
                print(f"處理RGB圖像時出錯: {e}")
                import traceback
                traceback.print_exc()
                break
        
        rgb_count += 1
    
    # 提取深度圖
    print(f"提取深度幀 {frame_number}...")
    depth_frame = None
    depth_count = 0
    for _, msg, _ in bag.read_messages(topics=[depth_topic]):
        if depth_count == frame_number:
            try:
                # 直接從消息數據中提取深度數據
                height, width = msg.height, msg.width
                encoding = msg.encoding
                print(f"深度圖編碼: {encoding}, 尺寸: {width}x{height}")
                
                if encoding == "32FC1":
                    depth_data = np.frombuffer(msg.data, dtype=np.float32).reshape(height, width)
                    depth_frame = depth_data
                    
                    # 保存原始深度數據
                    depth_raw_path = depth_output + ".npy"
                    np.save(depth_raw_path, depth_data)
                    print(f"已保存原始深度數據到 {depth_raw_path}")
                    
                    # 創建可視化用的深度圖
                    depth_mask = np.isfinite(depth_data) & (depth_data > 0)
                    if np.any(depth_mask):
                        valid_min = np.min(depth_data[depth_mask])
                        valid_max = np.max(depth_data[depth_mask])
                        print(f"深度範圍: {valid_min:.3f} 到 {valid_max:.3f} 米")
                        
                        # 標準化為8位灰度
                        depth_vis = np.zeros_like(depth_data)
                        if valid_max > valid_min:
                            depth_vis[depth_mask] = 255.0 * (depth_data[depth_mask] - valid_min) / (valid_max - valid_min)
                        
                        depth_vis = depth_vis.astype(np.uint8)
                        
                        # 保存灰度深度圖
                        depth_gray_path = depth_output + ".png"
                        cv2.imwrite(depth_gray_path, depth_vis)
                        print(f"已保存灰度深度圖到 {depth_gray_path}")
                        
                        # 創建彩色深度圖
                        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                        depth_color_path = depth_output + "_color.png"
                        cv2.imwrite(depth_color_path, depth_color)
                        print(f"已保存彩色深度圖到 {depth_color_path}")
                    else:
                        print("警告: 未找到有效的深度值")
                else:
                    print(f"不支持的深度編碼: {encoding}")
                    break
            except Exception as e:
                print(f"處理深度圖像時出錯: {e}")
                import traceback
                traceback.print_exc()
                break
        
        depth_count += 1
    
    bag.close()
    
    success = True
    
    if rgb_frame is None:
        print(f"錯誤: 未能提取RGB幀 {frame_number}")
        success = False
    
    if depth_frame is None:
        print(f"錯誤: 未能提取深度幀 {frame_number}")
        success = False
    
    return success

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='從ROS bag中提取單張RGB和深度圖像')
    parser.add_argument('bag_file', help='ROS bag文件路徑')
    parser.add_argument('--rgb-topic', default='/zed2i/zed_node/rgb/image_rect_color',
                        help='RGB圖像話題 (默認: /zed2i/zed_node/rgb/image_rect_color)')
    parser.add_argument('--depth-topic', default='/zed2i/zed_node/depth/depth_registered',
                        help='深度圖像話題 (默認: /zed2i/zed_node/depth/depth_registered)')
    parser.add_argument('--rgb-output', default='output/images/frame.jpg',
                        help='RGB圖像輸出路徑 (默認: output/images/frame.jpg)')
    parser.add_argument('--depth-output', default='output/depth/depth',
                        help='深度圖基本輸出路徑，無需添加後綴 (默認: output/depth/depth)')
    parser.add_argument('--frame', '-f', type=int, default=0,
                        help='要提取的幀號 (默認: 0)')
    
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
    
    # 提取圖像和深度數據
    if not extract_from_bag(bag_file, args.rgb_topic, args.depth_topic, 
                           args.rgb_output, args.depth_output, args.frame):
        print("提取數據失敗")
        sys.exit(1)
    
    print("提取數據成功")