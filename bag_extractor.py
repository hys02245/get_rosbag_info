#!/usr/bin/env python3
"""
ROS Bag 數據提取工具集

這是一個用於從 ROS bag 文件中提取各種數據的命令行工具集。
支持提取：
- 相機/機器人軌跡
- 單張彩色圖像和深度圖
- 按照指定頻率提取整個序列的彩色圖像和深度圖
"""

import os
import sys
import argparse
import time

# 添加 src 目錄到模塊搜索路徑
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src'))

# 導入各個功能模塊
from trajectory.extract_trajectory import extract_trajectory
from trajectory.visualize_trajectory import TrajectoryVisualizer
from extract_simple import extract_from_bag
from extract_bag_sequence import extract_sequence_from_bag

def main():
    # 創建主解析器
    parser = argparse.ArgumentParser(
        description='ROS Bag 數據提取工具集',
        formatter_class=argparse.RawTextHelpFormatter
    )
    subparsers = parser.add_subparsers(dest='command', help='可用命令')
    
    # === 提取軌跡子命令 ===
    traj_parser = subparsers.add_parser('trajectory', help='提取相機/機器人軌跡')
    traj_parser.add_argument('bag_file', help='ROS bag文件路徑')
    traj_parser.add_argument('--output', '-o', default='output/trajectory/trajectory.txt', 
                           help='輸出軌跡文件路徑 (默認: output/trajectory/trajectory.txt)')
    traj_parser.add_argument('--frame-id', default='odom', help='父座標系ID (默認: odom)')
    traj_parser.add_argument('--child-frame-id', default='base_link', help='子座標系ID (默認: base_link)')
    traj_parser.add_argument('--rate', '-r', type=float, default=10.0, help='採樣率 Hz (默認: 10.0)')
    traj_parser.add_argument('--timestamp', '-t', action='store_true', help='包含時間戳')
    traj_parser.add_argument('--visualize', '-v', action='store_true', help='提取後視覺化軌跡')
    
    # === 視覺化軌跡子命令 ===
    vis_parser = subparsers.add_parser('visualize', help='視覺化軌跡數據')
    vis_parser.add_argument('traj_file', help='軌跡文件路徑')
    vis_parser.add_argument('--save3d', help='3D視覺化圖像保存路徑', default=None)
    vis_parser.add_argument('--save2d', help='2D視覺化圖像保存路徑', default=None)
    
    # === 提取單幀子命令 ===
    frame_parser = subparsers.add_parser('frame', help='提取單張彩色圖像和深度圖')
    frame_parser.add_argument('bag_file', help='ROS bag文件路徑')
    frame_parser.add_argument('--rgb-topic', default='/zed2i/zed_node/rgb/image_rect_color',
                            help='RGB圖像話題 (默認: /zed2i/zed_node/rgb/image_rect_color)')
    frame_parser.add_argument('--depth-topic', default='/zed2i/zed_node/depth/depth_registered',
                            help='深度圖像話題 (默認: /zed2i/zed_node/depth/depth_registered)')
    frame_parser.add_argument('--rgb-output', default='output/images/frame.jpg',
                            help='RGB圖像輸出路徑 (默認: output/images/frame.jpg)')
    frame_parser.add_argument('--depth-output', default='output/depth/depth',
                            help='深度圖基本輸出路徑，無需添加後綴 (默認: output/depth/depth)')
    frame_parser.add_argument('--frame', '-f', type=int, default=0,
                            help='要提取的幀號 (默認: 0)')
    
    # === 提取序列子命令 ===
    sequence_parser = subparsers.add_parser('sequence', help='按頻率提取圖像序列')
    sequence_parser.add_argument('bag_file', help='ROS bag文件路徑')
    sequence_parser.add_argument('--rgb-topic', default='/zed2i/zed_node/rgb/image_rect_color',
                               help='RGB圖像話題 (默認: /zed2i/zed_node/rgb/image_rect_color)')
    sequence_parser.add_argument('--depth-topic', default='/zed2i/zed_node/depth/depth_registered',
                               help='深度圖像話題 (默認: /zed2i/zed_node/depth/depth_registered)')
    sequence_parser.add_argument('--output', '-o', default='output',
                               help='輸出目錄 (默認: output)')
    sequence_parser.add_argument('--frequency', '-f', type=float, default=10.0,
                               help='提取頻率，單位Hz (默認: 10.0)')
    sequence_parser.add_argument('--start-time', type=float, default=None,
                               help='提取的起始時間 (秒)，默認從頭開始')
    sequence_parser.add_argument('--end-time', type=float, default=None,
                               help='提取的結束時間 (秒)，默認到結尾')
    sequence_parser.add_argument('--save-depth-raw', action='store_true',
                               help='保存原始深度數據 (.npy)')
    sequence_parser.add_argument('--save-depth-vis', action='store_false', dest='save_depth_vis',
                               help='不保存可視化深度圖 (.png)')
    sequence_parser.add_argument('--save-depth-color', action='store_true',
                               help='保存彩色深度圖 (.png)')
    
    # === 整合提取子命令 ===
    extract_all_parser = subparsers.add_parser('extract-all', help='同時提取圖像序列和軌跡')
    extract_all_parser.add_argument('bag_file', help='ROS bag文件路徑')
    extract_all_parser.add_argument('--output', '-o', default='output',
                                  help='輸出根目錄 (默認: output)')
    # 圖像相關參數
    extract_all_parser.add_argument('--rgb-topic', default='/zed2i/zed_node/rgb/image_rect_color',
                                  help='RGB圖像話題 (默認: /zed2i/zed_node/rgb/image_rect_color)')
    extract_all_parser.add_argument('--depth-topic', default='/zed2i/zed_node/depth/depth_registered',
                                  help='深度圖像話題 (默認: /zed2i/zed_node/depth/depth_registered)')
    extract_all_parser.add_argument('--frequency', '-f', type=float, default=10.0,
                                  help='提取頻率，單位Hz (默認: 10.0)')
    extract_all_parser.add_argument('--save-depth-raw', action='store_true',
                                  help='保存原始深度數據 (.npy)')
    extract_all_parser.add_argument('--save-depth-color', action='store_true',
                                  help='保存彩色深度圖 (.png)')
    # 軌跡相關參數
    extract_all_parser.add_argument('--frame-id', default='odom',
                                  help='父座標系ID (默認: odom)')
    extract_all_parser.add_argument('--child-frame-id', default='base_link',
                                  help='子座標系ID (默認: base_link)')
    extract_all_parser.add_argument('--visualize', '-v', action='store_true',
                                  help='生成軌跡視覺化')
    extract_all_parser.add_argument('--start-time', type=float, default=None,
                                  help='提取的起始時間 (秒)，默認從頭開始')
    extract_all_parser.add_argument('--end-time', type=float, default=None,
                                  help='提取的結束時間 (秒)，默認到結尾')
    
    # === 分析子命令 ===
    info_parser = subparsers.add_parser('info', help='分析ROS bag文件信息')
    info_parser.add_argument('bag_file', help='ROS bag文件路徑')
    
    # 解析參數
    args = parser.parse_args()
    
    # 如果沒有指定命令，顯示幫助信息
    if args.command is None:
        parser.print_help()
        return
    
    # 確保輸入文件路徑是絕對路徑
    if hasattr(args, 'bag_file'):
        bag_file = args.bag_file
        if not os.path.isabs(bag_file):
            # 如果是相對路徑，嘗試在 data 目錄中查找
            if os.path.exists(os.path.join('data', bag_file)):
                args.bag_file = os.path.join('data', bag_file)
            elif not os.path.exists(bag_file):
                print(f"錯誤: 找不到bag文件 {bag_file}")
                sys.exit(1)
    
    # 根據命令執行相應的功能
    if args.command == 'trajectory':
        # 提取軌跡
        output_file = extract_trajectory(
            args.bag_file,
            args.output,
            args.frame_id,
            args.child_frame_id,
            args.rate,
            args.timestamp
        )
        
        # 如果指定了視覺化，則視覺化軌跡
        if args.visualize and output_file:
            # 獲取軌跡文件所在目錄
            traj_dir = os.path.dirname(output_file)
            # 生成視覺化文件路徑
            vis3d_path = os.path.join(traj_dir, 'trajectory_3d.png')
            vis2d_path = os.path.join(traj_dir, 'trajectory_2d.png')
            
            visualizer = TrajectoryVisualizer(output_file)
            visualizer.visualize_3d(vis3d_path)
            visualizer.visualize_2d(vis2d_path)
            print(f"已生成軌跡視覺化圖像：")
            print(f"  - 3D視圖：{vis3d_path}")
            print(f"  - 2D視圖：{vis2d_path}")
    
    elif args.command == 'visualize':
        # 視覺化軌跡
        visualizer = TrajectoryVisualizer(args.traj_file)
        visualizer.visualize_3d(args.save3d)
        visualizer.visualize_2d(args.save2d)
    
    elif args.command == 'frame':
        # 提取單幀圖像和深度圖
        extract_from_bag(
            args.bag_file,
            args.rgb_topic,
            args.depth_topic,
            args.rgb_output,
            args.depth_output,
            args.frame
        )
    
    elif args.command == 'sequence':
        # 記錄開始時間
        start_time = time.time()
        
        # 按頻率提取圖像序列
        extract_sequence_from_bag(
            args.bag_file, 
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
    
    elif args.command == 'extract-all':
        # 同時提取圖像序列和軌跡
        output_file = extract_trajectory(
            args.bag_file,
            os.path.join(args.output, 'trajectory', 'trajectory.txt'),
            args.frame_id,
            args.child_frame_id,
            args.frequency,  # 使用頻率作為採樣率
            False  # 不需要時間戳
        )
        
        # 提取圖像序列
        extract_sequence_from_bag(
            args.bag_file, 
            args.rgb_topic, 
            args.depth_topic, 
            args.output, 
            args.frequency, 
            args.start_time, 
            args.end_time,
            args.save_depth_raw,
            True,  # 保存可視化深度圖
            args.save_depth_color
        )
        
        # 如果指定了視覺化，則視覺化軌跡
        if args.visualize and output_file:
            # 獲取軌跡文件所在目錄
            traj_dir = os.path.dirname(output_file)
            # 確保輸出目錄存在
            os.makedirs(traj_dir, exist_ok=True)
            
            # 生成視覺化文件路徑
            vis3d_path = os.path.join(traj_dir, 'trajectory_3d.png')
            vis2d_path = os.path.join(traj_dir, 'trajectory_2d.png')
            
            # 生成視覺化
            visualizer = TrajectoryVisualizer(output_file)
            visualizer.visualize_3d(vis3d_path)
            visualizer.visualize_2d(vis2d_path)
            print(f"已生成軌跡視覺化圖像：")
            print(f"  - 3D視圖：{vis3d_path}")
            print(f"  - 2D視圖：{vis2d_path}")
    
    elif args.command == 'info':
        # 分析ROS bag文件信息
        import rosbag
        
        print(f"分析ROS bag文件: {args.bag_file}")
        bag = rosbag.Bag(args.bag_file, 'r')
        
        # 獲取基本信息
        info = bag.get_type_and_topic_info()
        topics = info.topics
        
        # 顯示bag的開始和結束時間
        bag_start_time = None
        bag_end_time = None
        
        for topic, msg, t in bag.read_messages():
            if bag_start_time is None or t.to_sec() < bag_start_time:
                bag_start_time = t.to_sec()
            if bag_end_time is None or t.to_sec() > bag_end_time:
                bag_end_time = t.to_sec()
        
        print(f"\nBag時間範圍: {bag_start_time:.2f} - {bag_end_time:.2f} 秒")
        print(f"總時長: {bag_end_time - bag_start_time:.2f} 秒")
        
        # 顯示話題信息
        print(f"\n包含 {len(topics)} 個話題:")
        
        for topic_name, topic_info in topics.items():
            print(f"\n話題: {topic_name}")
            print(f"  - 類型: {topic_info.msg_type}")
            print(f"  - 消息數: {topic_info.message_count}")
            print(f"  - 頻率: {topic_info.frequency:.2f} Hz")
        
        bag.close()

if __name__ == "__main__":
    main()