#!/usr/bin/env python3

# 在導入 matplotlib.pyplot 之前設置後端為 'Agg'，避免 Qt 相關問題
import matplotlib
matplotlib.use('Agg')  # 使用非交互式的 'Agg' 後端

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os
import sys

# 添加項目根目錄到系統路徑，以便導入其他模塊
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

class TrajectoryVisualizer:
    """
    Modular trajectory visualization class for displaying trajectory data extracted from ROS bag
    """
    def __init__(self, traj_file):
        """Initialize visualizer and load trajectory data"""
        self.traj_file = traj_file
        self.trajectory_data = None
        self.timestamps = None
        self.has_timestamps = False
        
        # Load trajectory data
        self._load_trajectory()
    
    def _load_trajectory(self):
        """Load trajectory data from file"""
        print(f"Loading trajectory file: {self.traj_file}")
        try:
            # Read trajectory data
            data = np.loadtxt(self.traj_file)
            
            # Check if each row contains timestamp
            rows, cols = data.shape
            if cols > 16:  # If columns > 16, assume first column is timestamp
                self.has_timestamps = True
                self.timestamps = data[:, 0]
                # Extract 4x4 transformation matrix from each row
                self.trajectory_data = [data[i, 1:].reshape(4, 4) for i in range(rows)]
            else:
                # No timestamps, extract 4x4 transformation matrix directly
                self.trajectory_data = [data[i, :].reshape(4, 4) for i in range(rows)]
            
            print(f"Successfully loaded {len(self.trajectory_data)} trajectory points")
        except Exception as e:
            print(f"Error loading trajectory file: {e}")
            exit(1)
    
    def visualize_3d(self, save_path=None):
        """Visualize trajectory as 3D path"""
        if not self.trajectory_data:
            print("No trajectory data to visualize")
            return
        
        # Extract position information from trajectory (first three rows of fourth column of each matrix)
        positions = np.array([matrix[:3, 3] for matrix in self.trajectory_data])
        x = positions[:, 0]
        y = positions[:, 1]
        z = positions[:, 2]
        
        # Create 3D figure
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot trajectory path
        ax.plot(x, y, z, 'b-', linewidth=1, label='Trajectory Path')
        ax.scatter(x[0], y[0], z[0], color='green', s=100, label='Start Point')
        ax.scatter(x[-1], y[-1], z[-1], color='red', s=100, label='End Point')
        
        # Plot coordinate axes
        ax_length = max(max(x) - min(x), max(y) - min(y), max(z) - min(z)) * 0.1
        origin = positions[0]
        
        # Draw a pose frame every few points (visualize rotation)
        step = max(1, len(self.trajectory_data) // 20)  # Show at most 20 pose frames
        for i in range(0, len(self.trajectory_data), step):
            matrix = self.trajectory_data[i]
            pos = matrix[:3, 3]
            x_axis = pos + matrix[:3, 0] * ax_length
            y_axis = pos + matrix[:3, 1] * ax_length
            z_axis = pos + matrix[:3, 2] * ax_length
            
            ax.plot([pos[0], x_axis[0]], [pos[1], x_axis[1]], [pos[2], x_axis[2]], 'r-', linewidth=1)
            ax.plot([pos[0], y_axis[0]], [pos[1], y_axis[1]], [pos[2], y_axis[2]], 'g-', linewidth=1)
            ax.plot([pos[0], z_axis[0]], [pos[1], z_axis[1]], [pos[2], z_axis[2]], 'b-', linewidth=1)
        
        # Set figure properties
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Trajectory Visualization')
        ax.legend()
        
        # Set view angle
        ax.view_init(elev=30, azim=45)
        
        # Save or display the figure
        if save_path:
            # 確保輸出目錄存在
            output_dir = os.path.dirname(save_path)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)
                
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Saved 3D trajectory plot to {save_path}")
        else:
            plt.show()
    
    def visualize_2d(self, save_path=None):
        """Visualize trajectory as 2D plane views (top view and side views)"""
        if not self.trajectory_data:
            print("No trajectory data to visualize")
            return
        
        # Extract position information
        positions = np.array([matrix[:3, 3] for matrix in self.trajectory_data])
        x = positions[:, 0]
        y = positions[:, 1]
        z = positions[:, 2]
        
        # Create 2D figure (2x2 grid)
        fig, axs = plt.subplots(2, 2, figsize=(12, 10))
        
        # Plot XY plane (top view)
        axs[0, 0].plot(x, y, 'b-', linewidth=2)
        axs[0, 0].scatter(x[0], y[0], color='green', s=100, label='Start')
        axs[0, 0].scatter(x[-1], y[-1], color='red', s=100, label='End')
        axs[0, 0].set_xlabel('X')
        axs[0, 0].set_ylabel('Y')
        axs[0, 0].set_title('XY Plane (Top View)')
        axs[0, 0].grid(True)
        axs[0, 0].axis('equal')
        axs[0, 0].legend()
        
        # Plot XZ plane
        axs[0, 1].plot(x, z, 'r-', linewidth=2)
        axs[0, 1].scatter(x[0], z[0], color='green', s=100, label='Start')
        axs[0, 1].scatter(x[-1], z[-1], color='red', s=100, label='End')
        axs[0, 1].set_xlabel('X')
        axs[0, 1].set_ylabel('Z')
        axs[0, 1].set_title('XZ Plane (Side View)')
        axs[0, 1].grid(True)
        axs[0, 1].axis('equal')
        
        # Plot YZ plane
        axs[1, 0].plot(y, z, 'g-', linewidth=2)
        axs[1, 0].scatter(y[0], z[0], color='green', s=100, label='Start')
        axs[1, 0].scatter(y[-1], z[-1], color='red', s=100, label='End')
        axs[1, 0].set_xlabel('Y')
        axs[1, 0].set_ylabel('Z')
        axs[1, 0].set_title('YZ Plane (Side View)')
        axs[1, 0].grid(True)
        axs[1, 0].axis('equal')
        
        # If timestamps available, plot height vs time
        if self.has_timestamps:
            axs[1, 1].plot(self.timestamps, z, 'm-', linewidth=2)
            axs[1, 1].set_xlabel('Time (seconds)')
            axs[1, 1].set_ylabel('Z (Height)')
            axs[1, 1].set_title('Height vs Time')
            axs[1, 1].grid(True)
        else:
            # If no timestamps, plot trajectory length
            path_length = np.cumsum(np.sqrt(np.sum(np.diff(positions, axis=0)**2, axis=1)))
            path_length = np.insert(path_length, 0, 0)  # Insert 0 at the beginning
            axs[1, 1].plot(path_length, z, 'm-', linewidth=2)
            axs[1, 1].set_xlabel('Path Length')
            axs[1, 1].set_ylabel('Z (Height)')
            axs[1, 1].set_title('Height vs Path Length')
            axs[1, 1].grid(True)
        
        plt.tight_layout()
        
        # Save or display the figure
        if save_path:
            # 確保輸出目錄存在
            output_dir = os.path.dirname(save_path)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)
                
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Saved 2D trajectory plots to {save_path}")
        else:
            plt.show()

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Visualize trajectory file')
    parser.add_argument('traj_file', help='Path to trajectory file')
    parser.add_argument('--save3d', help='Path to save 3D visualization image', default='output/images/trajectory_3d.png')
    parser.add_argument('--save2d', help='Path to save 2D visualization image', default='output/images/trajectory_2d.png')
    parser.add_argument('--backend', default='Agg', help='Matplotlib backend (default: Agg)')
    parser.add_argument('--show', action='store_true', help='Show figures (requires a suitable backend)')
    
    args = parser.parse_args()
    
    # 設置 Matplotlib 後端
    if args.show and args.backend != 'Agg':
        matplotlib.use(args.backend)
    
    # 如果沒有指定保存路徑，且不顯示圖形，則使用默認路徑
    if not args.save3d and not args.show:
        args.save3d = 'output/images/trajectory_3d.png'
    if not args.save2d and not args.show:
        args.save2d = 'output/images/trajectory_2d.png'
    
    # 創建視覺化器
    visualizer = TrajectoryVisualizer(args.traj_file)
    
    # 視覺化軌跡
    visualizer.visualize_3d(None if args.show and not args.save3d else args.save3d)
    visualizer.visualize_2d(None if args.show and not args.save2d else args.save2d)
    
    if args.show:
        plt.show()

if __name__ == "__main__":
    main()