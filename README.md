# ROS Bag 資料提取工具

這個項目提供了一套從ROS bag文件中提取各種資料的工具集，包括相機/機器人軌跡、彩色圖像和深度圖。特別適合用於處理視覺SLAM或視覺里程計資料集。

## 功能特點

- **軌跡提取與視覺化**：從tf或tf2消息中提取位置軌跡，並生成2D和3D視覺化
- **單幀圖像提取**：從bag中提取指定幀號的彩色圖像和深度圖
- **序列提取**：按照指定頻率從整個bag中批量提取彩色圖像和深度圖序列
- **整合提取**：一鍵同時提取圖像序列和軌跡資料，並提供視覺化
- **文件分析**：查看bag文件的基本信息、話題列表和數據統計

## 安裝依賴

本工具需要以下依賴套件，您可以通過以下方式安裝：

### 快速安裝

```bash
pip install -r requirements.txt
```

### 依賴詳情

```
# 核心數據處理庫
numpy>=1.20.0
matplotlib>=3.5.0
opencv-python>=4.5.0
Pillow>=8.0.0

# ROS 相關庫
rosbag>=1.15.0
tf>=1.13.0
tf2_ros>=0.7.0

# 可選的數據處理庫
pandas>=1.3.0
scipy>=1.7.0

# 開發工具 (僅開發環境需要)
pytest>=6.0.0
pylint>=2.10.0
```

### 依賴說明

- **核心庫**：
  - numpy: 用於數值計算和矩陣操作
  - matplotlib: 用於軌跡的2D/3D視覺化
  - opencv-python: 處理RGB和深度圖像
  - Pillow: 增強的圖像處理功能

- **ROS庫**：
  - rosbag: 讀取和解析ROS bag文件
  - tf/tf2_ros: 處理座標變換和軌跡相關操作

## 使用方法

所有功能都整合到了一個統一的命令行界面中：

```bash
./bag_extractor.py <command> [options]
```

### 可用命令

- `info`: 分析ROS bag文件信息
- `trajectory`: 提取相機/機器人軌跡
- `visualize`: 視覺化已提取的軌跡
- `frame`: 提取單張彩色圖像和深度圖
- `sequence`: 按頻率批量提取圖像序列
- `extract-all`: 同時提取圖像序列和軌跡（推薦使用）

### 示例

**查看bag文件信息**：
```bash
./bag_extractor.py info data/storehouse02.bag
```

**提取並視覺化軌跡**：
```bash
./bag_extractor.py trajectory data/storehouse02.bag --visualize
```

**提取單張圖像和深度圖**：
```bash
./bag_extractor.py frame data/storehouse02.bag --frame 50
```

**以5Hz的頻率批量提取圖像序列**：
```bash
./bag_extractor.py sequence data/storehouse02.bag --frequency 5.0
```

**提取指定時間範圍內的圖像序列**：
```bash
./bag_extractor.py sequence data/storehouse02.bag --frequency 10.0 --start-time 5.0 --end-time 15.0
```

**同時提取圖像序列和軌跡（推薦）**：
```bash
./bag_extractor.py extract-all data/storehouse02.bag -f 5.0 -o output/dataset01 --visualize
```

**保存原始深度數據和彩色深度圖**：
```bash
./bag_extractor.py extract-all data/storehouse02.bag -o output/complete_data --save-depth-raw --save-depth-color
```

## 深度圖輸出選項

為了節省儲存空間，工具默認只輸出灰度可視化深度圖 (`.png`)，不儲存原始深度數據和彩色深度圖。您可以使用以下選項來修改這個行為：

- `--save-depth-raw`: 保存原始深度數據 (`.npy` 格式)
- `--save-depth-color`: 保存彩色深度圖 (彩虹色映射)
- `--save-depth-vis`: 預設啟用，使用 `--save-depth-vis` 選項將關閉灰度深度圖輸出

## 輸出文件

所有提取的數據默認保存在`output`目錄下：

- 軌跡數據：`output/trajectory/trajectory.txt`
- 軌跡視覺化：`output/trajectory/trajectory_2d.png` 和 `output/trajectory/trajectory_3d.png`
- 彩色圖像：`output/images/rgb_*.jpg`
- 深度圖：
  - 灰度可視化：`output/depth/depth_*.png` (默認啟用)
  - 彩色可視化：`output/depth/depth_*_color.png` (可選)
  - 原始數據：`output/depth/depth_*.npy` (可選)

## 常見使用場景

**資料集採集**：
```bash
./bag_extractor.py extract-all data/mybag.bag -f 5.0 -o output/dataset
```

**為視覺SLAM評估提取資料**：
```bash
./bag_extractor.py extract-all data/mybag.bag -f 10.0 -o output/slam_evaluation --visualize
```

**僅提取軌跡用於比較**：
```bash
./bag_extractor.py trajectory data/mybag.bag -o output/comparison/groundtruth.txt --visualize
```

## 項目結構

```
.
├── bag_extractor.py  # 主程序入口
├── data/             # 存放ROS bag文件
├── output/           # 提取的數據輸出目錄
│   ├── depth/        # 深度圖
│   ├── images/       # 彩色圖像
│   └── trajectory/   # 軌跡數據和視覺化
└── src/              # 源代碼
    ├── extract_bag_sequence.py  # 批量提取序列
    ├── extract_simple.py        # 提取單幀圖像和深度圖
    ├── depth/                   # 深度圖處理模塊
    ├── image/                   # 圖像處理模塊
    ├── trajectory/              # 軌跡處理模塊
    │   ├── extract_trajectory.py    # 軌跡提取
    │   └── visualize_trajectory.py  # 軌跡視覺化
    └── utils/                   # 通用工具
```

## 最佳實踐

1. **使用 `extract-all` 命令**：這是最推薦的方式，可以一次性提取所有需要的數據。

2. **調整提取頻率**：根據您的需求選擇合適的提取頻率 (`-f` 選項)。對於視覺SLAM測試，5-10Hz通常是合適的；對於視覺效果展示，1-2Hz可能就足夠了。

3. **節省存儲空間**：默認只保存必要的文件格式（RGB圖像和灰度深度圖）。只有在確實需要時才啟用原始深度數據 (`--save-depth-raw`) 或彩色深度圖 (`--save-depth-color`)。

4. **時間範圍提取**：使用 `--start-time` 和 `--end-time` 參數可以只提取bag文件中特定時間段的數據，對於處理長時間錄製的數據特別有用。

## 常見問題

- **找不到話題**：如果您的bag使用了不同的話題名稱，請使用 `--rgb-topic` 和 `--depth-topic` 參數指定正確的話題。
- **深度圖顯示問題**：如果提取的深度圖顯示異常，可能是因為您的相機使用了不同的深度編碼格式，請檢查源碼中的深度轉換部分。
- **記憶體問題**：處理大型bag文件時，可能會遇到記憶體不足的情況，建議使用時間範圍參數分段處理。