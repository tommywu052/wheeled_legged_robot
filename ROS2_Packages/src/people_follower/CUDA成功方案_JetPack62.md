# ✅ CUDA成功啟用方案 - Jetson Orin NX JetPack 6.2

## 🎯 問題回顧

```bash
# 症狀
python3 -c "import torch; print(torch.cuda.is_available())"
# ❌ 輸出: False

# 錯誤信息
ValueError: libcublas.so.*[0-9] not found in the system path
OSError: libnvToolsExt.so.1: cannot open shared object file
```

## ✅ 成功解決方案

### 關鍵發現
**PyTorch在Jetson上需要 `libcusparselt` 庫才能正常使用CUDA！**

### 完整步驟

```bash
# 1. 下載cuSPARSELt for Tegra (JetPack 6.x)
wget https://developer.download.nvidia.com/compute/cusparselt/0.7.1/local_installers/cusparselt-local-tegra-repo-ubuntu2204-0.7.1_1.0-1_arm64.deb

# 2. 安裝repository
sudo dpkg -i cusparselt-local-tegra-repo-ubuntu2204-0.7.1_1.0-1_arm64.deb

# 3. 添加GPG密鑰
sudo cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.7.1/cusparselt-*-keyring.gpg /usr/share/keyrings/

# 4. 更新apt緩存
sudo apt-get update

# 5. 安裝libcusparselt
sudo apt-get install -y libcusparselt0 libcusparselt-dev

# 6. 確保NumPy版本正確
pip3 install "numpy<2"

# 7. 驗證CUDA
python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"
```

### ✅ 成功輸出

```
PyTorch: 2.5.0a0+872d972e41.nv24.08
CUDA: True  ✅
```

## 📊 性能提升預期

| 模式 | FPS | 推理時間 | 性能提升 |
|------|-----|---------|---------|
| CPU模式 | 0.7 FPS | ~466ms | 基準 |
| GPU模式 | 15-30 FPS | 30-60ms | **20-40倍** |

## 🧪 測試YOLOv8 GPU性能

### 1. 測試檢測腳本

```bash
cd ~/legged_robot/ROS2_Packages/src/people_follower
python3 test_detection_yolo.py
```

**期望輸出：**
```
✅ 模型已載入
裝置: cuda:0  ✅ GPU模式
⚠️ CUDA不可用，使用CPU（會較慢）  ❌ 這行不應該出現

[INFO] ✅ 偵測到 1 人 | FPS: 25.3 | 推理: 39.5ms  ✅ 快很多！
```

### 2. 測試完整people_follower節點

```bash
# 確保RealSense正在運行
ros2 launch realsense2_camera rs_launch.py config_file:=/home/robotester1/legged_robot/realsense/realsense_params.yaml

# 新終端: 啟動people follower (YOLO版本)
cd ~/legged_robot/ROS2_Packages/src/people_follower
source /opt/ros/humble/setup.bash
source ~/legged_robot/ROS2_Packages/src/install/setup.bash

ros2 launch people_follower people_follower_yolo.launch.py
```

## 🔧 系統配置記錄

### 硬體
- **平台**: Jetson Orin NX 16GB
- **JetPack**: 6.2 (R36 REVISION: 4.7)
- **CUDA**: 12.6

### 軟體版本
```bash
# PyTorch
PyTorch: 2.5.0a0+872d972e41.nv24.08  (JetPack 6.1版本，兼容6.2)

# 依賴
numpy==1.26.4
ultralytics==8.1.0
opencv-python==4.10.0.82

# CUDA庫
libcusparselt0==0.7.1.0-1  ← 關鍵！
libcublas-12-6
cuda-toolkit-12-6
```

### 環境變數（可選，通常不需要）
```bash
# 如果仍有問題，可以手動添加到 ~/.bashrc
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:$LD_LIBRARY_PATH
```

## 📝 關鍵知識點

### 1. 為什麼需要libcusparselt？
- **cuSPARSELt** = CUDA Sparse Matrix Linear Algebra Library
- PyTorch的某些操作（特別是transformer和現代神經網路）使用稀疏矩陣運算
- 在Jetson上，這個庫不隨標準CUDA一起安裝，需要單獨下載

### 2. PyTorch版本兼容性
- JetPack 6.2 官方PyTorch尚未發布（截至2025-12-31）
- JetPack 6.1的PyTorch (2.5.0a0) **可以**在JetPack 6.2上工作
- 關鍵是安裝正確的CUDA庫依賴

### 3. NumPy版本重要性
- **必須使用NumPy < 2.0**
- ROS2 Humble的cv_bridge與NumPy 2.x不兼容
- 推薦: `numpy==1.26.4`

## 🚨 常見問題

### Q1: CUDA顯示True但YOLOv8仍然慢？
```bash
# 檢查YOLOv8是否真的在使用GPU
python3 -c "
from ultralytics import YOLO
import torch
model = YOLO('yolov8n.pt')
print(f'Model device: {next(model.model.parameters()).device}')
"
```

應該輸出: `Model device: cuda:0`

### Q2: 執行時出現"CUDA out of memory"？
- Orin NX 16GB應該足夠YOLOv8n/s
- 如果仍有問題，使用更小的模型: `yolov8n.pt`
- 或降低輸入解析度

### Q3: 找不到cusparselt下載？
備用下載位置：
```bash
# NVIDIA官方下載頁面
https://developer.nvidia.com/cusparselt-downloads

# 選擇: cuSPARSELt v0.7.1 → Local Installers → Ubuntu 22.04 ARM64 (Tegra)
```

## 📚 參考資源

- **NVIDIA Jetson論壇 - PyTorch**: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
- **cuSPARSELt文檔**: https://docs.nvidia.com/cuda/cusparselt/
- **JetPack檔案**: https://developer.nvidia.com/embedded/jetpack

## 🎉 成功標誌

當您看到以下輸出，表示一切正常：

```bash
# 1. PyTorch CUDA可用
$ python3 -c "import torch; print(torch.cuda.is_available())"
True  ✅

# 2. YOLOv8高FPS
$ python3 test_detection_yolo.py
裝置: cuda:0  ✅
[INFO] ✅ 偵測到 1 人 | FPS: 25.3 | 推理: 39.5ms  ✅

# 3. GPU使用率
$ sudo tegrastats
...
GR3D_FREQ 50%  ✅ (GPU正在工作)
```

---

**文檔更新**: 2025-12-31
**測試平台**: Jetson Orin NX 16GB, JetPack 6.2
**狀態**: ✅ 驗證成功


