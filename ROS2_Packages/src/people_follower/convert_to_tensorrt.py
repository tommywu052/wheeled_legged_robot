#!/usr/bin/env python3
"""
YOLOv8-Pose TensorRT 轉換工具

自動將 PyTorch 模型轉換為 TensorRT 引擎以提升性能。

使用方法:
    python3 convert_to_tensorrt.py
    python3 convert_to_tensorrt.py --model yolov8s-pose.pt
    python3 convert_to_tensorrt.py --model yolov8n-pose.pt --half --imgsz 480

作者: AI Assistant
日期: 2026-01-02
"""

import argparse
import os
import sys
import time

def check_cuda():
    """檢查 CUDA 是否可用"""
    try:
        import torch
        if torch.cuda.is_available():
            print(f"✅ CUDA 可用")
            print(f"   裝置: {torch.cuda.get_device_name(0)}")
            print(f"   CUDA 版本: {torch.version.cuda}")
            return True
        else:
            print("❌ CUDA 不可用！")
            print("   TensorRT 需要 CUDA 支持")
            return False
    except ImportError:
        print("❌ PyTorch 未安裝！")
        print("   請安裝: pip3 install torch")
        return False


def convert_to_tensorrt(model_path, half=True, imgsz=640, verbose=True):
    """轉換模型為 TensorRT 引擎
    
    參數:
        model_path: PyTorch 模型路徑 (.pt)
        half: 是否使用 FP16 精度（推薦）
        imgsz: 輸入圖像尺寸（默認 640）
        verbose: 顯示詳細日誌
    
    返回:
        引擎路徑或 None（失敗時）
    """
    try:
        from ultralytics import YOLO
        
        # 檢查模型文件是否存在
        if not os.path.exists(model_path):
            print(f"❌ 模型文件不存在: {model_path}")
            return None
        
        print(f"\n📦 載入模型: {model_path}")
        model = YOLO(model_path)
        
        # 生成引擎路徑
        engine_path = model_path.replace('.pt', '.engine')
        
        # 檢查是否已存在引擎
        if os.path.exists(engine_path):
            print(f"\n⚠️  TensorRT 引擎已存在: {engine_path}")
            response = input("是否覆蓋？(y/N): ").strip().lower()
            if response not in ['y', 'yes']:
                print("取消轉換")
                return engine_path
            print("刪除舊引擎...")
            os.remove(engine_path)
        
        # 開始轉換
        print(f"\n🔄 開始轉換為 TensorRT...")
        print(f"   精度模式: {'FP16' if half else 'FP32'}")
        print(f"   輸入尺寸: {imgsz}x{imgsz}")
        print(f"   預計時間: 5-10 分鐘（首次轉換）")
        print(f"   請耐心等待...\n")
        
        start_time = time.time()
        
        # 執行轉換
        model.export(
            format='engine',
            half=half,
            imgsz=imgsz,
            verbose=verbose
        )
        
        elapsed_time = time.time() - start_time
        
        # 檢查結果
        if os.path.exists(engine_path):
            file_size_mb = os.path.getsize(engine_path) / (1024 * 1024)
            print(f"\n✅ 轉換成功！")
            print(f"   引擎路徑: {engine_path}")
            print(f"   檔案大小: {file_size_mb:.1f} MB")
            print(f"   轉換時間: {elapsed_time:.1f} 秒")
            print(f"\n🚀 預期性能提升: 2-4倍（相比 PyTorch）")
            return engine_path
        else:
            print(f"\n❌ 轉換失敗：未生成引擎文件")
            return None
            
    except ImportError:
        print("❌ Ultralytics 未安裝！")
        print("   請安裝: pip3 install ultralytics")
        return None
    except Exception as e:
        print(f"\n❌ 轉換失敗: {e}")
        return None


def update_config(engine_path):
    """更新配置文件以使用 TensorRT 引擎"""
    config_path = "config/posture_mimic_params.yaml"
    
    if not os.path.exists(config_path):
        print(f"\n⚠️  配置文件不存在: {config_path}")
        return
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        # 修改 yolo_model 行
        modified = False
        for i, line in enumerate(lines):
            if 'yolo_model:' in line and not line.strip().startswith('#'):
                # 保留縮進
                indent = len(line) - len(line.lstrip())
                lines[i] = ' ' * indent + f'yolo_model: "{os.path.basename(engine_path)}"  # TensorRT 引擎（自動生成）\n'
                modified = True
                break
        
        if modified:
            print(f"\n📝 更新配置文件...")
            response = input(f"是否自動更新 {config_path}？(y/N): ").strip().lower()
            if response in ['y', 'yes']:
                with open(config_path, 'w', encoding='utf-8') as f:
                    f.writelines(lines)
                print(f"✅ 配置文件已更新")
            else:
                print(f"請手動編輯 {config_path}:")
                print(f'   yolo_model: "{os.path.basename(engine_path)}"')
        
    except Exception as e:
        print(f"⚠️  更新配置文件失敗: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='YOLOv8-Pose TensorRT 轉換工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
範例:
  %(prog)s                                      # 轉換默認模型
  %(prog)s --model yolov8s-pose.pt             # 轉換指定模型
  %(prog)s --model yolov8n-pose.pt --imgsz 480 # 使用較小尺寸
  %(prog)s --no-half                           # 使用 FP32 精度
        """
    )
    
    parser.add_argument(
        '--model',
        type=str,
        default='yolov8n-pose.pt',
        help='PyTorch 模型路徑（默認: yolov8n-pose.pt）'
    )
    
    parser.add_argument(
        '--half',
        action='store_true',
        default=True,
        help='使用 FP16 精度（推薦，默認啟用）'
    )
    
    parser.add_argument(
        '--no-half',
        action='store_false',
        dest='half',
        help='使用 FP32 精度（精度高但速度慢）'
    )
    
    parser.add_argument(
        '--imgsz',
        type=int,
        default=640,
        help='輸入圖像尺寸（默認: 640）'
    )
    
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='減少日誌輸出'
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("  YOLOv8-Pose TensorRT 轉換工具")
    print("=" * 60)
    
    # 檢查 CUDA
    if not check_cuda():
        print("\n⚠️  建議在支持 CUDA 的設備上運行此腳本")
        sys.exit(1)
    
    # 執行轉換
    engine_path = convert_to_tensorrt(
        model_path=args.model,
        half=args.half,
        imgsz=args.imgsz,
        verbose=not args.quiet
    )
    
    if engine_path:
        # 提示更新配置
        update_config(engine_path)
        
        print("\n" + "=" * 60)
        print("  🎉 完成！")
        print("=" * 60)
        print("\n下一步:")
        print("  1. 啟動 ROS2 節點:")
        print("     ros2 launch people_follower posture_mimic.launch.py")
        print("\n  2. 查看性能提升（FPS 顯示在視窗右上角）")
        print("\n  3. 參考文檔:")
        print("     cat TensorRT優化指南.md")
        
        sys.exit(0)
    else:
        print("\n❌ 轉換失敗")
        sys.exit(1)


if __name__ == '__main__':
    main()


