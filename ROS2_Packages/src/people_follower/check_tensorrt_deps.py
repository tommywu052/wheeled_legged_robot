#!/usr/bin/env python3
"""
TensorRT 依賴檢查工具

快速檢查系統是否已具備轉換 YOLOv8 為 TensorRT 的所有依賴。

使用方法:
    python3 check_tensorrt_deps.py

作者: AI Assistant
日期: 2026-01-02
"""

import sys
import subprocess

def print_header(text):
    """打印分隔線標題"""
    print("\n" + "=" * 60)
    print(f"  {text}")
    print("=" * 60)


def print_section(text):
    """打印子標題"""
    print(f"\n📋 {text}")
    print("-" * 60)


def check_command(cmd, description):
    """檢查命令是否可用"""
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=5
        )
        return result.returncode == 0, result.stdout.strip()
    except Exception as e:
        return False, str(e)


def check_python_module(module_name, import_name=None):
    """檢查 Python 模組是否可導入"""
    if import_name is None:
        import_name = module_name
    
    try:
        exec(f"import {import_name}")
        return True
    except ImportError:
        return False


def main():
    print_header("TensorRT 依賴檢查工具")
    
    all_good = True
    warnings = []
    errors = []
    
    # ========== 1. JetPack / L4T 版本 ==========
    print_section("1. JetPack / L4T 版本")
    
    success, output = check_command(
        "cat /etc/nv_tegra_release 2>/dev/null || echo 'Not found'",
        "JetPack version"
    )
    
    if "Not found" not in output:
        print(f"✅ {output}")
    else:
        print("⚠️  無法檢測 JetPack 版本（可能不是 Jetson 設備）")
        warnings.append("無法檢測 JetPack 版本")
    
    # ========== 2. CUDA ==========
    print_section("2. CUDA")
    
    success, output = check_command(
        "nvcc --version 2>/dev/null | grep 'release' || echo 'Not found'",
        "CUDA"
    )
    
    if "Not found" not in output:
        print(f"✅ CUDA: {output.split('release')[-1].strip()}")
    else:
        print("❌ CUDA 未安裝或未在 PATH 中")
        errors.append("CUDA 未安裝")
        all_good = False
    
    # ========== 3. TensorRT ==========
    print_section("3. TensorRT")
    
    success, output = check_command(
        "dpkg -l | grep 'tensorrt' | head -1",
        "TensorRT"
    )
    
    if success and output:
        print(f"✅ TensorRT 已安裝")
        print(f"   {output}")
    else:
        print("❌ TensorRT 未安裝")
        errors.append("TensorRT 未安裝 (需要 JetPack)")
        all_good = False
    
    # ========== 4. Python 版本 ==========
    print_section("4. Python 版本")
    
    python_version = sys.version.split()[0]
    print(f"✅ Python {python_version}")
    
    # ========== 5. PyTorch ==========
    print_section("5. PyTorch")
    
    try:
        import torch
        print(f"✅ PyTorch 版本: {torch.__version__}")
        
        # 檢查 CUDA 支持
        if torch.cuda.is_available():
            print(f"✅ CUDA 可用")
            print(f"   CUDA 版本: {torch.version.cuda}")
            print(f"   GPU: {torch.cuda.get_device_name(0)}")
        else:
            print("❌ PyTorch 無法使用 CUDA")
            errors.append("PyTorch 不支持 CUDA")
            all_good = False
            
    except ImportError:
        print("❌ PyTorch 未安裝")
        errors.append("PyTorch 未安裝")
        all_good = False
        print("   安裝方法: 請參考 https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048")
    
    # ========== 6. Ultralytics ==========
    print_section("6. Ultralytics")
    
    try:
        import ultralytics
        print(f"✅ Ultralytics 版本: {ultralytics.__version__}")
        
        # 檢查版本是否足夠新
        version_parts = ultralytics.__version__.split('.')
        major = int(version_parts[0])
        minor = int(version_parts[1]) if len(version_parts) > 1 else 0
        
        if major < 8 or (major == 8 and minor < 0):
            print("⚠️  版本較舊，建議升級")
            warnings.append("Ultralytics 版本較舊")
            print("   升級: pip3 install --upgrade ultralytics")
            
    except ImportError:
        print("❌ Ultralytics 未安裝")
        errors.append("Ultralytics 未安裝")
        all_good = False
        print("   安裝: pip3 install ultralytics")
    
    # ========== 7. ONNX ==========
    print_section("7. ONNX (可選)")
    
    try:
        import onnx
        print(f"✅ ONNX 版本: {onnx.__version__}")
    except ImportError:
        print("⚠️  ONNX 未安裝（轉換時可能需要）")
        warnings.append("ONNX 未安裝")
        print("   安裝: pip3 install onnx")
    
    # ========== 8. 其他依賴 ==========
    print_section("8. 其他依賴")
    
    modules = [
        ('numpy', 'NumPy'),
        ('cv2', 'OpenCV'),
        ('PIL', 'Pillow'),
    ]
    
    for module, name in modules:
        if check_python_module(module):
            print(f"✅ {name}")
        else:
            print(f"⚠️  {name} 未安裝")
            warnings.append(f"{name} 未安裝")
    
    # ========== 總結 ==========
    print_header("檢查結果")
    
    if all_good and not errors:
        print("\n🎉 恭喜！所有必需依賴都已安裝")
        print("\n✅ 您可以直接轉換模型:")
        print("   python3 convert_to_tensorrt.py")
        print("   或")
        print("   yolo export model=yolov8n-pose.pt format=engine")
        
        if warnings:
            print(f"\n⚠️  有 {len(warnings)} 個警告（不影響使用）:")
            for w in warnings:
                print(f"   - {w}")
    else:
        print("\n❌ 發現問題，需要先解決:")
        for e in errors:
            print(f"   ❌ {e}")
        
        if warnings:
            print(f"\n⚠️  警告 ({len(warnings)} 個):")
            for w in warnings:
                print(f"   ⚠️  {w}")
        
        print("\n📚 解決方案:")
        print("   1. 確保已安裝 JetPack (包含 TensorRT 和 CUDA)")
        print("   2. 安裝 PyTorch: 查看 https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048")
        print("   3. 安裝 Ultralytics: pip3 install ultralytics")
        print("\n   或使用 Docker (推薦):")
        print("   sudo docker pull ultralytics/ultralytics:latest-jetson-jetpack5")
    
    # ========== 測試建議 ==========
    if all_good:
        print("\n" + "=" * 60)
        print("  🧪 建議測試步驟")
        print("=" * 60)
        print("\n1. 測試 PyTorch 推理:")
        print("   python3 -c \"from ultralytics import YOLO; YOLO('yolov8n.pt')('https://ultralytics.com/images/bus.jpg')\"")
        print("\n2. 測試 TensorRT 轉換:")
        print("   python3 convert_to_tensorrt.py")
        print("\n3. 查看詳細文檔:")
        print("   cat TensorRT依賴檢查.md")
    
    print("\n" + "=" * 60)
    
    return 0 if all_good else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n⚠️  檢查已取消")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 發生錯誤: {e}")
        sys.exit(1)


