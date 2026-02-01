#!/usr/bin/env python3
"""
QUICK START LAUNCHER
====================
Menu đơn giản để chọn tool muốn dùng
"""

import subprocess
import sys
import os

def print_banner():
    banner = """
╔══════════════════════════════════════════════════════════════╗
║                                                              ║
║    🤖  ADVANCED PID AUTO-TUNER FOR BALANCING ROBOT  🤖      ║
║                                                              ║
║                    Version 2.0 - 2026                        ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
    """
    print(banner)

def check_dependencies():
    """Kiểm tra dependencies"""
    try:
        import numpy
        import matplotlib
        return True
    except ImportError:
        return False

def main():
    print_banner()
    
    # Check dependencies
    if not check_dependencies():
        print("⚠️  THIẾU DEPENDENCIES!")
        print("\nVui lòng cài đặt:")
        print("  pip install numpy matplotlib")
        print("\nHoặc:")
        print("  pip install -r requirements.txt")
        input("\nNhấn ENTER để thoát...")
        return
    
    while True:
        print("\n" + "=" * 70)
        print("  📋 CHỌN TOOL BẠN MUỐN DÙNG")
        print("=" * 70)
        
        print("\n🎯 AUTO-TUNING TOOLS:")
        print("  1. GUI Auto-Tuner (Khuyến nghị - dễ dùng)")
        print("  2. CLI Auto-Tuner (Advanced - nhiều options)")
        
        print("\n🔬 ANALYSIS TOOLS:")
        print("  3. Compare PID Configs (So sánh nhiều config)")
        
        print("\n📚 MANUAL TOOLS (Legacy):")
        print("  4. GUI Manual Tuner (GuiK_V2_OK.py)")
        print("  5. Simple Auto Tuner (auto_pid_tuner.py)")
        
        print("\n📖 DOCUMENTATION:")
        print("  6. Xem hướng dẫn chi tiết")
        print("  7. Xem README")
        
        print("\n  0. Thoát")
        
        print("\n" + "=" * 70)
        choice = input("Lựa chọn (0-7): ").strip()
        
        if choice == "0":
            print("\n👋 Bye!")
            break
        
        elif choice == "1":
            print("\n🚀 Khởi động GUI Auto-Tuner...")
            print("📌 Nhớ đặt robot thẳng đứng trước khi click 'Start Tuning'!")
            input("\nNhấn ENTER để tiếp tục...")
            try:
                subprocess.run([sys.executable, "gui_pid_tuner.py"])
            except KeyboardInterrupt:
                print("\n🛑 Đã dừng")
            except Exception as e:
                print(f"\n❌ Lỗi: {e}")
        
        elif choice == "2":
            print("\n🚀 Khởi động CLI Auto-Tuner...")
            print("📌 Bạn sẽ được chọn mode: GA / Grid Search / Hybrid")
            input("\nNhấn ENTER để tiếp tục...")
            try:
                subprocess.run([sys.executable, "advanced_pid_tuner.py"])
            except KeyboardInterrupt:
                print("\n🛑 Đã dừng")
            except Exception as e:
                print(f"\n❌ Lỗi: {e}")
        
        elif choice == "3":
            print("\n🔬 Khởi động Comparison Tool...")
            print("📌 Dùng để so sánh hiệu suất của nhiều config")
            input("\nNhấn ENTER để tiếp tục...")
            try:
                subprocess.run([sys.executable, "compare_pid.py"])
            except KeyboardInterrupt:
                print("\n🛑 Đã dừng")
            except Exception as e:
                print(f"\n❌ Lỗi: {e}")
        
        elif choice == "4":
            print("\n🎮 Khởi động GUI Manual Tuner...")
            print("📌 Dùng để tinh chỉnh thủ công bằng sliders")
            input("\nNhấn ENTER để tiếp tục...")
            try:
                subprocess.run([sys.executable, "GuiK_V2_OK.py"])
            except KeyboardInterrupt:
                print("\n🛑 Đã dừng")
            except Exception as e:
                print(f"\n❌ Lỗi: {e}")
        
        elif choice == "5":
            print("\n🔧 Khởi động Simple Auto Tuner...")
            print("📌 Tool đơn giản, thử tuần tự các config")
            input("\nNhấn ENTER để tiếp tục...")
            try:
                subprocess.run([sys.executable, "auto_pid_tuner.py"])
            except KeyboardInterrupt:
                print("\n🛑 Đã dừng")
            except Exception as e:
                print(f"\n❌ Lỗi: {e}")
        
        elif choice == "6":
            print("\n" + "=" * 70)
            print("  📖 HƯỚNG DẪN CHI TIẾT")
            print("=" * 70)
            if os.path.exists("HUONG_DAN_SU_DUNG.md"):
                print("\n✅ File: HUONG_DAN_SU_DUNG.md")
                print("\nMở file này bằng text editor hoặc markdown viewer để xem")
                print("Hoặc xem trực tiếp trên GitHub")
            else:
                print("\n❌ File HUONG_DAN_SU_DUNG.md không tìm thấy")
            input("\nNhấn ENTER để quay lại...")
        
        elif choice == "7":
            print("\n" + "=" * 70)
            print("  📖 README")
            print("=" * 70)
            if os.path.exists("README.md"):
                print("\n✅ File: README.md")
                print("\nMở file này bằng text editor hoặc markdown viewer để xem")
                
                # Print first few lines
                try:
                    with open("README.md", "r", encoding="utf-8") as f:
                        lines = f.readlines()[:30]
                        print("\nPreview (30 dòng đầu):")
                        print("-" * 70)
                        for line in lines:
                            print(line, end="")
                        print("\n" + "-" * 70)
                        print(f"\n... còn {len(f.readlines())} dòng nữa")
                except:
                    pass
            else:
                print("\n❌ File README.md không tìm thấy")
            input("\nNhấn ENTER để quay lại...")
        
        else:
            print("\n❌ Lựa chọn không hợp lệ!")
            input("\nNhấn ENTER để thử lại...")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 Bye!")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        input("\nNhấn ENTER để thoát...")
