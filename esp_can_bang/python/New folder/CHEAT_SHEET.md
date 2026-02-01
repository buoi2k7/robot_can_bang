# 🚀 PID TUNING CHEAT SHEET

## ⚡ Quick Commands

```bash
# Cài đặt
pip install numpy matplotlib

# Chạy nhanh nhất
python start.py                    # Menu chọn tool

# Hoặc chạy trực tiếp
python gui_pid_tuner.py            # GUI auto-tune
python advanced_pid_tuner.py       # CLI auto-tune  
python compare_pid.py              # So sánh configs
```

## 🎯 Which Tool to Use?

| Tình huống | Tool | Command |
|------------|------|---------|
| Lần đầu tune | GUI Auto-Tuner | `python gui_pid_tuner.py` |
| Muốn control nhiều | CLI Advanced | `python advanced_pid_tuner.py` |
| So sánh configs | Compare Tool | `python compare_pid.py` |
| Tinh chỉnh thủ công | GUI Manual | `python GuiK_V2_OK.py` |

## 📊 Đánh giá kết quả

| Stability Score | Đánh giá | Hành động |
|-----------------|----------|-----------|
| 80-100 | 🎉 Xuất sắc | ✅ Dùng ngay |
| 60-79 | 👍 Tốt | ⚠️ Có thể dùng |
| 40-59 | 😐 Khả dụng | 🔧 Tinh chỉnh thêm |
| 0-39 | ❌ Kém | 🔄 Tune lại |

## 🔧 Troubleshooting 1-Liner

```python
# Robot cứ ngã → Tăng MAX_ANGLE
MAX_ANGLE = 30.0

# Không tìm được config tốt → Mở rộng range
K1_RANGE = (20, 150)
K2_RANGE = (3, 50)

# Kết quả không nhất quán → Tăng test time
TEST_DURATION = 6.0
POPULATION_SIZE = 16
```

## 🎓 PID Quick Guide

```
K1 (P - Proportional):
├─ Phản ứng với góc lệch
├─ Càng cao: nhanh nhưng overshoot
└─ Sweet spot: 50-80

K2 (D - Derivative):
├─ Phanh/damping
├─ Càng cao: ổn định nhưng chậm
└─ Sweet spot: 15-25

K3 (Motor Feedback):
├─ Bù độ trễ motor
├─ Tùy loại motor
└─ Sweet spot: 0.8-2.0
```

## ⏱️ Thời gian dự kiến

| Mode | Thời gian | Khi nào dùng |
|------|-----------|--------------|
| GA only | 8-12 phút | Nhanh, kết quả tốt |
| Grid Search | 30-60 phút | Chắc chắn nhất |
| Hybrid | 15-20 phút | ⭐ Khuyến nghị |

## 📝 Workflow khuyến nghị

```
1. Kiểm tra hardware
   ├─ MPU6050 đã chuẩn?
   ├─ Motor đúng chiều?
   └─ Offset góc = 0°?

2. Chạy Auto-Tuner
   └─ python gui_pid_tuner.py

3. Lấy kết quả
   └─ Copy code vào ESP32

4. Test và so sánh
   └─ python compare_pid.py

5. Fine-tune (optional)
   └─ python GuiK_V2_OK.py
```

## 🚨 Safety Checklist

- [ ] Robot trên sàn phẳng
- [ ] Pin đầy / nguồn ổn
- [ ] Không vật cản xung quanh
- [ ] WiFi kết nối tốt
- [ ] IP ESP32 đúng
- [ ] Đã test motor riêng
- [ ] MPU6050 chuẩn
- [ ] Offset = 0° khi thẳng

## 💾 Files Output

```
best_pid_20260131_143045.txt       # Kết quả auto-tune
comparison_results_*.json          # Kết quả so sánh
```

## 🔗 Links

- **Hướng dẫn đầy đủ**: `HUONG_DAN_SU_DUNG.md`
- **Technical docs**: `README.md`
- **Quick start**: `python start.py`

## 📞 Common Issues

| Vấn đề | Giải pháp nhanh |
|--------|-----------------|
| Robot ngã ngay | Kiểm tra offset góc |
| Kết nối UDP fail | Kiểm tra IP + Port |
| Import error | `pip install -r requirements.txt` |
| Không đủ data | Tăng TEST_DURATION |
| Score thấp | Kiểm tra hardware |

## 🎯 Expected Results

**Good config mẫu:**
```python
K1 = 67.5   # P gain
K2 = 18.2   # D gain
K3 = 1.35   # Motor feedback

# Performance:
Avg Error: 0.85°
Max Error: 8.20°
Std Dev: 0.92°
Score: 82.3/100
```

---

**Pro tip**: Chạy tool 2-3 lần trong điều kiện khác nhau, lấy config tốt nhất! 🚀
