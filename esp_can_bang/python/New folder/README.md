# 🤖 Advanced PID Auto-Tuner cho Robot Cân Bằng

## 🎯 Tổng quan

Bộ công cụ tự động tìm kiếm PID cho robot cân bằng 2 bánh sử dụng **Genetic Algorithm** và **Adaptive Search**. Không cần phải dò thủ công hàng trăm lần!

### ✨ Tính năng chính

- ✅ **Genetic Algorithm** - tìm kiếm thông minh trong không gian PID
- ✅ **Adaptive Grid Search** - fine-tune chính xác
- ✅ **Multi-metric evaluation** - đánh giá toàn diện (stability, overshoot, settling time)
- ✅ **Real-time GUI** - theo dõi quá trình tìm kiếm
- ✅ **Safety features** - dừng tự động khi robot ngã
- ✅ **Comparison tool** - so sánh nhiều config
- ✅ **Auto-save results** - lưu kết quả tự động

## 📦 Cài đặt

### 1. Clone hoặc download các file

```bash
# Download tất cả các file trong folder này
```

### 2. Cài đặt dependencies

```bash
pip install -r requirements.txt
```

Hoặc cài thủ công:

```bash
pip install numpy matplotlib
```

### 3. Cấu hình IP ESP32

Mở file và sửa IP của ESP32:

```python
ESP32_IP = "192.168.1.7"  # Thay bằng IP thật của bạn
```

## 🚀 Sử dụng nhanh

### Phương án 1: GUI (Khuyến nghị)

```bash
python gui_pid_tuner.py
```

Giao diện hiện ra → Click "Start Tuning" → Đợi kết quả!

### Phương án 2: CLI Advanced

```bash
python advanced_pid_tuner.py
```

Chọn mode:
- **Mode 1**: Genetic Algorithm (nhanh - 8-10 phút)
- **Mode 2**: Grid Search (chậm - 30-40 phút)  
- **Mode 3**: Hybrid (tốt nhất - 15-20 phút) ⭐ **KHUYẾN NGHỊ**

### Phương án 3: So sánh configs

```bash
python compare_pid.py
```

Dùng để so sánh config cũ với config mới tìm được.

## 📁 Cấu trúc file

```
├── advanced_pid_tuner.py    # Tool CLI với GA + Grid Search
├── gui_pid_tuner.py         # Tool GUI với visualization
├── compare_pid.py           # Tool so sánh configs
├── HUONG_DAN_SU_DUNG.md     # Hướng dẫn chi tiết (Vietnamese)
├── README.md                # File này
├── requirements.txt         # Dependencies
├── auto_pid_tuner.py        # Legacy tool (simple)
└── GuiK_V2_OK.py           # Legacy GUI manual tuner
```

## 🎓 Cách hoạt động

### Genetic Algorithm

```
1. Khởi tạo quần thể (population)
   ├─ Seed configs tốt từ kinh nghiệm
   └─ Random configs

2. Đánh giá (evaluate)
   ├─ Test từng config trên robot
   ├─ Đo: avg error, max error, stability
   └─ Cho điểm 0-100

3. Chọn lọc (selection)
   ├─ Giữ top 20% (elitism)
   └─ Loại config kém

4. Lai ghép (crossover)
   ├─ Kết hợp 2 config tốt
   └─ Tạo config con

5. Đột biến (mutation)
   ├─ Thay đổi ngẫu nhiên nhỏ
   └─ Tránh local optimum

6. Lặp lại từ bước 2
```

### Grid Search

Thử toàn bộ các điểm trong không gian PID một cách có hệ thống. Chậm nhưng đảm bảo không bỏ sót vùng tốt.

### Hybrid (Khuyến nghị)

GA tìm vùng tốt → Grid Search fine-tune chính xác trong vùng đó.

## 📊 Kết quả mong đợi

### Tốt

```
Stability Score: 80-100
Avg Error: < 1.5°
Max Error: < 10°
Std Dev: < 1.0°
```

### Khá

```
Stability Score: 60-79
Avg Error: 1.5-3.0°
Max Error: 10-15°
Std Dev: 1.0-2.0°
```

### Khả dụng

```
Stability Score: 40-59
Avg Error: 3.0-5.0°
Max Error: 15-20°
Std Dev: 2.0-3.0°
```

## 🔧 Troubleshooting

### Robot cứ ngã ngay

**Kiểm tra:**
- Offset góc đúng chưa? (robot thẳng = 0°)
- MPU6050 đã chuẩn chưa?
- Motor đấu đúng chiều chưa?
- Pin có đủ điện không?

**Giải pháp:**
```python
# Tăng MAX_ANGLE để test dễ hơn
MAX_ANGLE = 30.0  # trong code
```

### Không tìm được config tốt

**Giải pháp:**
```python
# Mở rộng range
K1_RANGE = (20, 150)  # thay vì (30, 120)
K2_RANGE = (3, 50)    # thay vì (5, 40)

# Tăng số test
POPULATION_SIZE = 16   # thay vì 12
GENERATIONS = 15       # thay vì 10
TEST_DURATION = 6.0    # thay vì 4.0
```

### Kết quả không nhất quán

**Nguyên nhân:**
- Sàn không phẳng
- Pin yếu
- Nhiễu WiFi/Bluetooth

**Giải pháp:**
- Test trên sàn phẳng
- Dùng nguồn adapter
- Chạy tool nhiều lần, lấy kết quả tốt nhất

## 📈 Tips

1. **Chạy nhiều lần** - Chạy 2-3 lần, lấy config tốt nhất
2. **Test điều kiện thật** - Tune trên sàn/môi trường sẽ dùng thật
3. **Pin đầy** - Luôn tune với pin đầy hoặc nguồn ổn định
4. **Lưu lịch sử** - Đừng xóa file kết quả, có thể cần sau này
5. **Fine-tune sau** - Dùng GUI manual để tinh chỉnh thêm nếu cần

## 📊 So sánh với phương pháp cũ

| Phương pháp | Thời gian | Chất lượng | Tự động | 
|-------------|-----------|------------|---------|
| Thử thủ công | 2-3 ngày | ⭐⭐ | ❌ |
| Tool cũ (tuần tự) | 15-20 phút | ⭐⭐⭐ | Một phần |
| **Tool mới (GA)** | **10-15 phút** | **⭐⭐⭐⭐⭐** | **✅** |

## 🔬 Technical Details

### Algorithm Parameters

```python
POPULATION_SIZE = 12      # Số cá thể mỗi thế hệ
GENERATIONS = 10          # Số thế hệ
MUTATION_RATE = 0.3       # Tỷ lệ đột biến
TEST_DURATION = 4.0       # Thời gian test mỗi config (giây)

# PID Search Space
K1_RANGE = (30, 120)      # P gain
K2_RANGE = (5, 40)        # D gain
K3_RANGE = (0.1, 3.0)     # Motor feedback
```

### Evaluation Metrics

**Stability Score** được tính như sau:

```python
score = 100
score -= min(avg_error * 8, 40)          # 40% weight
score -= min(std_error * 10, 30)         # 30% weight
score -= min(settle_time * 10, 20)       # 20% weight
score -= min(overshoot_count * 2, 10)    # 10% weight
```

## 📝 Output Files

### best_pid_YYYYMMDD_HHMMSS.txt

```
=== BEST PID CONFIGURATION ===
Timestamp: 2026-01-31 14:30:45

K1 = 67.50
K2 = 18.20
K3 = 1.35

Stability Score: 82.3/100
Avg Error: 0.85°
...
```

### comparison_results_YYYYMMDD_HHMMSS.json

```json
{
  "timestamp": "2026-01-31T14:30:45",
  "configs": [...],
  "results": {...},
  "rankings": [...],
  "recommended": {
    "name": "GA Result",
    "k1": 67.5,
    "k2": 18.2,
    "k3": 1.35
  }
}
```

## 🌟 Ví dụ kết quả thực tế

```
🏆 KẾT QUẢ CUỐI CÙNG
=====================================
✅ BỘ PID TỐI ƯU:
   K1 (P)     = 67.50
   K2 (D)     = 18.20  
   K3 (Motor) = 1.35

📊 Performance:
   Stability Score: 82.3/100
   Avg Error: 0.85°
   Max Error: 8.20°
   Std Dev: 0.92°
   Settle Time: 1.15s
   Overshoot Count: 2

🎉 XUẤT SẮC! Robot nên đứng rất ổn định!
```

## 🤝 Contributing

Gặp bug hoặc có ý tưởng cải tiến? Tạo issue hoặc pull request!

## 📄 License

MIT License - Feel free to use and modify

## 🙏 Acknowledgments

- Dựa trên kinh nghiệm tune PID cho robot cân bằng
- Genetic Algorithm implementation
- Matplotlib visualization

---

**Happy Tuning! 🚀**

Robot của bạn sẽ sớm đứng vững như núi! ⛰️🤖
