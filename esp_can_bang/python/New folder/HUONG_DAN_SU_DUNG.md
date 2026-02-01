# 🤖 HƯỚNG DẪN SỬ DỤNG AUTO PID TUNER V2.0

## 📋 Tổng quan

Bộ công cụ này giúp bạn **TỰ ĐỘNG** tìm bộ PID tối ưu cho robot cân bằng mà không cần phải thử thủ công hàng trăm lần.

### Các công cụ có sẵn:

1. **advanced_pid_tuner.py** - Tool CLI với Genetic Algorithm
2. **gui_pid_tuner.py** - Tool GUI với visualization real-time
3. **auto_pid_tuner.py** - Tool đơn giản ban đầu (legacy)
4. **GuiK_V2_OK.py** - GUI manual tuning (legacy)

## 🚀 Bắt đầu nhanh

### Cách 1: Dùng GUI (Khuyến nghị cho người mới)

```bash
python gui_pid_tuner.py
```

**Ưu điểm:**
- Thấy được real-time robot đang test như thế nào
- Theo dõi tiến trình bằng biểu đồ
- Dễ dùng, chỉ cần click nút

**Cách dùng:**
1. Chạy script
2. Đặt robot thẳng đứng
3. Click nút "Start Tuning"
4. Đợi từ 5-15 phút (tùy số generation)
5. Lấy kết quả best từ màn hình

### Cách 2: Dùng CLI Advanced (Khuyến nghị cho pro)

```bash
python advanced_pid_tuner.py
```

**Ưu điểm:**
- Nhiều option hơn (GA, Grid Search, Hybrid)
- Tự động lưu file kết quả
- Có thể chạy qua SSH/remote

**Cách dùng:**
1. Chạy script
2. Chọn phương pháp (1/2/3)
   - **1**: Chỉ dùng Genetic Algorithm (nhanh, 5-10 phút)
   - **2**: Chỉ dùng Grid Search (chậm, 20-40 phút, nhưng chắc chắn)
   - **3**: Hybrid - GA rồi fine-tune (khuyến nghị, 10-15 phút)
3. Nhấn ENTER để bắt đầu
4. Đợi hoàn tất
5. Kết quả lưu trong file `best_pid_YYYYMMDD_HHMMSS.txt`

## 📊 Giải thích các metrics

Khi tool chạy, bạn sẽ thấy các chỉ số sau:

### 1. **Avg Error (Lỗi trung bình)**
- Góc trung bình lệch so với 0°
- **Mục tiêu:** < 2.0° là tốt, < 1.0° là xuất sắc

### 2. **Max Error (Lỗi tối đa)**
- Góc lệch lớn nhất trong quá trình test
- **Mục tiêu:** < 15° là an toàn

### 3. **Std Dev (Độ lệch chuẩn)**
- Đo độ dao động/rung
- **Mục tiêu:** < 1.5° là ổn định, < 0.8° là rất ổn

### 4. **Stability Score (Điểm ổn định)**
- Điểm tổng hợp từ 0-100
- **Đánh giá:**
  - 80-100: Xuất sắc ✨
  - 60-79: Tốt 👍
  - 40-59: Khả dụng ⚠️
  - 0-39: Kém ❌

### 5. **Settle Time (Thời gian ổn định)**
- Thời gian để robot đứng yên (< 2° lệch)
- **Mục tiêu:** < 1.5s là tốt

### 6. **Overshoot Count**
- Số lần robot vọt qua vị trí cân bằng
- **Mục tiêu:** < 3 lần trong 4 giây test

## 🧬 Genetic Algorithm hoạt động thế nào?

### Nguyên lý cơ bản:

1. **Khởi tạo quần thể:**
   - Tạo N cá thể (bộ PID) ngẫu nhiên
   - Thêm một số "seed" tốt từ kinh nghiệm

2. **Đánh giá (Evaluation):**
   - Test từng cá thể trên robot thật
   - Cho điểm dựa trên performance

3. **Chọn lọc (Selection):**
   - Giữ lại các cá thể tốt nhất (elitism)
   - Loại bỏ cá thể yếu

4. **Lai ghép (Crossover):**
   - Kết hợp 2 cá thể tốt để tạo con
   - Ví dụ: (K1₁, K2₁, K3₁) + (K1₂, K2₂, K3₂) → con lai

5. **Đột biến (Mutation):**
   - Thay đổi ngẫu nhiên một chút giá trị
   - Giúp tránh bị kẹt ở local optimum

6. **Lặp lại:**
   - Quay lại bước 2 với quần thể mới
   - Dừng khi đạt mục tiêu hoặc hết generation

### Ví dụ cụ thể:

```
Generation 1:
  Config A: K1=60, K2=15, K3=1.5 → Score: 45
  Config B: K1=70, K2=20, K3=1.0 → Score: 62 ✓
  Config C: K1=50, K2=25, K3=1.2 → Score: 38
  ...
  → Giữ B, loại C
  → Lai B với A → Con D: K1=65, K2=17, K3=1.25
  → Đột biến D → D': K1=67, K2=16, K3=1.3

Generation 2:
  Test D', E, F, ...
  → Tiếp tục tiến hóa...
```

## ⚙️ Tùy chỉnh cấu hình

Nếu robot bạn **khác** với cấu hình mặc định, sửa các tham số sau:

### Trong `advanced_pid_tuner.py` hoặc `gui_pid_tuner.py`:

```python
# Safety
MAX_ANGLE = 25.0         # Góc tối đa, sửa thành 30 nếu robot chịu được
DANGER_ANGLE = 20.0      # Góc cảnh báo

# Tuning
POPULATION_SIZE = 12     # Số config mỗi thế hệ (càng nhiều càng chậm nhưng tốt hơn)
GENERATIONS = 10         # Số thế hệ (càng nhiều càng tốt nhưng lâu hơn)
TEST_DURATION = 4.0      # Thời gian test mỗi config (tăng lên 5-6s nếu cần chắc chắn hơn)

# PID Ranges
K1_RANGE = (30, 120)     # Giới hạn K1 (P gain)
K2_RANGE = (5, 40)       # Giới hạn K2 (D gain)
K3_RANGE = (0.1, 3.0)    # Giới hạn K3 (Motor feedback)
```

### Khi nào nên sửa?

**Tăng POPULATION_SIZE và GENERATIONS khi:**
- Robot rất khó điều khiển
- Bạn có thời gian (không gấp)
- Muốn kết quả chính xác nhất

**Tăng TEST_DURATION khi:**
- Robot cần thời gian lâu để ổn định
- Kết quả không nhất quán
- Môi trường có nhiễu (sàn không bằng, gió,...)

**Mở rộng PID_RANGE khi:**
- Tool không tìm được config tốt nào
- Bạn biết range hiện tại quá hẹp
- Đã test thủ công và biết cần giá trị ngoài range

## 🔧 Xử lý sự cố

### Vấn đề 1: Robot cứ ngã ngay

**Nguyên nhân có thể:**
- Offset góc sai (robot đọc 0° nhưng thực tế nghiêng)
- Cảm biến MPU6050 chưa chuẩn
- Motor đấu ngược chiều

**Giải pháp:**
1. Kiểm tra offset trong code ESP32
2. Chạy calibration MPU6050
3. Test motor riêng từng cái
4. Tăng MAX_ANGLE lên 30° để test

### Vấn đề 2: Không tìm được config tốt

**Nguyên nhân:**
- Range PID quá hẹp
- Thời gian test quá ngắn
- Robot có vấn đề phần cứng

**Giải pấp:**
1. Mở rộng K1_RANGE, K2_RANGE, K3_RANGE
2. Tăng TEST_DURATION lên 6-8s
3. Tăng POPULATION_SIZE và GENERATIONS
4. Chạy mode "Grid Search" để thử toàn bộ không gian

### Vấn đề 3: Kết quả không nhất quán

**Nguyên nhân:**
- Sàn nhà không bằng
- Pin yếu/điện áp không ổn định
- Nhiễu từ WiFi/Bluetooth

**Giải pháp:**
1. Test trên sàn phẳng, mịn
2. Sạc đầy pin hoặc dùng nguồn ổn áp
3. Tăng TEST_DURATION
4. Chạy nhiều lần và lấy kết quả trung bình

### Vấn đề 4: Tool báo lỗi kết nối UDP

**Giải pháp:**
```bash
# Kiểm tra IP ESP32
ping 192.168.1.7

# Kiểm tra port đang dùng
netstat -an | grep 4210

# Nếu port bị chiếm, kill process đang dùng
# Windows:
netstat -ano | findstr 4210
taskkill /PID <PID> /F

# Linux/Mac:
lsof -i :4210
kill -9 <PID>
```

## 📈 So sánh các phương pháp

| Phương pháp | Thời gian | Độ chính xác | Độ tin cậy | Khuyến nghị |
|-------------|-----------|--------------|------------|-------------|
| **Legacy thử tuần tự** | 10-15 phút | ⭐⭐ | ⭐⭐ | ❌ Không dùng |
| **Genetic Algorithm** | 8-12 phút | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ✅ Tốt |
| **Grid Search** | 30-60 phút | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⚠️ Nếu có thời gian |
| **Hybrid (GA + Grid)** | 15-20 phút | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ✅ **KHUYẾN NGHỊ** |

## 💡 Tips & Tricks

### Tip 1: Chạy nhiều lần
Chạy tool 2-3 lần trong các điều kiện khác nhau (sàn khác, pin mới/cũ). Lấy config có điểm trung bình cao nhất.

### Tip 2: Fine-tune thủ công sau
Sau khi có config tốt từ tool, dùng GUI manual (`GuiK_V2_OK.py`) để tinh chỉnh thêm ±5% nếu cần.

### Tip 3: Lưu lại lịch sử
Tool tự động lưu file `best_pid_YYYYMMDD_HHMMSS.txt`. Đừng xóa! Nếu sau này robot bị lỗi, bạn có thể quay lại config cũ.

### Tip 4: Test trong điều kiện thật
Nếu robot sẽ chạy trên sàn gỗ, hãy tune trên sàn gỗ. Nếu sẽ chạy trên gạch, tune trên gạch.

### Tip 5: Kiểm tra pin
Pin yếu → motor yếu → PID sai. Luôn tune với pin đầy hoặc nguồn adapter.

## 📝 Checklist trước khi chạy

- [ ] ESP32 đã flash code và đang chạy
- [ ] Kết nối WiFi ổn định
- [ ] Robot được đặt trên sàn phẳng
- [ ] Pin đầy hoặc dùng nguồn adapter
- [ ] Đã test motor riêng (chạy đúng chiều)
- [ ] MPU6050 đã calibrate
- [ ] Offset góc đã điều chỉnh (robot thẳng = 0°)
- [ ] IP và PORT trong code Python khớp với ESP32
- [ ] Không có vật cản xung quanh robot

## 🎯 Kết quả mong đợi

Sau khi chạy xong, bạn sẽ có:

1. **File kết quả** với bộ PID tốt nhất
2. **Stability Score** > 60 (tối thiểu)
3. **Code ESP32** sẵn sàng copy-paste
4. Robot có thể **đứng ổn định** > 30 giây

### Ví dụ kết quả tốt:

```
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
```

## 🚨 Lưu ý quan trọng

1. **An toàn:** Robot có thể ngã bất cứ lúc nào. Đặt trên sàn có đệm hoặc theo dõi sát.

2. **Nhiễu môi trường:** Tránh test gần thiết bị phát WiFi mạnh, hoặc khi có người đi lại nhiều.

3. **Thời gian:** Đừng ngắt giữa chừng. Genetic algorithm cần chạy đủ generations mới hiệu quả.

4. **Không phải lúc nào cũng tìm được:** Nếu robot có vấn đề phần cứng nghiêm trọng, tool cũng không thể cứu.

## 🎓 Hiểu sâu hơn về PID

### K1 (P - Proportional):
- Phản ứng với góc lệch hiện tại
- **Càng cao:** phản ứng nhanh, nhưng dễ overshoot
- **Càng thấp:** phản ứng chậm, robot ngã nhanh
- **Range tốt:** 50-80

### K2 (D - Derivative):
- Phản ứng với tốc độ thay đổi góc (phanh)
- **Càng cao:** phanh mạnh, ổn định nhưng có thể rung
- **Càng thấp:** phanh yếu, dễ vọt lố
- **Range tốt:** 15-25

### K3 (Motor Feedback):
- Bù cho độ trễ của motor
- **Càng cao:** motor responsive nhưng dễ rung
- **Càng thấp:** motor chậm
- **Range tốt:** 0.8-2.0

### Tương tác giữa các hệ số:
- K1 và K2 thường tỷ lệ 3:1 đến 4:1
- K3 phụ thuộc vào encoder và loại motor
- Không có công thức chính xác → phải tune

## 📞 Hỗ trợ

Nếu gặp vấn đề:
1. Đọc lại phần "Xử lý sự cố"
2. Kiểm tra Checklist
3. Thử các config mẫu trong code
4. Giảm expectations: Score 60-70 đã rất tốt rồi!

---

**Good luck! 🍀**

Hy vọng robot của bạn sẽ đứng thẳng và ổn định! 🤖
