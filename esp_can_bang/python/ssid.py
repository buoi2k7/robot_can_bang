import subprocess
import re

def get_local_ip_address():
    """
    Sử dụng lệnh 'ipconfig' của Windows để lấy Địa chỉ IPv4 cục bộ.
    """
    try:
        # Chạy lệnh ipconfig. Tùy chọn '/all' cung cấp thông tin chi tiết hơn.
        command = "ipconfig" 
        # Sử dụng encoding='cp850' hoặc 'utf-8' cho Windows
        result = subprocess.run(command, capture_output=True, text=True, check=True, encoding='utf-8', errors='ignore')
        
        # Định nghĩa các từ khóa để tìm kiếm, hỗ trợ cả tiếng Anh và tiếng Việt 
        # trong trường hợp lệnh ipconfig hiển thị bằng ngôn ngữ khác nhau.
        keywords = ["IPv4 Address", "Địa chỉ IPv4"]
        
        # Biến để kiểm tra xem đã tìm thấy địa chỉ IP trong phần Wi-Fi chưa
        is_wifi_adapter = False
        
        # Phân tích kết quả
        for line in result.stdout.split('\n'):
            line = line.strip()
            
            # Bước 1: Tìm kiếm bộ điều hợp Wi-Fi (thường có tên là Wireless LAN adapter Wi-Fi)
            if "Wireless LAN adapter Wi-Fi" in line or "Bộ điều hợp Mạng LAN không dây Wi-Fi" in line:
                is_wifi_adapter = True
                continue # Bắt đầu tìm kiếm IP trong khối này
            
            # Nếu đã ở trong khối Wi-Fi và tìm thấy địa chỉ IP
            if is_wifi_adapter:
                for keyword in keywords:
                    if keyword in line:
                        # Tách chuỗi tại dấu ':'
                        # Cắt bỏ mọi thứ, chỉ giữ lại phần sau dấu ':' và làm sạch
                        ip_full = line.split(':')[-1].strip()
                        
                        # Kiểm tra xem chuỗi có phải là một địa chỉ IP hợp lệ không (ví dụ: 192.168.1.1)
                        if re.match(r"\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}", ip_full):
                            return ip_full

    except subprocess.CalledProcessError:
        # Xử lý lỗi nếu lệnh không chạy được
        return "LỖI HỆ THỐNG: Không thể chạy lệnh ipconfig."
    except Exception as e:
        return f"Đã xảy ra lỗi bất ngờ: {e}"
        
    # Nếu không tìm thấy sau khi duyệt hết
    return "Không tìm thấy Địa chỉ IP Cục bộ (IPv4). Máy có thể chưa kết nối mạng."

# --- CHƯƠNG TRÌNH CHÍNH ---
print("====================================")
print("  TÌM ĐỊA CHỈ IP CỤC BỘ (WINDOWS)")
print("====================================")

ip_hien_tai = get_local_ip_address()

print("\n------------------------------------")
print(f"Địa chỉ IP cục bộ của máy bạn là: {ip_hien_tai}")
print("------------------------------------")

# Gợi ý cho LPT
if "LỖI" in ip_hien_tai or "Không tìm thấy" in ip_hien_tai:
    print("\n*Chú ý: Kiểm tra lại kết nối Wi-Fi trên máy bạn nhé. IP cục bộ thường bắt đầu bằng 192.168...")
else:
    print("\nĐây chính là cái dãy số 192.168.x.x mà bạn cần tìm rồi đó!")