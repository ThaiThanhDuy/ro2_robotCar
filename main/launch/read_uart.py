import serial
import time

def uart_reader():
    ser = serial.Serial('/dev/ttyUSB3', 115200)
    while True:
        data = ser.readline().decode('utf-8').strip()
        with open("uart_data.txt", "w") as file:
            file.write(data)
        time.sleep(0.1)  # Đảm bảo không ghi liên tục quá nhanh

if _name_ == "_main_":
    uart_reader()
