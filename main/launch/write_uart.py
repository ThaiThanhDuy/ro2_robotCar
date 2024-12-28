import time

def process_data():
    while True:
        try:
            with open("uart_data.txt", "r") as file:
                data = file.read()
                if data:
                    print(f"Nhận dữ liệu: {data}")
        except FileNotFoundError:
            pass
        time.sleep(0.1)

if _name_ == "_main_":
    process_data()
