import serial
import socket
import time

def uart_reader(socket_server):
    ser = serial.Serial('/dev/ttyUSB3', 9600)
    while True:
        data = ser.readline().decode('utf-8').strip()
        socket_server.sendall(data.encode('utf-8'))

def start_server():
    host = 'localhost'
    port = 12345
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    
    print(f"Server đang chạy trên {host}:{port}")
    conn, addr = server_socket.accept()
    print(f"Đã kết nối với {addr}")
    
    uart_reader(conn)

if _name_ == "_main_":
    start_server()
