import socket
import serial
import threading

def read_uart(port, baudrate):
    ser = serial.Serial(port, baudrate)
    while True:
        data = ser.readline().decode('utf-8').strip()
        print(f"Sending data: {data}")
        for client in clients:
            client.sendall(data.encode('utf-8'))

def handle_client(client_socket):
    while True:
        try:
            client_socket.recv(1024)  # Keep the connection alive
        except:
            break
    client_socket.close()

clients = []

def start_server(host='localhost', port=5000):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port))
    server.listen(5)
    print("Server listening...")
    
    threading.Thread(target=read_uart, args=('/dev/ttyUSB3', 115200), daemon=True).start()
    
    while True:
        client_socket, addr = server.accept()
        print(f"Accepted connection from {addr}")
        clients.append(client_socket)
        threading.Thread(target=handle_client, args=(client_socket,), daemon=True).start()

if __name__ == "__main__":
    start_server()
