import socket
import serial
import threading
import argparse

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
    print(f"Server listening on {host}:{port}...")
    
    threading.Thread(target=read_uart, args=('/dev/ttyUSB3', 115200), daemon=True).start()
    
    while True:
        client_socket, addr = server.accept()
        print(f"Accepted connection from {addr}")
        clients.append(client_socket)
        threading.Thread(target=handle_client, args=(client_socket,), daemon=True).start()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='UART Server')
    parser.add_argument('--host', type=str, default='localhost', help='Host address for the server')
    parser.add_argument('--port', type=int, default=5000, help='Port number for the server')
    parser.add_argument('--uart-port', type=str, default='/dev/ttyUSB3', help='UART port to read from')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate for UART communication')

    args = parser.parse_args()

    # Start the server with the specified host and port
    start_server(host=args.host, port=args.port)

#python uart_server.py --host 0.0.0.0 --port 5000 --uart-port /dev/ttyUSB3 --baudrate 115200
