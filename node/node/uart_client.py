import socket

def start_client(host='localhost', port=5000):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((host, port))
    
    while True:
        data = client.recv(1024).decode('utf-8')
        print(f"Received data: {data}")

if __name__ == "__main__":
    start_client()
