import socket

def client(msg) -> None:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server_address = ('0.0.0.0', 8080)
    client_socket.connect(server_address)

    while True:
        client_socket.sendall(msg.encode())
        data = client_socket.recv(1024).decode()

    client_socket.close()