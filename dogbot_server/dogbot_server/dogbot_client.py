import socket, threading

class ClientSide:
    def __init__(self) -> None:
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_address = ('0.0.0.0', 8080)
        self.client_socket.connect(server_address)
        return
    
    def send(self, msg) -> object:
        self.client_socket.sendall(msg.encode())
        data = self.client_socket.recv(1024).decode()
        return data
    
    def sending(self, msg) -> None:
        sending_thread = threading.Thread(target=self.send, args=(msg))
        sending_thread.start()
        return
    
    def shutdown(self) -> None:
        self.client_socket.close()
        return
