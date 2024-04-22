import socket, threading, time

class ClientSide:
    def __init__(self) -> None:
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_socket.settimeout(3)
        server_address = ('0.0.0.0', 8080)
        while True:
            try:
                self.client_socket.connect(server_address)
                break
            except socket.timeout:
                print("Connection failed. Retrying in 3 seconds...")
                time.sleep(3)
        print("Connection established at from client side.")
        return
    
    def __send(self, msg) -> object:
        self.client_socket.sendall(msg.encode())
        data = self.client_socket.recv(1024).decode()
        return data
    
    def sending(self, msg) -> None:
        sending_thread = threading.Thread(target=self.__send, args=(msg + '\n', ))
        sending_thread.start()
        print("Msg sent.")
        return
    
    def shutdown(self) -> None:
        self.client_socket.close()
        print("Connection closed.")
        return
