import socket, threading, time

class ClientSide:
    def __init__(self) -> None:
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_socket.settimeout(0.5)
        # self.hearing_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.hearing_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.hearing_socket.settimeout(3)
        # self.hearing_socket.bind(('0.0.0.0', 8080))
        server_address = ('192.168.50.100', 8080)
        self.close = False
        while True:
            try:
                self.client_socket.connect(server_address)
                print("Connection established at client side.")
                time.sleep(1)
                # self.regular_listen()
                break
            except socket.timeout:
                print("Connection failed. Retrying in 3 seconds...")
                time.sleep(3)
        return
    
    def __send(self, msg) -> object:
            # data = self.client_socket.recv(64).decode()
            # print(data, flush=True)
        try:
            self.client_socket.send(msg.encode())
            if "echoback" in msg:
                data = self.client_socket.recv(128).decode()
                print(data, flush=True)
        except TimeoutError as e:
            print(e)
        return
    
    def sending(self, msg) -> None:
        sending_thread = threading.Thread(target=self.__send, args=(msg + '\n', ))
        sending_thread.start()
        # print("Msg sent.", end='')
        return
    
    def shutdown(self) -> None:
        self.client_socket.close()
        self.close = True
        print("Connection closed.")
        return
    
    def __listen(self) -> None:
        while self.close == False:
            self.__send("echoback")
            print("regular msg sent.")
            time.sleep(1)
        return
    
    def regular_listen(self) -> None:
        listening_thread = threading.Thread(target=self.__listen)
        listening_thread.start()
        print("Regular echo back thread start running.")
        return

def main_thread() -> None:
    while True:
        try:
            userIn = input(">> ", )
            if userIn == 'q':
                client.shutdown()
                exit(0)
            elif userIn == "crusing":
                client.sending(userIn)
            elif userIn == "approaching":
                client.sending(userIn)
            else:
                client.sending(userIn)
        except KeyboardInterrupt:
            print(flush=True)
        except Exception as e:
            print(e)

def stateMonitoring() -> None:
    while True:
        client.sending("echoback")
        # print("regular msg sent.")
        time.sleep(1)

if __name__ == '__main__' :
    client = ClientSide()
    # main = threading.Thread(target=main_thread)
    # main.start()
    userIn = int(input())
    if userIn == 1:
        stateMonitoring()
    elif userIn == 2:
        main_thread()
    
    