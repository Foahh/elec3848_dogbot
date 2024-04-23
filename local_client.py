import socket, threading, time

class ClientSide:
    def __init__(self) -> None:
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_socket.settimeout(3)
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
    
    def __recv(self, msg) -> None:
        try:
            
            if "echoback" in msg:
                data = self.client_socket.recv(128).decode()
                print(data, flush=True)
        except TimeoutError as e:
            print(e)
        return
    
    def listening(self, msg) -> None:
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
        
    def regular_sending(self) -> None:
        msg = "echoback\n"
        while True:
            self.client_socket.sendall(msg.encode())
            # print("regular msg sent.")
            time.sleep(0.2)

    def regular_receiving(self) -> None:
        while True:
            try:
                data = self.client_socket.recv(128).decode()
                print(data, end='', flush=True)
            except TimeoutError as e:
                print(e, flush=True)
        return


def main_thread() -> None:
    while True:
        try:
            userIn = input(">> ", )
            if userIn == 'q':
                client.shutdown()
                exit(0)
            elif userIn == "crusing":
                client.client_socket.sendall(userIn.encode())
            elif userIn == "approaching":
                client.client_socket.sendall(userIn.encode())
            else:
                client.client_socket.sendall(userIn.encode())
        except KeyboardInterrupt:
            print(flush=True)
        except Exception as e:
            print(e)

def stateMonitoring() -> None:
    sending = threading.Thread(target=client.regular_sending)
    receiving = threading.Thread(target=client.regular_receiving)
    try:
        sending.start()
        receiving.start()
    except KeyboardInterrupt:
        sending.join()
        receiving.join()
        return

if __name__ == '__main__' :
    client = ClientSide()
    # main = threading.Thread(target=main_thread)
    # main.start()
    userIn = int(input())
    if userIn == 1:
        stateMonitoring()
    elif userIn == 2:
        main_thread()
    
    