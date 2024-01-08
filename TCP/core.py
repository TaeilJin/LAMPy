import socket
import json
import struct

class TCP_Connection():
    """
    TCP Connection 
    """
    def __init__(self, server_ip, server_port):
        server_ip = '143.248.6.198'  # 모든 네트워크 인터페이스에서 연결 수락
        server_port = 80

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((server_ip, server_port))
        server_socket.listen(1)  # 최대 동시 연결 수

        print(f"서버 대기 중... IP: {server_ip}, 포트: {server_port}")

        self.data = b''
        self.data_packet =0
        self.client_socket, client_address = server_socket.accept()
        print(f"연결 수락: {client_address[0]}:{client_address[1]}") 
    
    def recieve(self, max_size=4096):
         
        self.data= self.client_socket.recv(max_size)
        if len(self.data) > max_size:
            print('oh my increasing the maximum')
    
    def recieve_Json(self):
        # JSON 데이터를 파싱하여 데이터 추출
        self.data_packet = json.loads(self.data.decode('utf-8'))
        

    def send(self, float_output_array):
        # float 배열을 바이트로 변환
        data_to_send = bytearray(struct.pack(f'{len(float_output_array)}f', *float_output_array))
        self.client_socket.send(data_to_send)

    def exit(self):
        self.client_socket.close()