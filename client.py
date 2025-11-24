import socket
import time

ESP_IP = "192.168.4.1"
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

print("Aguardando ESP32 criar o AP...")
time.sleep(4)

while True:
    try:
        sock.connect((ESP_IP, PORT))
        break
    except:
        print("Tentando conectar...")
        time.sleep(1)

print("Conectado!")

while True:
    sock.sendall(b"GET\n")
    resp = sock.recv(256)
    print("ESP32:", resp.decode())
    time.sleep(1)
