import socket
import time

ESP32_IP = "192.168.4.1"
ESP32_PORT = 5005

def conectar():
    print("[PY] Conectando ao ESP32...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)

    try:
        s.connect((ESP32_IP, ESP32_PORT))
        print("[PY] Conectado!")
        return s
    except Exception as e:
        print("[PY] Erro ao conectar:", e)
        return None

def enviar_get(sock):
    try:
        sock.sendall(b"GET\n")
        print("[PY] GET enviado!")

        resposta = sock.recv(1024).decode().strip()
        print("[PY] Resposta:", resposta)
        return resposta

    except Exception as e:
        print("[PY] Erro ao enviar:", e)
        return None


if __name__ == "__main__":
    sock = conectar()
    if not sock:
        exit()

    while True:
        enviar_get(sock)
        time.sleep(1)   # envia GET a cada 1s
