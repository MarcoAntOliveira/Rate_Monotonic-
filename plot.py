import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Porta do ESP32/Arduino
SERIAL_PORT = "/dev/ttyUSB0"   # ajuste se necessário
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Dados para o gráfico
distances = []

# Figura do Matplotlib
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_ylim(0, 10000)  # ajuste para a faixa do seu sensor
ax.set_xlim(0, 100)   # últimos 100 pontos
ax.set_xlabel("Amostras")
ax.set_ylabel("Distância (mm)")
ax.set_title("Leitura do VL53L0X em tempo real")


def update(frame):
    global distances

    line_received = ser.readline().decode().strip()

    if not line_received:
        return line,

    # Trata TIMEOUT vindo do sensor
    if "Timeout" in line_received:
        print("Timeout recebido")
        return line,

    # Extrai apenas o número (ex: "453 mm")
    try:
        value = int(line_received.split()[0])
        distances.append(value)
    except:
        print("Linha ignorada:", line_received)
        return line,

    # Mantém só os últimos 100 pontos
    distances = distances[-100:]

    # Atualiza gráfico
    line.set_data(range(len(distances)), distances)
    ax.set_xlim(0, len(distances))

    return line,


ani = animation.FuncAnimation(fig, update, interval=100)
plt.tight_layout()
plt.show()
