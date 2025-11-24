import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

BAUD = 115200
PORTAS = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0']
ser = None

# ------------------- TENTA ABRIR PORTA -------------------
for porta in PORTAS:
    try:
        ser = serial.Serial(porta, BAUD, timeout=1, dsrdtr=False, rtscts=False)
        print(f"✅ Conectado em {porta}")
        time.sleep(2)  # tempo para terminar boot
        break
    except Exception as e:
        print(f"⚠️ Não consegui abrir {porta}: {e}")

if ser is None:
    print("❌ Nenhuma porta serial disponível.")
    exit(1)

# ------------------- BUFFERS -------------------
mag_x, mag_y, mag_z = [], [], []
acc_x, acc_y, acc_z = [], [], []
gyro_x, gyro_y, gyro_z = [], [], []

WINDOW = 200  # últimos 200 pontos

# ------------------- PLOT -------------------
fig, ax = plt.subplots(3, 1, figsize=(10, 10))

lines = [
    ax[0].plot([], [], label="MAG X")[0],
    ax[0].plot([], [], label="MAG Y")[0],
    ax[0].plot([], [], label="MAG Z")[0],

    ax[1].plot([], [], label="ACC X")[0],
    ax[1].plot([], [], label="ACC Y")[0],
    ax[1].plot([], [], label="ACC Z")[0],

    ax[2].plot([], [], label="GYRO X")[0],
    ax[2].plot([], [], label="GYRO Y")[0],
    ax[2].plot([], [], label="GYRO Z")[0],
]

ax[0].set_title("Magnetômetro")
ax[1].set_title("Acelerômetro")
ax[2].set_title("Giroscópio")

for a in ax:
    a.legend()
    a.set_xlim(0, WINDOW)
    a.set_ylim(-200, 200)
    a.grid(True)

# ---------------------- FILTRO DE LIXO ----------------------
def is_boot_message(line):
    BOOT = [
        "rst:", "load:", "clk_drv", "mode:DIO",
        "entry", "ets Jul", "boot:", "configsip"
    ]
    return any(tag in line for tag in BOOT)


# ------------------- UPDATE LOOP -------------------
def update(frame):
    line = ser.readline().decode(errors='ignore').strip()

    if not line or is_boot_message(line):
        return lines

    try:
        # Formato esperado:
        # MAG,10,20,30; ACC,1,2,3; GYRO,0.1,0.2,0.3
        parts = line.split(";")

        if len(parts) < 3:
            raise Exception("Linha incompleta")

        # MAG
        mx, my, mz = map(float, parts[0].split(",")[1:])
        # ACC
        axx, axy, axz = map(float, parts[1].split(",")[1:])
        # GYRO
        gx, gy, gz = map(float, parts[2].split(",")[1:])

    except Exception:
        print("Linha inválida:", line)
        return lines

    # Adiciona ao buffer
    mag_x.append(mx);  mag_y.append(my);  mag_z.append(mz)
    acc_x.append(axx); acc_y.append(axy); acc_z.append(axz)
    gyro_x.append(gx); gyro_y.append(gy); gyro_z.append(gz)

    # Mantém janela fixa
    bufs = [mag_x, mag_y, mag_z, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
    for buf in bufs:
        buf[:] = buf[-WINDOW:]

    # Atualiza curvas
    for i, buf in enumerate(bufs):
        lines[i].set_data(range(len(buf)), buf)

    return lines


ani = animation.FuncAnimation(fig, update, interval=50)
plt.tight_layout()
plt.show()
