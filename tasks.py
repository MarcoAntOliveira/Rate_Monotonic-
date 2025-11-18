import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time
import re

# ================================
# CONFIGURAÇÃO DA SERIAL
# ================================
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# ================================
# FILAS DE DADOS PARA GRÁFICOS
# ================================
dist_data = deque(maxlen=100)     # VL53
heading_data = deque(maxlen=100)  # Magnetômetro

# ================================
# VARIÁVEIS PARA STATUS DAS TASKS
# ================================
task_status = {
    "TASK_MAG": "Aguardando...",
    "TASK_VL53": "Aguardando..."
}

last_update = {
    "TASK_MAG": time.time(),
    "TASK_VL53": time.time()
}

# ================================
# THREAD PARA LER SERIAL
# ================================
def serial_thread():
    global task_status, last_update

    regex_dist = re.compile(r"(\d+)\s*mm")
    regex_heading = re.compile(r"Heading:\s*([\d.]+)")

    while True:
        try:
            line = ser.readline().decode().strip()

            # Identificar task
            if line.startswith("[TASK_MAG]"):
                task_status["TASK_MAG"] = "Rodando"
                last_update["TASK_MAG"] = time.time()

                # Extrair heading
                match = regex_heading.search(line)
                if match:
                    heading_data.append(float(match.group(1)))

            elif line.startswith("[TASK_VL53]"):
                task_status["TASK_VL53"] = "Rodando"
                last_update["TASK_VL53"] = time.time()

                # Extrair distância
                match = regex_dist.search(line)
                if match:
                    dist_data.append(int(match.group(1)))

            # Verificar timeout (task parou)
            for t in task_status:
                if time.time() - last_update[t] > 2:
                    task_status[t] = "⚠ Sem dados"

        except:
            pass


# ================================
# GRÁFICO EM TEMPO REAL
# ================================
fig, ax = plt.subplots()
plt.style.use("ggplot")

def animate(i):
    ax.clear()
    ax.set_title("Leituras em Tempo Real")

    if dist_data:
        ax.plot(dist_data, label="VL53L0X (mm)")

    if heading_data:
        ax.plot(heading_data, label="Heading (°)")

    ax.legend()
    ax.set_ylim(0, max(360, max(dist_data, default=100)))

# ================================
# THREAD PARA MOSTRAR STATUS DAS TASKS
# ================================
def status_monitor():
    while True:
        print("\033c", end="")  # limpar terminal
        print("===== STATUS DAS TASKS ESP32 =====")
        for task, status in task_status.items():
            print(f"{task}: {status}")
        print("\nAtualizando... Ctrl+C para sair")
        time.sleep(0.5)

# ================================
# INICIAR THREADS
# ================================
threading.Thread(target=serial_thread, daemon=True).start()
threading.Thread(target=status_monitor, daemon=True).start()

# ================================
# INICIAR GRÁFICO
# ================================
ani = FuncAnimation(fig, animate, interval=200)
plt.show()
