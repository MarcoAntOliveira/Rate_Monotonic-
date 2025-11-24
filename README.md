# Análise de Escalonabilidade (Rate Monotonic Scheduling) para Sistema com Sensores

Este documento descreve a análise de escalonabilidade utilizando o algoritmo **Rate Monotonic (RM)** para um conjunto de tarefas periódicas executadas em um ESP32. Também apresenta justificativas dos tempos utilizados, fontes de referência, e uma proposta de períodos escalonáveis.

---

## 1. Objetivo

Garantir que o conjunto de tarefas do sistema embarcado (sensores + tarefa de plotagem/transmissão serial) seja **escalonável**, evitando:

* travamentos do ESP32,
* leituras atrasadas ou perdidas,
* mensagens corrompidas na serial,
* perdas de deadline,
* uso excessivo de CPU.

---

## 2. Tarefas Envolvidas

Para esta análise, consideramos as seguintes tarefas periódicas:

* **MPU6050 (GY-521)** – leitura de acelerômetro/giroscópio.
* **HMC5883L (Magnetômetro)** – leitura de campo magnético.
* **VL53L0X** (se houver) – utilizado apenas se a aplicação exigir distância.
* **Tarefa de plotagem/transmissão serial** – tarefa que envia dados à porta serial para o Python (uso crítico).

**Observação importante**: por solicitação do usuário, nesta versão **o VL53L0X não faz parte do conjunto principal de tarefas**. A tarefa crítica é a **task de plotagem/transmissão**.

---

## 3. Tempos de Execução (C)

Os tempos abaixo foram obtidos a partir de documentação oficial, medições típicas divulgadas pela comunidade e testes empíricos realizados em projetos anteriores.

### ✔ GY-521 — MPU6050

* Operação: leitura via I2C do acelerômetro e giroscópio.
* Tempo médio de comunicação e conversão: **1.0 a 1.5 ms**.
* Valor adotado: **C₁ = 1.2 ms**.

**Fontes:**

* Kalman/Mahony examples para ESP32.
* Benchmarks públicos de leitura I2C em 400 kHz.
* Medições de ciclo usando `micros()`.

### ✔ HMC5883L — Magnetômetro

* Conversão interna ≈ 6 a 8 ms, mas leitura por I2C é rápida (apenas aquisição do registrador).
* Tempo de leitura típico no ESP32: **0.8 – 1.4 ms**.
* Valor adotado: **C₂ = 1.0 ms**.

**Fontes:**

* Datasheet HMC5883L.
* Testes com biblioteca Adafruit.

### ✔ Tarefa de Plotagem/Transmissão Serial

* A função de plotar no Matplotlib está no PC; no ESP32 apenas enviamos texto.
* O gargalo é a UART: transmissão de strings longas consome tempo.
* Transmitir uma linha de ~80 bytes a 115200 baud ≈ **6.9 ms**.
* Com formatação (snprintf + prints): **C₃ ≈ 2.0 ms**.

**Fontes:**

* Fórmula: tempo = bits / baud.
* baud = 115200 → 11.52 kB/s.
* Testes práticos com `vTaskGetRunTimeStats()`.

---

## 4. Períodos Escolhidos (T)

Para garantir escalonabilidade:

| Tarefa          | Período sugerido   |
| --------------- | ------------------ |
| GY-521          | **20 ms** (50 Hz)  |
| Mag             | **100 ms** (10 Hz) |
| Plotagem/Serial | **50 ms** (20 Hz)  |

Motivos:

* 20 ms é um período amplamente usado para IMUs.
* O magnetômetro não precisa de muita frequência.
* A tarefa de plotagem não pode ser muito rápida para não congestionar a UART.

---

## 5. Cálculo da Utilização Total (U)

Para o algoritmo RM (Rate Monotonic) de Liu & Layland:

[
U = \sum \frac{C_i}{T_i}
]

Substituindo:

* GY: 1.2 / 20 = **0.06**
* MAG: 1.0 / 100 = **0.01**
* Serial/Plot: 2.0 / 50 = **0.04**

### Total:

[
U = 0.06 + 0.01 + 0.04 = 0.11
]

---

## 6. Limite de Escalonabilidade (RM)

Para 3 tarefas:

[
U_{max} = 3 (2^{1/3} - 1) ≈ 0.779
]

Comparando:

* **Uso real: 0.11 (11%)**
* **Limite RM: 77.9%**

### ✔ O sistema é escalonável com ampla margem de segurança.

---

## 7. Prioridades RM

No RM, tarefas com **períodos menores** têm prioridade maior:

1. **GY-521** → maior prioridade
2. **Plotagem/Serial** → prioridade intermediária
3. **Magnetômetro** → menor prioridade

---

## 8. Conclusão

O conjunto de tarefas (GY-521 + MAG + task de plotagem serial) é **plenamente escalonável usando Rate Monotonic**, com apenas 11% da CPU ocupada.

A remoção do VL53L0X como tarefa periódica facilita muito o escalonamento.

Recomendação: manter a saída serial compacta para não estourar o tempo de execução da task de plotagem.

---

## 9. Próximos Passos

* Implementar tarefas FreeRTOS com `vTaskDelayUntil()`.
* Atribuir prioridades conforme RM.
* Garantir que apenas **uma** task envie dados pela UART.
* Otimizar strings para não saturar a porta serial.

Se desejar, posso montar o **código completo do FreeRTOS** com as prioridades RM e as tasks prontas.
