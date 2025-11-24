


# -----------------------------
# Makefile para ESP32 + Plot + Cliente
# -----------------------------

# Caminho para o Python
PYTHON=python3

# Porta padrão do PlatformIO
UPLOAD_CMD=pio run -t upload

# -----------------------------
# Targets
# -----------------------------

build:
	$(UPLOAD_CMD)

plot:
	$(PYTHON) plot2.py

client:
	$(PYTHON) client.py

# -----------------------------
# RUN (abre 3 terminais usando XTerminator)
# -----------------------------
run:
	xterminator -x bash -c "echo 🔄 Upload ESP32...; pio run -t upload; echo ✔️ Upload finalizado; exec bash"
	sleep 2
	xterminator -x bash -c "echo 📈 Iniciando plot2.py...; python3 plot2.py; exec bash"
	sleep 2
	xterminator -x bash -c "echo 🌐 Iniciando client.py...; python3 client.py; exec bash"

moni:
	@python3 ../osc.py

tasks:
	@pio run -t upload
	@python3 plot2.py


 run:
	@python3 plot2.py
	@python client.py
test:	
	terminator -e "bash -c 'python3 plot.py; exec bash'" &


apply:
	@pio run -t upload
	@python3 plot2.py


