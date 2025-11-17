build:
	@pio run -t upload
	@python3 ../osc.py
moni:
	@python3 ../osc.py

	
 run:
	@python3 plot.py
test:	
	terminator -e "bash -c 'python3 plot.py; exec bash'" &

