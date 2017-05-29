all: demowallerBase

demowallerBase: demowallerBase.c robotBase.c v4l2write.c logging.c server_udp.c sounds.c
	g++ -pthread -lm -O2 -Wall demowallerBase.c robotBase.c logging.c v4l2write.c server_udp.c sounds.c pixels.c -o demowallerBase

demowallerCore: demowallerCore.c robotBase.c v4l2write.c logging.c server_udp.c sounds.c
	g++ -pthread -lm -O2 -Wall demowallerCore.c robotBase.c logging.c v4l2write.c server_udp.c sounds.c pixels.c -o demowallerCore

demowallerR-WIS: demowallerR-WIS.c robotBase.c v4l2write.c logging.c server_udp.c
	g++ -pthread -lm -O2 -Wall demowallerR-WIS.c robotBase.c logging.c v4l2write.c server_udp.c -o demowallerR-WIS

clean:
	rm -f demowallerBase