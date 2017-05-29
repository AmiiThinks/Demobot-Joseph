all: demowallerBase

demowallerBase: demowallerBase.c robotBase.c v4l2write.c logging.c server_udp.c sounds.c
	g++ -pthread -lm -O2 -Wall demowallerBase.c robotBase.c logging.c v4l2write.c server_udp.c sounds.c pixels.c -o demowallerBase

demowallerTest: demowallerTest.c robotBase.c v4l2write.c logging.c server_udp.c sounds.c
	g++ -pthread -lm -O2 -Wall demowallerTest.c robotBase.c logging.c v4l2write.c server_udp.c sounds.c pixels.c -o demowallerTest

demowallerBase-WIS: demowallerBase-WIS.c robotBase.c v4l2write.c logging.c server_udp.c sounds.c
	g++ -pthread -lm -O2 -Wall demowallerBase-WIS.c robotBase.c logging.c v4l2write.c server_udp.c sounds.c pixels.c -o demowallerBase-WIS

clean:
	rm -f demowallerBase
