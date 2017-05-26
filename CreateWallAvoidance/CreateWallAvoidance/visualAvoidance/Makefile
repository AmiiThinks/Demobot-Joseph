all: demowaller2 demowallerD1

demowaller2: demowaller2.c baseline-bumper2.c v4l2write.c logging.c server_udp.c
	g++ -pthread -lm -O2 -Wall -march=armv6 -mfpu=vfp -mfloat-abi=hard demowaller2.c baseline-bumper2.c logging.c v4l2write.c server_udp.c -o demowaller2

demowallerD1: demowallerD1.c baseline-bumper3.c v4l2write.c logging.c server_udp.c
	g++ -pthread -lm -O2 -Wall demowallerD1.c baseline-bumper3.c logging.c v4l2write.c server_udp.c -o demowallerD1

demowallerDAC: demowallerDAC.c baseline-bumper3.c v4l2write.c logging.c server_udp.c
	g++ -pthread -lm -O2 -Wall demowallerDAC.c baseline-bumper3.c logging.c v4l2write.c server_udp.c -o demowallerDAC

#demowallerR-WIS: demowallerR-WIS.c baseline-bumper3.c v4l2write.c logging.c server_udp.c
#	g++ -pthread -lm -O2 -Wall demowallerR-WIS.c baseline-bumper3.c logging.c v4l2write.c server_udp.c -o demowallerR-WIS

clean:
	rm -f demowaller2 demowallerD1