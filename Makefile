

PLAY:

	gcc pitaya_serv.c -O3 -march=armv7-a -mtune=cortex-a9 -mfpu=neon -mfloat-abi=hard -lpthread -lm  -lliquid -Wall -o serv


clean:
	rm -f *.o serv
