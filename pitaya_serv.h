#pragma once 

#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/ip.h>
#include <liquid/liquid.h>
#include <math.h>
#include <pthread.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <time.h>

#define PORT_1 11361
#define CHAN_NUMBER 1
#define FFT_POINTS 1024

#define MAX_PAK_LEN 1280+16

#define FFT_PAK_LEN 1040
#define G711_PAK_LEN 1042
#define IQ_PAK_LEN 1044

#define FFT_HEAD_LEN 16
#define G711_HEAD_LEN 18
#define IQ_HEAD_LEN 20



#define FREQ 4
#define SRATE 5
#define ARATE 6
#define RFG 7
#define AFG 8
#define DMOD 9
#define SSEL 10
#define CHAN_SEL 11
#define MAX_IN_BUF 256

#define DAY_SECS 86400










