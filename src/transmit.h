#pragma once

extern "C" void ICACHE_RAM_ATTR transmit(char pin, unsigned char message[], unsigned char repeat, unsigned int intraDelay, unsigned char dcHigh, unsigned char dcLow);
