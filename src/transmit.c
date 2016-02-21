#include <Arduino.h>

#define CPU_LATENCY 1
#define MAX_MESSAGE_LENGTH 600


// Transmit a command.
// pin: The pin to transmit to.
// message[[]: The 2-byte timings to send.
// repeat: How many times to repeat the message.
// intraDelay: How long to wait between repeats.
// dcHigh: How long the high duty cycle is (in microseconds).
//         Anything over 24 means 100% duty cycle. Will have a constant
//         added to it due to processing latency.
// dcLow: How long the low duty cycle is (in microseconds).
void ICACHE_RAM_ATTR transmit(char pin, unsigned char message[], unsigned char repeat, unsigned int intraDelay, unsigned char dcHigh, unsigned char dcLow) {
    int status = LOW;
    unsigned int out = 0;
    unsigned int i, j;
    unsigned int k;
    unsigned int dcTotal = dcHigh + dcLow;
    bool carrier = true;

    // Compensate for CPU latency.

    if (dcHigh < CPU_LATENCY) {
        dcHigh = 0;
    } else {
        dcHigh = dcHigh - CPU_LATENCY;
    }

    if (dcLow < CPU_LATENCY) {
        dcLow = 0;
    } else {
        dcLow = dcLow - CPU_LATENCY;
    }

    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    if (dcLow == 0) {
        // If the low time is 0, there's no carrier frequency.
        carrier = false;
    }

    // Send the message.
    for (j = 0; j < repeat; j++) {
        for (i = 0; i < MAX_MESSAGE_LENGTH; i = i + 2) {
            if (message[i] == 0 || message[i+1] == 0) {
                break;
            }

            // Convert two bytes to an int.
            out = (message[i] << 8) + message[i+1];
            if (status == HIGH) {
                status = LOW;
                digitalWrite(pin, status);
                delayMicroseconds(out);
            } else {
                status = HIGH;
                if (carrier) {
                    // Loop, once per pulse.
                    for (k = 0; k < out / dcTotal; k++) {
                        digitalWrite(pin, HIGH);
                        delayMicroseconds(dcHigh);
                        digitalWrite(pin, LOW);
                        delayMicroseconds(dcLow);
                    }
                } else {
                    digitalWrite(pin, status);
                    delayMicroseconds(out);
                }
            }
        }

        // Reset, just in case.
        digitalWrite(pin, LOW);
        delayMicroseconds(intraDelay);
        yield();
    }
}
