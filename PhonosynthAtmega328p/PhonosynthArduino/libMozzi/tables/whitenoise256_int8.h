#ifndef WHITENOISE256_H_
#define WHITENOISE256_H_

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <avr/pgmspace.h>

#define WHITENOISE256_NUM_CELLS 256
#define WHITENOISE256_SAMPLERATE 256

/** @ingroup tables
White noise table.
*/

const char __attribute__((section(".progmem.data"))) WHITENOISE256_DATA []  =
        {
                22, -14, 108, 2, -80, -99, 62, -62,
                -11, -127, -102, -80, -114, -66, -2, 68, -63, -101, 0, -13, 37, 55, 104, 61,
                125, 101, -54, 60, -91, -102, -127, 17, 26, -23, -97, 80, 43, -77, 117, -71, 66,
                -59, 30, -86, 111, 112, 65, -49, 85, -95,38, 27, -39, 80, 94, 44, 20, 116, 36,
                -74, -106, -57, 57, 96, -37, 39, -72, 21, -80, 121, -127, 88, -84, 54, 120, -6,
                -94, 41, -41, -22, 39, 113, 84, 85, -31, -49, 79, -13, -43, -25, 7, -27, -2,
                -91, 85, 102, 82, 37, -28, 115,-82, -79, -33, -38, 80, 47, 68, 32, 36, -29, 30,
                47, -36, -90, -2, -37, 46, -35, -49, -64, 81, 35, -87, 121, 121, -110, -44, -33,
                9, 83, 70, 48, 10, 43, 85, 40, 48, 88, 122, -83, -22, -38, 78, 113, -91, 110,
                -1, 114, 72, 2, 49, -92, 80, -30, 124, -15, 125, -116, 87, -76, 115, 84, -112,
                107, -101, 109, -66, 27, -99, 59, -56, 91, 89, -18, -113, -57, 27, -110, -107,
                -87, -14, -31, -54, -10, 1, 90, 91, 28, -9, -122, 31, -32, -128, 98, 25, 110,
                -41, -97, 19, 17, 42, 5, 68, 123, 66, 22, -77, -43, -125, 86, 92, -4, -15, 33,
                -79, -38, -128, 113, -102, -64, 94, -12, 14, 118, 106, -105, -86, -76, 27, -77,
                -16, 46, -100, 22, 118, -31, -2, 36, 74, 85, 63, -81, 93, 114, 37, -29, -92,
                104, 115, -89	
				        };

#endif /* WHITENOISE256_H_ */
