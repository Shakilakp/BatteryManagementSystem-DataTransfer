#include "stdio.h"
#include "String.h"
#include "stdlib.h"
#include "stdint.h"

void AddUint8ToArray(uint8_t value, uint8_t SendNumbers[], size_t index)
{
	    SendNumbers[index] = value;
}

void AddInt16ToArray(int16_t value, uint8_t SendNumbers[], size_t index)
{
			// little-endian system
	    SendNumbers[index] = (uint8_t)(value & 0xFF);         // Low byte
			SendNumbers[index + 1] = (uint8_t)((value >> 8) & 0xFF); // High byte
}

void AddUint16ToArray(uint16_t value, uint8_t SendNumbers[], size_t index)
{
			// little-endian system
	    SendNumbers[index] = (uint8_t)(value & 0xFF);         // Low byte
			SendNumbers[index + 1] = (uint8_t)((value >> 8) & 0xFF); // High byte
}