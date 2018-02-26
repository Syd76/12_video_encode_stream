









#include "Crc_Encode.h"

#include <ctime>
#include <cstring>
#include <cstdlib>
#include <iostream>


double Crc_Encode::getUnixTime(void) {
    struct timespec tv;

    if (clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}


//Initialise CRC Rec and creates CRC Table based on the polynomial.

Crc* Crc_Encode::InitCrc(unsigned int CrcPolynomial)
{
    unsigned short int i;
    unsigned short int j;
    unsigned int tempcrc;
    Crc *phCrc;
    phCrc = (Crc*) malloc (sizeof(Crc));
    if (phCrc == NULL)
    {
        std::cerr << "Mem allocation failed for Init CRC" << std::endl;
        return NULL;
    }

    memset (phCrc, 0, sizeof(Crc));

    for (i = 0; i <= 255; i++)
    {
        tempcrc = i;
        for (j = 8; j > 0; j--)
        {
            if (tempcrc & 1)
            {
                tempcrc = (tempcrc >> 1) ^ CrcPolynomial;
            }
            else
            {
                tempcrc >>= 1;
            }
        }
        phCrc->CRCTable[i] = tempcrc;
    }

    phCrc->CrcValue = 0;
    return phCrc;
}

//Calculates CRC of data provided in by buffer.
void Crc_Encode::CalculateCrc(Crc *phCrc, unsigned char *buffer, uint32_t count)
{
    unsigned char *p;
    unsigned int temp1;
    unsigned int temp2;
    unsigned int crc = phCrc->CrcValue;
    unsigned int *CRCTable = phCrc->CRCTable;

    if(!count)
        return;

    p = (unsigned char *) buffer;
    while (count-- != 0)
    {
        temp1 = (crc >> 8) & 0x00FFFFFFL;
        temp2 = CRCTable[((unsigned int) crc ^ *p++) & 0xFF];
        crc = temp1 ^ temp2;
    }

    phCrc->CrcValue = crc;
}

//Closes CRC related handles.
void Crc_Encode::CloseCrc(Crc **phCrc)
{
    if (*phCrc)
        free (*phCrc);
}


