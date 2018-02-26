





#ifndef _CRC_ENCODE_H_
#define _CRC_ENCODE_H_


#define CRC32_POLYNOMIAL  0xEDB88320L

#include <cstdint>

typedef struct CrcRec
{
    unsigned int CRCTable[256];
    unsigned int CrcValue;
} Crc;


class Crc_Encode {
    
public :
    Crc_Encode();
    virtual ~Crc_Encode();
    
    static double getUnixTime(void);
    static Crc* InitCrc(unsigned int CrcPolynomial);
    static void CalculateCrc(Crc *phCrc, unsigned char *buffer, uint32_t count);
    static void CloseCrc(Crc **phCrc);
    
protected :    
    
    
};



#endif  // _CRC_ENCODE_H_
