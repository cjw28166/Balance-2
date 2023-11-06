#include "crc32.h"



uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
		uint32_t xbit=0,bits;
		uint32_t data = 0;
		uint32_t CRC32 = 0xFFFFFFFF;
   // const 
		uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        
        data=ptr[i];bits=0;
        for(bits=0;bits<32;bits++)
        {
						xbit = 1<<(31-bits);
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;
        }
    }

    return CRC32;
}
