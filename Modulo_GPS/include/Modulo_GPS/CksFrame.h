#define CRC32_POLYNOMIAL 0xEDB88320

unsigned long CRC32Value(int i);
unsigned long CalculateBlockCRC32(unsigned long ulCount,unsigned char *ucBuffer);
