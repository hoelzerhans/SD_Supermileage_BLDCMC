int itf_decodePC(uint8_t* str);
int itf_checkCRC(int message);
int itf_addCRC(int message);
int itf_actOnMessage(int message,int source);
void itf_init_UART0(void);
void itf_init_UART1(void);
uint8_t itf_crc4(uint8_t c, uint64_t x, int bits);
int itf_sendDataMCU(const char* data);
void itf_dirHandler(void *arg);
void itf_initDirPins(void);

extern int itf_dirInput1;
extern int itf_dirInput0;