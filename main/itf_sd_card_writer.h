int itf_writeFileFromBuffer(int bufferNum);
int itf_addToSD(char *toStore,int length);
int itf_addTestToSD(int testNum);
void itf_writeSD_task(void * params);
int itf_forceWriteBuffers(void);
void itf_writeTestMessage(char *str);
extern int itf_forceWriteBuffers_FLAG;
