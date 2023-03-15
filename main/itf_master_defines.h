#ifndef ITF_MASTER_DEFINES_H_
#define ITF_MASTER_DEFINES_H_

//Communication function defines
#define ITF_COM_DEFINES 1

#define ITF_TX_PIN_UART1 47
#define ITF_RX_PIN_UART1 48//CHANGE SOON
#define ITF_DIR0_PIN 19
#define ITF_DIR1_PIN 20
#define ITF_RX_BUF_SIZE_UART0 1024
#define ITF_TX_BUF_SIZE_UART0 1024
#define ITF_RX_BUF_SIZE_UART1 1024
#define ITF_TX_BUF_SIZE_UART1 1024

//SD card setup defines
#define ITF_SD_DEFINES 1

#define ITF_MOUNT_POINT_DEF "/sdcard"
#define ITF_SD_WRITE_SIZE 1024 //Power of 2 Pref (1024, 512, ect)
#define ITF_SD_MISO_PIN  37
#define ITF_SD_MOSI_PIN  35
#define ITF_SD_CLK_PIN   36
#define ITF_SD_CS_PIN    38
#define ITF_SD_BUFFER_SIZE ITF_SD_WRITE_SIZE*2

//SD card wrting defines
#define ITF_HEX_DEFINES 1

#define ITF_A_PIN 9 //WAS 7
#define ITF_B_PIN 13 //WAS 15
#define ITF_C_PIN 14 //WAS 16
#define ITF_D_PIN 17
#define ITF_E_PIN 18
#define ITF_F_PIN 8
#define ITF_G_PIN 3

#endif
