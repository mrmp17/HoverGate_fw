//
// Created by matej on 22/07/2019.
//

#ifndef HOVERGATE_FW_SERIAL_H
#define HOVERGATE_FW_SERIAL_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "stm32f1xx_hal_gpio.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;


#define DMA_BUFF_LEN 32
#define SERIAL_BUFF_LEN 64

extern  "C" {
void LPUART1_IRQHandler(void);
void UART4_IRQHandler(void);
void USART2_IRQHandler(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
}

class Serial {
public:
    Serial(UART_HandleTypeDef *uartHandle, DMA_HandleTypeDef *dmaHandle);
    void begin();
    bool available();
    int8_t read();
    int8_t write(uint8_t singleByte);
    int8_t write(uint8_t buffer[], uint16_t len);
    void ILHandler();
    uint16_t getErrorCount();
    void clearError();  //counter for serial error events (calling write too soon, buffer full, HW error)
    bool txOngoing = false;
private:
    void cpyToBuffer (int16_t);
    UART_HandleTypeDef *uartHandle;
    DMA_HandleTypeDef *dmaHandle;
    uint8_t DMABuffer [DMA_BUFF_LEN];
    uint8_t serialBuffer [SERIAL_BUFF_LEN];
    uint16_t head = 1;
    uint16_t tail = 0;
    uint16_t errorCount = 0;
    bool serialBufferFull = false;
    uint8_t txBuffer [SERIAL_BUFF_LEN];

};

extern Serial serial_01;



#endif //HOVERGATE_FW_SERIAL_H
