//
// Created by matej on 22/07/2019.
//

//TODO: check circular buffer logic for head, tail, bufferFull (full, empty, overflow...)

#include "Serial.h"

Serial serial_01(&huart1, &hdma_usart1_rx);

Serial::Serial(UART_HandleTypeDef *uartHandle, DMA_HandleTypeDef *dmaHandle) {
    this->uartHandle = uartHandle;
    this->dmaHandle = dmaHandle;
}

void Serial::begin() {
    __HAL_UART_CLEAR_FLAG(uartHandle, UART_FLAG_IDLE);
    __HAL_UART_ENABLE_IT(uartHandle, UART_IT_IDLE); //enable idle line interrupt (LPUART1_IRQHandler)
    __HAL_UART_ENABLE_IT(uartHandle, UART_IT_ERR);  //enable error interrupt (LPUART1_IRQHandler)
    HAL_UART_Receive_DMA(uartHandle, DMABuffer, DMA_BUFF_LEN);   //Start receiving data in background with DMA


}

bool Serial::available() {
    return (head != tail || serialBufferFull);
}

int8_t Serial::read() {
    if(tail == head && !serialBufferFull) {
        return -1;
    }
    else {
        int8_t out = serialBuffer[tail];
        if(tail < SERIAL_BUFF_LEN-1){
            tail++;
        }
        else{
            tail = 0;
        }
        serialBufferFull = false;
        return out;
    }
}

int8_t Serial::write(uint8_t singleByte) {    //use carefully. DMA transfer must finish before successive call
    if(txOngoing){
        errorCount++;
        return -1;  //tx failed, prev transfer ongoing
    }

    txBuffer[0] = singleByte;
    HAL_UART_Transmit_DMA(uartHandle, txBuffer, 1);
    txOngoing = true;
    return 0;   //ok, tx started
}

/*
 *
 */
int8_t Serial::write(uint8_t *buffer, uint16_t len) {

    if(txOngoing){
        errorCount++;
        return -1;
    }
    if(len > SERIAL_BUFF_LEN){
        errorCount++;
        len = SERIAL_BUFF_LEN;
    }
    for(uint16_t n = 0 ; n<len ; n++){
        txBuffer[n] = buffer[n];
    }
    HAL_StatusTypeDef ok;
    ok = HAL_UART_Transmit_DMA(uartHandle, txBuffer, len);

    txOngoing = true;
    if(ok == HAL_OK){
        return 0;
    }
    else{
        return -1;
    }
}

uint16_t Serial::getErrorCount() {
    return errorCount;
}

void Serial::clearError() {
    errorCount = 0;
}

void Serial::cpyToBuffer (int16_t len){
    for(uint16_t n = 0 ; n<len ; n++){
        if(head == tail && serialBufferFull) {
            errorCount++;
            break;
        }
        serialBuffer[head] = DMABuffer[n];
        if(head < SERIAL_BUFF_LEN-1){
            head++;
        }
        else{
            head = 0;
        }
        if(head == tail){
            serialBufferFull = true;
        }
    }
}

void Serial::ILHandler (){  //interrupt handler. place in UARTx_IRQHandler(). handles Idle Line and Error interrupts
//    int test = __HAL_UART_GET_IT(uartHandle, UART_IT_ERR);
//    test = __HAL_UART_GET_IT(uartHandle, UART_IT_TC);
//    test = __HAL_UART_GET_IT(uartHandle, UART_IT_IDLE);
//    test = __HAL_UART_GET_IT(uartHandle, UART_IT_RXNE);
//    test = __HAL_UART_GET_IT(uartHandle, UART_IT_PE);
//
//
//
//    test = __HAL_UART_GET_FLAG(uartHandle, UART_FLAG_FE);
//    test = __HAL_UART_GET_FLAG(uartHandle, UART_FLAG_NE);
//    test = __HAL_UART_GET_FLAG(uartHandle, UART_FLAG_ORE);
//    test = __HAL_UART_GET_FLAG(uartHandle, UART_FLAG_PE);

    if(__HAL_UART_GET_IT_SOURCE(uartHandle, UART_IT_ERR) || __HAL_UART_GET_FLAG(uartHandle, UART_FLAG_NE) || __HAL_UART_GET_FLAG(uartHandle, UART_FLAG_FE) || __HAL_UART_GET_FLAG(uartHandle, UART_FLAG_ORE)){ //error
        errorCount++;
        __HAL_UART_CLEAR_FLAG(uartHandle, UART_FLAG_FE);
        __HAL_UART_CLEAR_FLAG(uartHandle, UART_FLAG_NE);
        __HAL_UART_CLEAR_FLAG(uartHandle, UART_FLAG_ORE);
        __HAL_UART_CLEAR_FLAG(uartHandle, UART_FLAG_PE);    //clear error just in case TODO: check if needed (parity error interrupt not enabled)

        //__HAL_UART_CLEAR_IT(uartHandle, UART_IT_ERR);   //clear error interrupt //doesnt work on STM32F1 - prob not needed


        __HAL_UART_CLEAR_FLAG(uartHandle, UART_FLAG_IDLE); //clear flag to prevent multiple interrupts
        uint16_t lenToCpy = DMA_BUFF_LEN - __HAL_DMA_GET_COUNTER(dmaHandle);
        HAL_UART_AbortReceive(uartHandle);  //halt end restart DMA transfer after error
        cpyToBuffer(lenToCpy);
        HAL_UART_Receive_DMA(uartHandle, DMABuffer, DMA_BUFF_LEN); //reinitiate DMA transfer


    }

    //TODO: change this to else if
    if(__HAL_UART_GET_FLAG(uartHandle, UART_FLAG_IDLE)){   //idle line detected (end of transmission)
        __HAL_UART_CLEAR_FLAG(uartHandle, UART_FLAG_IDLE); //clear flag to prevent multiple interrupts
        uint16_t lenToCpy = DMA_BUFF_LEN - __HAL_DMA_GET_COUNTER(dmaHandle);
        HAL_UART_AbortReceive(uartHandle);  //halt DMA transfer
        cpyToBuffer(lenToCpy);
        HAL_UART_Receive_DMA(uartHandle, DMABuffer, DMA_BUFF_LEN); //reinitiate DMA transfer
    }
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
//
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  serial_01.ILHandler();
//
  /* USER CODE END USART1_IRQn 1 */
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){    //end of TX DMA transfer callback
    if(huart == &huart1){
        serial_01.txOngoing = false;
    }
    //...else if for other uarts
}
