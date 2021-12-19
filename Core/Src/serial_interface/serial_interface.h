#ifndef HELLOWORLD_SERIAL_INTERFACE_H
#define HELLOWORLD_SERIAL_INTERFACE_H

#include "main.h"

void initSerialInterface(UART_HandleTypeDef *ptr, char *sn);
void serialPortRoutine();
void handleUartIrq(UART_HandleTypeDef *huart);

#endif //HELLOWORLD_SERIAL_INTERFACE_H
