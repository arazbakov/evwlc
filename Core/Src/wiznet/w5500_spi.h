/*
 * w5500_spi.h
 */

#ifndef SRC_WIZNET_W5500_SPI_H_
#define SRC_WIZNET_W5500_SPI_H_

#include "../memory/memory.h"

void W5500Init(Config_t *config);
void printNetworkInfo();
uint8_t isEthernetLink();

#endif /* SRC_WIZNET_W5500_SPI_H_ */
