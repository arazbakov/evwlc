#include <string.h>
#include "memory.h"
#include "stdio.h"
#include "stdbool.h"

extern char serialNumber[27];
extern CRC_HandleTypeDef hcrc;

uint8_t defaultMacAddress[] = {0x02, 0xA0, 0xD9, 0, 0, 0};

uint8_t defaultIpAddress[] = {192, 168, 1, 42};

uint8_t defaultSubnetMask[] = {255, 255, 255, 0};

uint8_t defaultGateway[] = {192, 168, 1, 1};

uint8_t defaultMqttBrokerAddress[] = {0, 0, 0, 0};
uint16_t defaultMqttBrokerPort = 1883;

char defaultMqttTopicPrefix[] = "homeAutomation/light";

char halStatuses[4][8] = {
        "OK",
        "ERROR",
        "BUSY",
        "TIMEOUT"
};

uint8_t currentPageNumber = 60 - 1;

uint32_t globalWritesNumber = 0;

void initDefaultMacAddress() {
    uint32_t buffer = HAL_GetUIDw0();
    uint32_t crc = HAL_CRC_Calculate(&hcrc, &buffer, 1);
    defaultMacAddress[3] = (uint8_t)crc;

    buffer = HAL_GetUIDw1();
    crc = HAL_CRC_Calculate(&hcrc, &buffer, 1);
    defaultMacAddress[4] = crc;

    buffer = HAL_GetUIDw2();
    crc = HAL_CRC_Calculate(&hcrc, &buffer, 1);
    defaultMacAddress[5] = crc;
}

void fillInDefaultConfig(Config_t *config) {
    config->version = CONFIG_VERSION;

    if (defaultMacAddress[3] == 0 && defaultMacAddress[4] == 0 && defaultMacAddress[5] == 0) {
        initDefaultMacAddress();
    }

    memcpy(config->macAddress, defaultMacAddress, sizeof(defaultMacAddress));
    memcpy(config->ipAddress, defaultIpAddress, sizeof(defaultIpAddress));
    memcpy(config->subnetMask, defaultSubnetMask, sizeof(defaultSubnetMask));
    memcpy(config->gateway, defaultGateway, sizeof(defaultGateway));
    memcpy(config->mqttBrokerAddress, defaultMqttBrokerAddress, sizeof(defaultMqttBrokerAddress));

    config->mqttBrokerPort = defaultMqttBrokerPort;

    strcpy(config->mqttClientId , serialNumber);
    memset(config->mqttLogin, '\0', sizeof (config->mqttLogin));
    memset(config->mqttPassword, '\0', sizeof (config->mqttPassword));
    sprintf(config->mqttTopic, "%s/%s", defaultMqttTopicPrefix, serialNumber);
}

void readMemoryConfig(uint32_t configVersion, Config_t *config) {
    uint32_t address = FLASH_BASE + FLASH_PAGE_SIZE * currentPageNumber;

    uint32_t *magicNumberPtr = (uint32_t *) address;
    uint32_t magicNumber = *magicNumberPtr;

    address += sizeof(uint32_t);

    uint32_t *writesNumberPtr = (uint32_t *) address;
    uint32_t writesNumber = *writesNumberPtr;
    address += sizeof(uint32_t);

    if (magicNumber == MAGIC_NUMBER) {
        // the stored data is valid
        globalWritesNumber = writesNumber;
        memcpy(config, (const void *) address, sizeof(Config_t));
        if (config->version != configVersion) {
            fillInDefaultConfig(config);
            writeMemoryConfig(config);
        }
    } else {
        fillInDefaultConfig(config);
        writeMemoryConfig(config);
    }
}

bool writeMemoryConfig(Config_t *config) {
    FLASH_EraseInitTypeDef eraseInitStruct;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.Page = currentPageNumber;
    eraseInitStruct.NbPages = 1;

    HAL_StatusTypeDef status;

    uint32_t pageError = 0;
    status = HAL_FLASH_Unlock();
#ifdef DEBUG_MODE
    printf("Flash unlock: %s\r\n", halStatuses[status]);
#endif
    if (status != HAL_OK) {
        return false;
    }
    status = HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
#ifdef DEBUG_MODE
    printf("Flash erase: %s\r\n", halStatuses[status]);
#endif
    if (status != HAL_OK) {
        return false;
    }

    uint32_t address = FLASH_BASE + FLASH_PAGE_SIZE  * currentPageNumber;
    uint64_t header = ((uint64_t) (++globalWritesNumber) << 32) | (MAGIC_NUMBER);

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, header);

#ifdef DEBUG_MODE
    printf("Writing config header: %s\r\n", halStatuses[status]);
#endif

    if (status != HAL_OK) {
        return false;
    }

    address += sizeof(header);

    uint8_t dwordSizeOfConfig = sizeof(Config_t) / 8;

    if (sizeof(Config_t) % 8 > 0) {
        dwordSizeOfConfig++;
    }

    uint64_t *dataPtr = (uint64_t *) config;
    for (uint8_t i = 0; i < dwordSizeOfConfig; ++i, ++dataPtr, address += 8) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, *dataPtr);
#ifdef DEBUG_MODE
        printf("Block %d of %d: %s\r\n", i, dwordSizeOfConfig, halStatuses[status]);
#endif
        if (status != HAL_OK) {
            return false;
        }
    }

    status = HAL_FLASH_Lock();

#ifdef DEBUG_MODE
    printf("Flash lock: %s\r\n", halStatuses[status]);
#endif
    return true;
}