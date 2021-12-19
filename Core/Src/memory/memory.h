#ifndef HELLOWORLD_MEMORY_H
#define HELLOWORLD_MEMORY_H

#include <stdbool.h>
#include "main.h"

// don't update this constant:
#define MAGIC_NUMBER 0x5E6241FBU

// don't forget to update config version on changing of the config's structure:
#define CONFIG_VERSION 0x42211208U // 1 - device code, 2 - year, 3 - month, 4 - version number

// must be multiple of 32 bytes:
typedef struct {
    uint32_t version;

    uint8_t macAddress[6];
    uint8_t ipAddress[4];
    uint8_t subnetMask[4];
    uint8_t gateway[4];

    uint8_t mqttBrokerAddress[4];
    uint16_t mqttBrokerPort;

    char mqttTopic[256];

    char mqttClientId[100];
    char mqttLogin[100];
    char mqttPassword[100];
} Config_t;

void readMemoryConfig(uint32_t configVersion, Config_t *config);
bool writeMemoryConfig(Config_t *config);
void fillInDefaultConfig(Config_t *config);

#endif //HELLOWORLD_MEMORY_H
