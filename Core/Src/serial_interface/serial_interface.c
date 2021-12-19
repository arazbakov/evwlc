#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "serial_interface.h"
#include "../memory/memory.h"
#include "main.h"

extern Config_t config;
extern bool outputMqttDataToSerial;
char *serialNumberPtr;

UART_HandleTypeDef *uartPtr;

bool errorHappened = false;

char commandBuf[256];
uint8_t rxBuf[255];
uint8_t rxBufPtr = 0;
bool isRxBufFull = false;
bool isCommandAvailable = false;
bool isRxBufAvailable = true;

typedef struct {
    void (*functionPtr)(char *, uint16_t);

    char *commandName;
    uint8_t commandLength;
    char commandDescription[100];
} serialCommand;


uint8_t
setConfigStringParam(char *configParam, uint16_t maxLength, char *dataPtr, uint16_t dataLength) {
    if (dataLength >= maxLength) {
        return 0;
    }

    memcpy(configParam, dataPtr, dataLength);
    configParam[dataLength] = '\0';
    return 1;
}

uint8_t setConfigUIntParam(uint16_t *configParam, const char *dataPtr, uint16_t dataLength) {
    uint32_t result = 0;
    uint16_t pow = 1;

    // max value is 65535

    if (dataLength > 5) {
        return 0;
    }

    for (uint16_t i = 0; i < dataLength; ++i, pow *= 10) {
        uint8_t digit = dataPtr[dataLength - i - 1];

        if (digit < '0' || digit > '9') {
            return 0;
        }

        result += (digit - '0') * pow;
    }

    if (result > 65535U) {
        return 0;
    }

    *configParam = (uint16_t) result;
    return 1;
}

uint8_t setConfigIpAddressParam(uint8_t **configParam, const char *dataPtr, uint16_t dataLength) {
    uint16_t result = 0;

    uint8_t buffer[4];

    uint16_t position = 0;

    for (uint8_t i = 0; i < (uint8_t) (sizeof buffer); ++i) {

        result = 0;

        // max value is 255
        uint8_t digitsNumber = 0;
        while (1) {
            if (digitsNumber > 3) {
                return 0;
            }

            if (position >= dataLength || dataPtr[position] == '.') {
                if (digitsNumber == 0) {
                    return 0;
                } else {
                    ++position;
                    break;
                }
            }

            uint8_t digit = dataPtr[position];

            if (digit < '0' || digit > '9') {
                return 0;
            }

            digit -= '0';

            result = result * 10 + digit;

            ++position;
            ++digitsNumber;
        }

        if (result > 255U) {
            return 0;
        }

        buffer[i] = result;
    }

    memcpy(configParam, buffer, sizeof buffer);
    return 1;
}

char commandDisplayConfigName[] = "AT+DC";

void commandDisplayConfig(char *dataPtr, uint16_t dataLength) {
    printf("Config Version: %lx\r\n", config.version);
    printf("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", config.macAddress[0],
           config.macAddress[1],
           config.macAddress[2], config.macAddress[3], config.macAddress[4], config.macAddress[5]);
    printf("IP Address: %d.%d.%d.%d\r\n", config.ipAddress[0], config.ipAddress[1],
           config.ipAddress[2], config.ipAddress[3]);
    printf("Subnet Mask: %d.%d.%d.%d\r\n", config.subnetMask[0], config.subnetMask[1],
           config.subnetMask[2], config.subnetMask[3]);
    printf("Gateway: %d.%d.%d.%d\r\n", config.gateway[0], config.gateway[1], config.gateway[2],
           config.gateway[3]);
    printf("MQTT Broker Address: %d.%d.%d.%d\r\n", config.mqttBrokerAddress[0],
           config.mqttBrokerAddress[1], config.mqttBrokerAddress[2],
           config.mqttBrokerAddress[3]);
    printf("MQTT Broker Port: %d\r\n", config.mqttBrokerPort);
    printf("MQTT Client ID: \"%s\"\r\n", config.mqttClientId);
    printf("MQTT Topic: \"%s\"\r\n", config.mqttTopic);
    printf("MQTT Login: \"%s\"\r\n", config.mqttLogin);
    printf("MQTT Password: \"%s\"\r\n", config.mqttPassword);
}

char commandSaveConfigAndRestartName[] = "AT+SR";

void commandSaveConfigAndRestart(char *dataPtr, uint16_t dataLength) {
    printf("Writing config to flash...\r\n\r\n");
    if (writeMemoryConfig(&config)) {
        printf("Config is successfully saved\r\n");
    } else {
        printf("Unable to save config\r\n");
    }
    printf("Restarting...\r\n\r\n");
    HAL_NVIC_SystemReset();
}

char commandPrintSerialNumberName[] = "AT+SN";

void commandPrintSerialNumber(char *dataPtr, uint16_t dataLength) {
    printf("%s\r\n", serialNumberPtr);
}

char commandSetMqttLoginName[] = "AT+CS+mqttLogin=";

void commandSetMqttLogin(char *dataPtr, uint16_t dataLength) {
    if (!setConfigStringParam(config.mqttLogin, sizeof config.mqttLogin, dataPtr, dataLength)) {
        printf("Incorrect value\r\n");
        return;
    }

    printf("MQTT Login is set to \"%s\"\r\n", config.mqttLogin);
}

char commandSetMqttPasswordName[] = "AT+CS+mqttPassword=";

void commandSetMqttPassword(char *dataPtr, uint16_t dataLength) {
    if (!setConfigStringParam(config.mqttPassword, sizeof config.mqttPassword, dataPtr,
                              dataLength)) {
        printf("Incorrect value\r\n");
        return;
    }

    printf("MQTT Password is set to \"%s\"\r\n", config.mqttPassword);
}

char commandSetMqttBrokerPortName[] = "AT+CS+mqttBrokerPort=";

void commandSetMqttBrokerPort(char *dataPtr, uint16_t dataLength) {
    if (!setConfigUIntParam(&(config.mqttBrokerPort), dataPtr, dataLength)) {
        printf("Incorrect value\r\n");
        return;
    }

    printf("MQTT Broker Port is set to \"%d\"\r\n", config.mqttBrokerPort);
}


char commandSetIpAddressName[] = "AT+CS+ipAddress=";

void commandSetIpAddress(char *dataPtr, uint16_t dataLength) {
    if (!setConfigIpAddressParam((uint8_t **) &(config.ipAddress), dataPtr, dataLength)) {
        printf("Incorrect value\r\n");
        return;
    }

    printf("IP Address is set to \"%d.%d.%d.%d\"\r\n", config.ipAddress[0], config.ipAddress[1],
           config.ipAddress[2], config.ipAddress[3]);
}

char commandSetSubnetMaskName[] = "AT+CS+subnetMask=";

void commandSetSubnetMask(char *dataPtr, uint16_t dataLength) {
    if (!setConfigIpAddressParam((uint8_t **) &(config.subnetMask), dataPtr, dataLength)) {
        printf("Incorrect value\r\n");
        return;
    }

    printf("Subnet Mask is set to \"%d.%d.%d.%d\"\r\n", config.subnetMask[0], config.subnetMask[1],
           config.subnetMask[2], config.subnetMask[3]);
}

char commandSetGatewayName[] = "AT+CS+gateway=";

void commandSetGateway(char *dataPtr, uint16_t dataLength) {
    if (!setConfigIpAddressParam((uint8_t **) &(config.gateway), dataPtr, dataLength)) {
        printf("Incorrect value\r\n");
        return;
    }

    printf("Gateway is set to \"%d.%d.%d.%d\"\r\n", config.gateway[0], config.gateway[1],
           config.gateway[2], config.gateway[3]);
}

char commandSetMqttBrokerAddressName[] = "AT+CS+mqttBrokerAddress=";

void commandSetMqttBrokerAddress(char *dataPtr, uint16_t dataLength) {
    if (!setConfigIpAddressParam((uint8_t **) &(config.mqttBrokerAddress), dataPtr, dataLength)) {
        printf("Incorrect value\r\n");
        return;
    }

    printf("MQTT Broker Address is set to \"%d.%d.%d.%d\"\r\n", config.mqttBrokerAddress[0],
           config.mqttBrokerAddress[1], config.mqttBrokerAddress[2], config.mqttBrokerAddress[3]);
}

char commandSetMqttClientIdName[] = "AT+CS+mqttClientId=";

void commandSetMqttClientId(char *dataPtr, uint16_t dataLength) {
    if (!setConfigStringParam(config.mqttClientId, sizeof config.mqttClientId, dataPtr,
                              dataLength)) {
        printf("Incorrect value\r\n");
        return;
    }

    printf("MQTT Client ID is set to \"%s\"\r\n", config.mqttClientId);
}

char commandSetMqttTopicName[] = "AT+CS+mqttTopic=";

void commandSetMqttTopic(char *dataPtr, uint16_t dataLength) {
    if (!setConfigStringParam(config.mqttTopic, sizeof config.mqttTopic, dataPtr,
                              dataLength)) {
        printf("Incorrect value\r\n");
        return;
    }

    printf("MQTT Topic is set to \"%s\"\r\n", config.mqttTopic);
}

char commandResetConfigName[] = "AT+RC";

void commandResetConfig(char *dataPtr, uint16_t dataLength) {
    fillInDefaultConfig(&config);
    printf("Config is reset to default values\r\n");
}

char commandListAllCommandsName[] = "AT";

void commandListAllCommands(char *dataPtr, uint16_t dataLength);

char commandEnableMqttSerialOutputName[] = "AT+MSE";

void commandEnableMqttSerialOutput(char *dataPtr, uint16_t dataLength) {
    outputMqttDataToSerial = true;
    printf("MQTT messages output to serial is enabled\r\n");
}
char commandDisableMqttSerialOutputName[] = "AT+MSD";

void commandDisableMqttSerialOutput(char *dataPtr, uint16_t dataLength) {
    outputMqttDataToSerial = false;
    printf("MQTT messages output to serial is diabled\r\n");
}

serialCommand commandsList[] = {
        {
                .commandName = commandPrintSerialNumberName,
                .commandLength = sizeof commandPrintSerialNumberName,
                .commandDescription = "Print serial number",
                .functionPtr = commandPrintSerialNumber
        },
        {
                .commandName = commandDisplayConfigName,
                .commandLength = sizeof commandDisplayConfigName,
                .commandDescription = "Display all config params",
                .functionPtr = &commandDisplayConfig
        },
        {
                .commandName = commandSaveConfigAndRestartName,
                .commandLength = sizeof commandSaveConfigAndRestartName,
                .commandDescription = "Save config to memory and restart",
                .functionPtr = commandSaveConfigAndRestart
        },
        {
                .commandName = commandSetIpAddressName,
                .commandLength = sizeof commandSetIpAddressName,
                .commandDescription = "Set device IP address. Format: XXX.XXX.XXX.XXX",
                .functionPtr = commandSetIpAddress
        },
        {
                .commandName = commandSetSubnetMaskName,
                .commandLength = sizeof commandSetSubnetMaskName,
                .commandDescription = "Set subnet mask. Format: XXX.XXX.XXX.XXX",
                .functionPtr = commandSetSubnetMask
        },
        {
                .commandName = commandSetGatewayName,
                .commandLength = sizeof commandSetGatewayName,
                .commandDescription = "Set network gateway. Format: XXX.XXX.XXX.XXX",
                .functionPtr = commandSetGateway
        },
        {
                .commandName = commandSetMqttBrokerPortName,
                .commandLength = sizeof commandSetMqttBrokerPortName,
                .commandDescription = "Set MQTT broker port. Value is a 16-bit unsigned number (0..65535)",
                .functionPtr = commandSetMqttBrokerPort
        },
        {
                .commandName = commandSetMqttBrokerAddressName,
                .commandLength = sizeof commandSetMqttBrokerAddressName,
                .commandDescription = "Set MQTT broker IP address. Format: XXX.XXX.XXX.XXX",
                .functionPtr = commandSetMqttBrokerAddress
        },
        {
                .commandName = commandSetMqttLoginName,
                .commandLength = sizeof commandSetMqttLoginName,
                .commandDescription = "Set MQTT login. Value is a string up to 99 bytes",
                .functionPtr = commandSetMqttLogin
        },
        {
                .commandName = commandSetMqttClientIdName,
                .commandLength = sizeof commandSetMqttClientIdName,
                .commandDescription = "Set MQTT client id. Value is a string up to 99 bytes",
                .functionPtr = commandSetMqttClientId
        },
        {
                .commandName = commandSetMqttTopicName,
                .commandLength = sizeof commandSetMqttTopicName,
                .commandDescription = "Set MQTT base topic. Value is a string up to 255 bytes",
                .functionPtr = commandSetMqttTopic
        },
        {
                .commandName = commandSetMqttPasswordName,
                .commandLength = sizeof commandSetMqttPasswordName,
                .commandDescription = "Set MQTT password. Value is a string up to 99 bytes",
                .functionPtr = commandSetMqttPassword
        },
        {
                .commandName = commandResetConfigName,
                .commandLength = sizeof commandResetConfigName,
                .commandDescription = "Reset config to default values",
                .functionPtr = commandResetConfig
        },
        {
                .commandName = commandEnableMqttSerialOutputName,
                .commandLength = sizeof commandEnableMqttSerialOutputName,
                .commandDescription = "Enable output of MQTT messages to serial port",
                .functionPtr = commandEnableMqttSerialOutput
        },
        {
                .commandName = commandDisableMqttSerialOutputName,
                .commandLength = sizeof commandDisableMqttSerialOutputName,
                .commandDescription = "Disable output of MQTT messages to serial port",
                .functionPtr = commandDisableMqttSerialOutput
        },


        {
            .commandName = commandListAllCommandsName,
            .commandLength = sizeof commandListAllCommandsName,
            .commandDescription = "List all AT commands",
            .functionPtr = commandListAllCommands
        }
};

void commandListAllCommands(char *dataPtr, uint16_t dataLength) {
    printf("All AT commands:\r\n");
    for (uint16_t i = 0, l = sizeof (commandsList) / sizeof (serialCommand); i < l; ++i) {
        printf("\"%s\" - %s\r\n", commandsList[i].commandName, commandsList[i].commandDescription);
    }
}

void executeCommand(uint16_t totalLength) {
    uint16_t commandsListLength = sizeof(commandsList) / sizeof(serialCommand);
    for (uint16_t i = 0; i < commandsListLength; ++i) {
        if (strncmp(commandBuf, commandsList[i].commandName, commandsList[i].commandLength - 1) ==
            0) {
            printf("%s\r\n", commandBuf);
            commandsList[i].functionPtr(commandBuf + commandsList[i].commandLength - 1,
                                        totalLength - commandsList[i].commandLength + 1);
            return;
        }
    }
    printf("Invalid command \"%s\"\r\n", commandBuf);
}

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(uartPtr, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    return len;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *ptr) {
    errorHappened = true;
}

void serialPortRoutine() {
    if (errorHappened) {
        printf("UART Error happened :(\r\n");
        errorHappened = false;
    }

    if (isRxBufFull) {
        printf("The command is too long. Max command length is %d bytes\r\n", sizeof rxBuf);
        rxBufPtr = 0;
        isRxBufFull = false;
        isRxBufAvailable = true;
    } else if (isCommandAvailable) {
        memcpy(commandBuf, rxBuf, rxBufPtr);
        commandBuf[rxBufPtr] = '\0';
        uint16_t commandLength = rxBufPtr;
        rxBufPtr = 0;
        isCommandAvailable = false;
        isRxBufAvailable = true;
        executeCommand(commandLength);
    }
}

void initSerialInterface(UART_HandleTypeDef *ptr, char *sn) {
    memset(rxBuf, '\0', sizeof rxBuf);
    uartPtr = ptr;
    serialNumberPtr = sn;

//    HAL_UARTEx_ReceiveToIdle_DMA(uartPtr, rxBuf, sizeof rxBuf);
//    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void handleUartIrq(UART_HandleTypeDef *huart) {
    if (huart->Instance->ISR & UART_IT_RXNE) {
        uint8_t byte = (uint8_t) (huart->Instance->RDR & (uint8_t) 0xFF);
        if (isRxBufAvailable) {
            switch ((char) byte) {
                case '\r':
                    // ignore
                    break;
                case '\n':
                    isCommandAvailable = true;
                    isRxBufAvailable = false;
                    break;
                default:
                    rxBuf[rxBufPtr] = byte;
                    ++rxBufPtr;

                    if (rxBufPtr > sizeof rxBuf) {
                        isRxBufFull = true;
                        isRxBufAvailable = false;
                    }
            }
        }
        __HAL_UART_FLUSH_DRREGISTER(huart);
    }
}
