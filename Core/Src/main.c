/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "wiznet/socket.h"
#include "wiznet/w5500_spi.h"
#include "memory/memory.h"
#include "serial_interface/serial_interface.h"
#include "mqtt_interface.h"
#include "MQTTClient.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HTTP_SOCKET 0
#define MQTT_SOCKET 1

#define RECONNECTION_DELAY 5000
#define UPTIME_SENDING_PERIOD 10000

#define HTTP_SERVER_PORT 80
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

bool outputMqttDataToSerial = true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_ADC1_Init(void);

static void MX_CRC_Init(void);

static void MX_SPI1_Init(void);

static void MX_TIM1_Init(void);

static void MX_TIM3_Init(void);

static void MX_TIM15_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t boardLedState = 0;

char serialNumber[27];

#define CHANNEL_STATES_NUMBER 5

unsigned char mqttSendBuf[512];

unsigned char mqttReceiveBuf[2048];

ChannelState_t channelStates[CHANNEL_STATES_NUMBER];
uint8_t initialActive[CHANNEL_STATES_NUMBER];
uint8_t initialBrightness[CHANNEL_STATES_NUMBER];
uint16_t initialColorTemperature[CHANNEL_STATES_NUMBER];

bool mqttConnected = false;
bool ethernetLink = false;
bool isW5500Inited = false;
bool waitingForEthernetLink = false;
uint16_t messageId = 0;
static char mqttTopicSetAll[128];
static char mqttTopicStateAll[128];
uint8_t mqttTopicSetAllDomainLength = 0;
uint8_t mqttTopicStateAllDomainLength = 0;

uint32_t counter = 0;
volatile uint32_t uptimeSeconds = 0;
volatile uint32_t connectionUptimeSeconds = 0;

uint32_t lastUptimeSendTime = 0;

Config_t config;

AnimationType_t *turnOnAnimation;
AnimationType_t *turnOffAnimation;


Network mqttNetwork;
MQTTClient mqttClient;

typedef enum {
    MF_INIT_START,
    MF_W5500_INITED,
    MF_WAITING_FOR_ETHERNET,
    MF_WAITING_FOR_MQTT_INIT,
    MF_RECONNECTION_TIMEOUT,
    MF_GENERAL_WORKFLOW
} MainFlowStateTypeDef;

static MainFlowStateTypeDef mainFlowState = MF_INIT_START;

uint8_t parseUint8(const char *data, size_t dataLength, uint8_t *result, uint8_t min, uint8_t max) {
    uint16_t buffer = 0;

    if (dataLength > 3) {
        return 0;
    }


    for (size_t i = 0; i < dataLength; ++i) {
        if (data[i] < '0' || data[i] > '9') {
            return 0;
        }

        buffer = buffer * 10 + (data[i] - '0');
    }

    if (buffer < min || buffer > max) {
        return 0;
    }

    *result = (uint8_t) buffer;
    return 1;
}

uint8_t
parseUint16(const char *data, size_t dataLength, uint16_t *result, uint16_t min, uint16_t max) {
    uint32_t buffer = 0;

    if (dataLength > 5) {
        return 0;
    }


    for (size_t i = 0; i < dataLength; ++i) {
        if (data[i] < '0' || data[i] > '9') {
            return 0;
        }

        buffer = buffer * 10 + (data[i] - '0');
    }

    if (buffer < min || buffer > max) {
        return 0;
    }

    *result = (uint16_t) buffer;
    return 1;
}

void publishChannelStateActive(ChannelState_t *channelState) {
    MQTTMessage pubMessage;

    char message[10];
    char topic[150];
    uint8_t rc;

    // active:
    sprintf(topic, "%s/state/%d/active", config.mqttTopic, channelState->id);
    pubMessage.qos = QOS0;
    pubMessage.id = messageId++;
    if (channelState->active) {
        pubMessage.payload = "on";
        pubMessage.payloadlen = 2;
    } else {
        pubMessage.payload = "off";
        pubMessage.payloadlen = 3;
    }
    pubMessage.retained = 1;
    rc = MQTTPublish(&mqttClient, topic, &pubMessage);
    if (outputMqttDataToSerial) {
        if (rc == SUCCESSS) {
            printf("Successfully sent \"%s\" = \"%.*s\"\r\n", topic, pubMessage.payloadlen,
                   (char *) pubMessage.payload);
        } else {
            printf("Unable to send \"%s\" = \"%.*s\"\r\n", topic, pubMessage.payloadlen,
                   (char *) pubMessage.payload);
        }
    }
}

void publishChannelStateBrightness(ChannelState_t *channelState) {
    MQTTMessage pubMessage;

    char message[5];
    char topic[150];
    uint8_t rc;

    sprintf(topic, "%s/state/%d/brightness", config.mqttTopic, channelState->id);
    sprintf(message, "%d", channelState->brightness);
    pubMessage.qos = QOS0;
    pubMessage.id = messageId++;
    pubMessage.payload = message;
    pubMessage.payloadlen = strlen(message);
    pubMessage.retained = 1;
    rc = MQTTPublish(&mqttClient, topic, &pubMessage);
    if (outputMqttDataToSerial) {
        if (rc == SUCCESSS) {
            printf("Successfully sent \"%s\" = \"%.*s\"\r\n", topic, pubMessage.payloadlen,
                   (char *) pubMessage.payload);
        } else {
            printf("Unable to send \"%s\" = \"%.*s\"\r\n", topic, pubMessage.payloadlen,
                   (char *) pubMessage.payload);
        }
    }
}

void publishChannelStateColorTemperature(ChannelState_t *channelState) {
    MQTTMessage pubMessage;

    char message[8];
    char topic[150];
    uint8_t rc;

    sprintf(topic, "%s/state/%d/colorTemperature", config.mqttTopic, channelState->id);
    sprintf(message, "%d", channelState->colorTemperature);
    pubMessage.qos = QOS0;
    pubMessage.id = messageId++;
    pubMessage.payload = message;
    pubMessage.payloadlen = strlen(message);
    pubMessage.retained = 1;
    rc = MQTTPublish(&mqttClient, topic, &pubMessage);
    if (outputMqttDataToSerial) {
        if (rc == SUCCESSS) {
            printf("Successfully sent \"%s\" = \"%.*s\"\r\n", topic, pubMessage.payloadlen,
                   (char *) pubMessage.payload);
        } else {
            printf("Unable to send \"%s\" = \"%.*s\"\r\n", topic, pubMessage.payloadlen,
                   (char *) pubMessage.payload);
        }
    }
}

void publishChannelState(ChannelState_t *channelState) {
    publishChannelStateBrightness(channelState);
    publishChannelStateColorTemperature(channelState);
    publishChannelStateActive(channelState);
}

uint16_t mapUnsigned(uint16_t x, uint16_t inMin, uint16_t inMax, uint16_t outMin, uint16_t outMax) {
    int32_t result = ((int32_t) x - (int32_t) inMin) * ((int32_t) outMax - (int32_t) outMin);
    result = result / ((int32_t) inMax - (int32_t) inMin) + outMin;

    return (uint16_t) result;
}

void setChannelState(ChannelState_t *channelState, uint8_t active, uint8_t brightness,
                     uint16_t colorTemperature) {

    bool changed = channelState->active != active
                   || channelState->brightness != brightness
                   || channelState->colorTemperature != colorTemperature;

    bool wasTurnedOff = !channelState->active;

    channelState->active = active;
    channelState->brightness = brightness;
    channelState->colorTemperature = colorTemperature;

    if (!changed || (wasTurnedOff && !active)) {
        return;
    }

    channelState->isAnimationInProgress = false;

    uint16_t warmStartValue = channelState->warmPwmValue;
    uint16_t coldStartValue = channelState->coldPwmValue;

    uint16_t maxCoefficient = (MAX_COLOR_TEMPERATURE_VALUE + MIN_COLOR_TEMPERATURE_VALUE) / 2 - MIN_COLOR_TEMPERATURE_VALUE;

    uint16_t warmCoefficient = channelState->colorTemperature - MIN_COLOR_TEMPERATURE_VALUE > maxCoefficient
                               ? MAX_COLOR_TEMPERATURE_VALUE - channelState->colorTemperature
                               : maxCoefficient;

    uint16_t coldCoefficient = channelState->colorTemperature - MIN_COLOR_TEMPERATURE_VALUE < maxCoefficient
                               ? channelState->colorTemperature - MIN_COLOR_TEMPERATURE_VALUE
                               : maxCoefficient;

    uint16_t targetBrightnessPWM = channelState->active ? mapUnsigned(
            channelState->brightness,
            MIN_BRIGHTNESS_VALUE,
            MAX_BRIGHTNESS_VALUE,
            MIN_PWM_VALUE,
            MAX_PWM_VALUE
    ) : 0;

    channelState->targetWarmPwmValue = mapUnsigned(
            warmCoefficient,
            0,
            maxCoefficient,
            MIN_PWM_VALUE,
            targetBrightnessPWM
    );

    channelState->targetColdPwmValue = mapUnsigned(
            coldCoefficient,
            0,
            maxCoefficient,
            MIN_PWM_VALUE,
            targetBrightnessPWM
    );

    int32_t warmPwmValueSpan = (int16_t) channelState->targetWarmPwmValue - warmStartValue;
    int32_t coldPwmValueSpan = (int16_t) channelState->targetColdPwmValue - coldStartValue;

    bool isWarmAnimationReversed = (warmPwmValueSpan < 0);
    bool isColdAnimationReversed = (coldPwmValueSpan < 0);

    uint16_t warmRangeStart = warmStartValue < channelState->targetWarmPwmValue
                              ? warmStartValue
                              : channelState->targetWarmPwmValue;
    uint16_t coldRangeStart = coldStartValue < channelState->targetColdPwmValue
                              ? coldStartValue
                              : channelState->targetColdPwmValue;


    uint16_t warmRangeEnd = warmStartValue > channelState->targetWarmPwmValue
                            ? warmStartValue
                            : channelState->targetWarmPwmValue;
    uint16_t coldRangeEnd = coldStartValue > channelState->targetColdPwmValue
                            ? coldStartValue
                            : channelState->targetColdPwmValue;


    for (uint16_t i = 0; i < ANIMATION_STEPS_NUMBER; ++i) {
        channelState->animationWarmChart[i] = mapUnsigned(
                isWarmAnimationReversed
                ? turnOffAnimation->chart[ANIMATION_STEPS_NUMBER - i - 1]
                : turnOnAnimation->chart[i],

                0,
                255,
                warmRangeStart,
                warmRangeEnd
        );

        channelState->animationColdChart[i] = mapUnsigned(
                isColdAnimationReversed
                ? turnOffAnimation->chart[ANIMATION_STEPS_NUMBER - i - 1]
                : turnOnAnimation->chart[i],

                0,
                255,
                coldRangeStart,
                coldRangeEnd
        );
    }

    channelState->animationStep = 0;
    channelState->isAnimationInProgress = true;
}

void stateChangedHandler(MessageData *messageData) {
    MQTTMessage *message = messageData->message;

    bool isStateTopic = false; // reading from 'state' topic instead of 'set'

    char value[40];

    if (strncmp(messageData->topicName->lenstring.data, mqttTopicStateAll,
                mqttTopicStateAllDomainLength) == 0) {
        isStateTopic = true;
    }

    if (isStateTopic) {
        memcpy(value,
               messageData->topicName->lenstring.data + mqttTopicStateAllDomainLength,
               messageData->topicName->lenstring.len - mqttTopicStateAllDomainLength);

        value[messageData->topicName->lenstring.len - mqttTopicStateAllDomainLength] = '\0';
    } else {
        memcpy(value,
               messageData->topicName->lenstring.data + mqttTopicSetAllDomainLength,
               messageData->topicName->lenstring.len - mqttTopicSetAllDomainLength);

        value[messageData->topicName->lenstring.len - mqttTopicSetAllDomainLength] = '\0';
    }

    if (!isStateTopic && messageData->message->retained) {
        // ignore retained "set" messages
        return;
    }

    if (value[0] < '0' || value[0] > (CHANNEL_STATES_NUMBER - 1) + '0' || value[1] != '/') {
#ifdef DEBUG_MODE
        if (outputMqttDataToSerial) {
            printf("Incorrect state message topic received: \"%s\"\r\n", value);
        }
#endif
        return;
    }

    uint8_t channelNumber = value[0] - '0';
    ChannelState_t *channelState = &channelStates[channelNumber];

    char *valuePtr = value + 2;
    uint8_t buffer;

    if (strcmp("active", valuePtr) == 0) {
        if (message->payloadlen == 2 && (strncasecmp(message->payload, "on", 2) == 0)) {
            if (isStateTopic) {
#ifdef DEBUG_MODE
                if (outputMqttDataToSerial) {
                    printf("Received from STATE topic: %d ON\r\n", channelState->id);
                }
#endif
                initialActive[channelState->id] = 1;
            } else {
#ifdef DEBUG_MODE
                if (outputMqttDataToSerial) {
                    printf("Received from SET topic: %d ON\r\n", channelState->id);
                }
#endif
                setChannelState(channelState, 1, channelState->brightness,
                                channelState->colorTemperature);
                publishChannelStateActive(channelState);
            }
        } else if (message->payloadlen == 3 && (strncasecmp(message->payload, "off", 3) == 0)) {
            if (isStateTopic) {
#ifdef DEBUG_MODE
                if (outputMqttDataToSerial) {
                    printf("Received from STATE topic: %d OFF\r\n", channelState->id);
                }
#endif
                initialActive[channelState->id] = 0;
            } else {
#ifdef DEBUG_MODE
                if (outputMqttDataToSerial) {
                    printf("Received from SET topic: %d OFF\r\n", channelState->id);
                }
#endif
                setChannelState(channelState, 0, channelState->brightness,
                                channelState->colorTemperature);
                publishChannelStateActive(channelState);
            }
        } else if (message->payloadlen == 6 && (strncasecmp(message->payload, "toggle", 6) == 0)) {
            if (isStateTopic) {
#ifdef DEBUG_MODE
                if (outputMqttDataToSerial) {
                    printf("Received from STATE topic: %d TOGGLE\r\n", channelState->id);
                }
#endif
                initialActive[channelState->id] = !channelState->active;
            } else {
#ifdef DEBUG_MODE
                if (outputMqttDataToSerial) {
                    printf("Received from SET topic: %d TOGGLE\r\n", channelState->id);
                }
#endif
                setChannelState(channelState, !channelState->active, channelState->brightness,
                                channelState->colorTemperature);
                publishChannelStateActive(channelState);
            }
        } else {
#ifdef DEBUG_MODE
            if (outputMqttDataToSerial) {
                printf("Incorrect active state value: \"%.*s\"\r\n", message->payloadlen,
                       (char *) message->payload);
            }
#endif
        }
    } else if (strcmp("brightness", valuePtr) == 0) {
        if (!parseUint8(message->payload, message->payloadlen, &buffer, 1, 255)) {
#ifdef DEBUG_MODE
            if (outputMqttDataToSerial) {
                printf("Incorrect brightness state value: \"%.*s\"\r\n", message->payloadlen,
                       (char *) message->payload);
            }
#endif
        } else {
            if (isStateTopic) {
#ifdef DEBUG_MODE
                if (outputMqttDataToSerial) {
                    printf("Received from STATE topic: %d brightness=%d\r\n", channelState->id,
                           buffer);
                }
#endif
                initialBrightness[channelState->id] = buffer;
            } else {
#ifdef DEBUG_MODE
                if (outputMqttDataToSerial) {
                    printf("Received from SET topic: %d brightness=%d\r\n", channelState->id,
                           buffer);
                }
#endif
                setChannelState(channelState, channelState->active, buffer,
                                channelState->colorTemperature);
                publishChannelStateBrightness(channelState);
            }
        }
    } else if (strcmp("colorTemperature", valuePtr) == 0) {
        uint16_t bufferColorTemperature = 0;
        if (!parseUint16(message->payload, message->payloadlen, &bufferColorTemperature,
                         MIN_COLOR_TEMPERATURE_VALUE,
                         MAX_COLOR_TEMPERATURE_VALUE)) {
#ifdef DEBUG_MODE
            if (outputMqttDataToSerial) {
                printf("Incorrect colorTemperature state value: \"%.*s\"\r\n", message->payloadlen,
                       (char *) message->payload);
            }
#endif
        } else {
            if (isStateTopic) {
#ifdef DEBUG_MODE
                if (outputMqttDataToSerial) {
                    printf("Received from STATE topic: %d colorTemperature=%d\r\n",
                           channelState->id,
                           bufferColorTemperature);
                }
#endif
                initialColorTemperature[channelState->id] = bufferColorTemperature;
            } else {
#ifdef DEBUG_MODE
                if (outputMqttDataToSerial) {
                    printf("Received from SET topic: %d colorTemperature=%d\r\n",
                           channelState->id,
                           bufferColorTemperature);
                }
#endif
                setChannelState(channelState, channelState->active, channelState->brightness,
                                bufferColorTemperature);
                publishChannelStateColorTemperature(channelState);
            }
        }
    } else {
        if (outputMqttDataToSerial) {
            printf("Undefined state name \"%s\" for channel %d\r\n", valuePtr, channelNumber);
        }
    }
}

bool mqttInit() {
    int result;

    bool functionResult = false;

    sprintf(mqttTopicStateAll, "%s/state/#", config.mqttTopic);
    mqttTopicStateAllDomainLength = strlen(mqttTopicStateAll) - 1;

    sprintf(mqttTopicSetAll, "%s/set/#", config.mqttTopic);
    mqttTopicSetAllDomainLength = strlen(mqttTopicSetAll) - 1;

    NewNetwork(&mqttNetwork, MQTT_SOCKET);
    result = ConnectNetwork(&mqttNetwork, config.mqttBrokerAddress, config.mqttBrokerPort);

#ifdef DEBUG_MODE
    if (outputMqttDataToSerial) {
        printf("ConnectNetwork:%d\r\n", result);
    }
#endif

    if (result == 0) {
        if (outputMqttDataToSerial) {
            printf("Unable to connect to MQTT broker: the broker is not available. Check ip address, port or firewall settings.\r\n");
        }
        return false;
    } else {
        if (outputMqttDataToSerial) {
            printf("Successfully connected to MQTT broker.\r\n");
        }
    }

    MQTTClientInit(&mqttClient, &mqttNetwork, 1000, mqttSendBuf, sizeof mqttSendBuf, mqttReceiveBuf,
                   sizeof mqttReceiveBuf);

    char topic[150];

    MQTTPacket_connectData mqttConnectData = MQTTPacket_connectData_initializer;
    mqttConnectData.willFlag = 1;
    MQTTPacket_willOptions willOptions = MQTTPacket_willOptions_initializer;
    sprintf(topic, "%s/status", config.mqttTopic);
    willOptions.topicName.cstring = topic;
    willOptions.retained = 1;
    willOptions.message.cstring = "offline";
    willOptions.qos = QOS0;
    mqttConnectData.will = willOptions;
    mqttConnectData.MQTTVersion = 4;
    mqttConnectData.clientID.cstring = config.mqttClientId;
    mqttConnectData.username.cstring = config.mqttLogin;
    mqttConnectData.password.cstring = config.mqttPassword;

    mqttConnectData.keepAliveInterval = 60;
    mqttConnectData.cleansession = 1;

    enum returnCode rc;

    rc = MQTTConnect(&mqttClient, &mqttConnectData);
    if (rc == SUCCESSS) {
        if (outputMqttDataToSerial) {
            printf("Successfully passed auth at MQTT broker\r\n");
        }
        connectionUptimeSeconds = 0;
    } else {
        if (outputMqttDataToSerial) {
            printf("Unable to auth at MQTT broker. Check login or password\r\n");
        }
        return false;
    }

    for (uint8_t i = 0; i < CHANNEL_STATES_NUMBER; ++i) {
        initialActive[i] = channelStates[i].active;
        initialBrightness[i] = channelStates[i].brightness;
        initialColorTemperature[i] = channelStates[i].colorTemperature;
    }

    // subscribe to state topic to get initial data (from retained messages)

    rc = MQTTSubscribe(&mqttClient, mqttTopicStateAll, QOS0, stateChangedHandler);
    if (rc == SUCCESSS) {
#ifdef DEBUG_MODE
        if (outputMqttDataToSerial) {
            printf("Subscribed to topic \"%s\"\r\n", mqttTopicStateAll);
        }
#endif
    } else {
        if (outputMqttDataToSerial) {
            printf("Unable to subscribe to topic \"%s\"\r\n", mqttTopicStateAll);
        }
        MQTTDisconnect(&mqttClient);
        return false;
    }

    // unsubscribe from state topic, all retained messages are already received:
    rc = MQTTUnsubscribe(&mqttClient, mqttTopicStateAll);
    if (rc == SUCCESSS) {
#ifdef DEBUG_MODE
        if (outputMqttDataToSerial) {
            printf("Unsubscribed from topic \"%s\"\r\n", mqttTopicStateAll);
        }
#endif
    } else {
        if (outputMqttDataToSerial) {
            printf("Unable to unsubscribe from topic \"%s\"\r\n", mqttTopicStateAll);
        }
        MQTTDisconnect(&mqttClient);
        return false;
    }

    for (uint8_t i = 0; i < CHANNEL_STATES_NUMBER; ++i) {
        setChannelState(&channelStates[i], initialActive[i], initialBrightness[i],
                        initialColorTemperature[i]);
        publishChannelState(&channelStates[i]);
    }

    // subscribe to /set/ topic to receive control operations:
    rc = MQTTSubscribe(&mqttClient, mqttTopicSetAll, QOS0, stateChangedHandler);
    if (rc == SUCCESSS) {
#ifdef DEBUG_MODE
        if (outputMqttDataToSerial) {
            printf("Subscribed to topic \"%s\"\r\n", mqttTopicSetAll);
        }
#endif
    } else {
        if (outputMqttDataToSerial) {
            printf("Unable to subscribe to topic \"%s\"\r\n", mqttTopicSetAll);
        }
        MQTTDisconnect(&mqttClient);
        return false;
    }

    MQTTMessage pubMessage;

    char message[] = "online";

    sprintf(topic, "%s/status", config.mqttTopic);

    pubMessage.qos = QOS0;
    pubMessage.id = messageId++;
    pubMessage.payload = message;
    pubMessage.payloadlen = sizeof message - 1;
    pubMessage.retained = 1;
    rc = MQTTPublish(&mqttClient, topic, &pubMessage);
    if (rc == SUCCESSS) {
#ifdef DEBUG_MODE
        if (outputMqttDataToSerial) {
            printf("Online message is sent to topic \"%s\"\r\n", topic);
        }
#endif
    } else {
#ifdef DEBUG_MODE
        if (outputMqttDataToSerial) {
            printf("Unable to send online message\r\n");
        }
#endif
    }
}

void sendUptime() {
    MQTTMessage pubMessage;
    char topic[120];
    sprintf(topic, "%s/uptimeSeconds", config.mqttTopic);
    char message[12];
    sprintf(message, "%lu", uptimeSeconds);

    pubMessage.qos = QOS0;
    pubMessage.payload = message;
    pubMessage.payloadlen = strlen(message);
    pubMessage.retained = 0;

    if (MQTTPublish(&mqttClient, topic, &pubMessage) == SUCCESSS) {
        if (outputMqttDataToSerial) {
            printf("Message is sent to topic \"%s\"\r\n", topic);
        }
    } else {
        if (outputMqttDataToSerial) {
            printf("Unable to send the uptime message\r\n");
        }
    }

    lastUptimeSendTime = HAL_GetTick();
}

void sendConnectionUptime() {
    MQTTMessage pubMessage;
    char topic[120];
    sprintf(topic, "%s/connectionUptimeSeconds", config.mqttTopic);
    char message[12];
    sprintf(message, "%lu", connectionUptimeSeconds);

    pubMessage.qos = QOS0;
    pubMessage.payload = message;
    pubMessage.payloadlen = strlen(message);
    pubMessage.retained = 0;

    if (MQTTPublish(&mqttClient, topic, &pubMessage) == SUCCESSS) {
        if (outputMqttDataToSerial) {
            printf("Message is sent to topic \"%s\"\r\n", topic);
        }
    } else {
        if (outputMqttDataToSerial) {
            printf("Unable to send the connectionUptime message\r\n");
        }
    }

    lastUptimeSendTime = HAL_GetTick();
}

static uint16_t timerSecondCounter = 0;
static uint8_t timerAnimationFrameCounter = 0;


void timerTick() {
    // timer ticks each 1ms

    ++timerSecondCounter;
    ++timerAnimationFrameCounter;

    if (timerSecondCounter >= 1000) {
        timerSecondCounter = 0;
        ++uptimeSeconds;
        ++connectionUptimeSeconds;
    }

    if (timerAnimationFrameCounter >= 3) {
        timerAnimationFrameCounter = 0;

        for (uint8_t i = 0; i < CHANNEL_STATES_NUMBER; ++i) {
            ChannelState_t *channelState = &(channelStates[i]);
            if (!channelState->isAnimationInProgress) {
                continue;
            }

            if (channelState->animationStep >= ANIMATION_STEPS_NUMBER) {
                channelState->isAnimationInProgress = false;
                channelState->animationStep = 0;

                channelState->warmPwmValue = channelState->targetWarmPwmValue;
                channelState->coldPwmValue = channelState->targetColdPwmValue;
                *channelState->warmTimerCCR = channelState->warmPwmValue;
                *channelState->coldTimerCCR = channelState->coldPwmValue;
                continue;
            }

            channelState->warmPwmValue = channelState->animationWarmChart[channelState->animationStep];
            channelState->coldPwmValue = channelState->animationColdChart[channelState->animationStep];

            *channelState->warmTimerCCR = channelState->warmPwmValue;
            *channelState->coldTimerCCR = channelState->coldPwmValue;

            ++channelState->animationStep;
        }
    }
}

void initChannelStates() {
    channelStates[0].warmTimerHandle = &htim3;
    channelStates[0].warmTimerChannel = TIM_CHANNEL_4;
    channelStates[0].warmTimerCCR = &(htim3.Instance->CCR4);
    channelStates[0].coldTimerHandle = &htim3;
    channelStates[0].coldTimerChannel = TIM_CHANNEL_3;
    channelStates[0].coldTimerCCR = &(htim3.Instance->CCR3);

    channelStates[1].warmTimerHandle = &htim1;
    channelStates[1].warmTimerChannel = TIM_CHANNEL_4;
    channelStates[1].warmTimerCCR = &(htim1.Instance->CCR4);
    channelStates[1].coldTimerHandle = &htim1;
    channelStates[1].coldTimerChannel = TIM_CHANNEL_3;
    channelStates[1].coldTimerCCR = &(htim1.Instance->CCR3);

    channelStates[2].warmTimerHandle = &htim3;
    channelStates[2].warmTimerChannel = TIM_CHANNEL_2;
    channelStates[2].warmTimerCCR = &(htim3.Instance->CCR2);
    channelStates[2].coldTimerHandle = &htim3;
    channelStates[2].coldTimerChannel = TIM_CHANNEL_1;
    channelStates[2].coldTimerCCR = &(htim3.Instance->CCR1);

    channelStates[3].warmTimerHandle = &htim1;
    channelStates[3].warmTimerChannel = TIM_CHANNEL_2;
    channelStates[3].warmTimerCCR = &(htim1.Instance->CCR2);
    channelStates[3].coldTimerHandle = &htim1;
    channelStates[3].coldTimerChannel = TIM_CHANNEL_1;
    channelStates[3].coldTimerCCR = &(htim1.Instance->CCR1);

    channelStates[4].warmTimerHandle = &htim15;
    channelStates[4].warmTimerChannel = TIM_CHANNEL_2;
    channelStates[4].warmTimerCCR = &(htim15.Instance->CCR2);
    channelStates[4].coldTimerHandle = &htim15;
    channelStates[4].coldTimerChannel = TIM_CHANNEL_1;
    channelStates[4].coldTimerCCR = &(htim15.Instance->CCR1);

    for (uint32_t i = 0; i < CHANNEL_STATES_NUMBER; ++i) {
        ChannelState_t *channelState = &(channelStates[i]);
        channelState->id = i;
        channelState->isAnimationInProgress = false;
        memset(channelState->animationColdChart, 0, ANIMATION_STEPS_NUMBER);
        memset(channelState->animationWarmChart, 0, ANIMATION_STEPS_NUMBER);
        channelState->animationStep = 0;
        channelState->coldPwmValue = 0;
        channelState->warmPwmValue = 0;
        channelState->targetColdPwmValue = 0;
        channelState->targetWarmPwmValue = 0;

        channelStates[i].active = 0;
        channelStates[i].brightness = (MAX_BRIGHTNESS_VALUE + MIN_BRIGHTNESS_VALUE) / 2;
        channelStates[i].colorTemperature =
                (MAX_COLOR_TEMPERATURE_VALUE + MIN_COLOR_TEMPERATURE_VALUE) / 2;

        HAL_TIM_PWM_Start(channelStates[i].warmTimerHandle, channelStates[i].warmTimerChannel);
        HAL_TIM_PWM_Start(channelStates[i].coldTimerHandle, channelStates[i].coldTimerChannel);
    }
}

uint32_t reconnectionStartTime = 0;
uint32_t reconnectionTimeout = 3000;


/**
 * Changes mainFlowState to MF_RECONNECTION_TIMEOUT
 */
void reconnect() {
    reconnectionStartTime = HAL_GetTick();
    mainFlowState = MF_RECONNECTION_TIMEOUT;
    if (outputMqttDataToSerial) {
        printf("Reconnection timeout %lus\r\n", reconnectionTimeout / 1000);
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_CRC_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM15_Init();
    MX_USART2_UART_Init();
    MX_TIM6_Init();
    /* USER CODE BEGIN 2 */
    // read serial number:


    turnOnAnimation = &animations[1];
    turnOffAnimation = &animations[3];

    sprintf(serialNumber, "%08lx-%08lx-%08lx", HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
    initSerialInterface(&huart2, serialNumber);
    printf("\r\n\r\n");

    printf("Serial Number: \"%s\"\r\n", serialNumber);
    printf("Size of config: %d\r\n", sizeof config);

    readMemoryConfig(CONFIG_VERSION, &config);

    printf("Initializing W5500...\r\n");
    W5500Init(&config);
    printNetworkInfo();
    mainFlowState = MF_W5500_INITED;

    HAL_TIM_Base_Start_IT(&htim6);

    initChannelStates();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        serialPortRoutine();

        switch (mainFlowState) {
            case MF_INIT_START:
                if (outputMqttDataToSerial) {
                    printf("Initializing W5500...\r\n");
                }
                W5500Init(&config);
                printNetworkInfo();
                mainFlowState = MF_W5500_INITED;
                break;
            case MF_W5500_INITED:
                if (outputMqttDataToSerial) {
                    printf("W5500 is initialized. Waiting for ethernet link...\r\n");
                }
                mainFlowState = MF_WAITING_FOR_ETHERNET;
                break;
            case MF_WAITING_FOR_ETHERNET:
                if (isEthernetLink()) {
                    if (outputMqttDataToSerial) {
                        printf("Ethernet link is established\r\n");
                    }
                    mainFlowState = MF_WAITING_FOR_MQTT_INIT;
                }
                break;
            case MF_WAITING_FOR_MQTT_INIT:
                if (outputMqttDataToSerial) {
                    printf("Trying to connect to MQTT broker...\r\n");
                }
                if (mqttInit()) {
                    mainFlowState = MF_GENERAL_WORKFLOW;
                } else {
                    reconnect();
                }

                break;
            case MF_RECONNECTION_TIMEOUT:
                if (HAL_GetTick() - reconnectionStartTime > reconnectionTimeout) {
                    mainFlowState = MF_INIT_START;
                }
                break;
            case MF_GENERAL_WORKFLOW:
                if (MQTTYield(&mqttClient, 1000) != SUCCESSS) {
                    mqttConnected = false;
                    if (outputMqttDataToSerial) {
                        printf("MQTT connection failed\r\n");
                    }
                }

                if (HAL_GetTick() - lastUptimeSendTime > UPTIME_SENDING_PERIOD) {
                    sendConnectionUptime();
                    sendUptime();
                }

                uint8_t socketStatus = getSn_SR(MQTT_SOCKET);

                if (socketStatus != SOCK_ESTABLISHED) {
                    if (outputMqttDataToSerial) {
                        printf("Socket status has changed: %x\r\n", socketStatus);
                        printf("MQTT broker has disconnected\r\n");
                    }
                    reconnect();
                }

                if (!isEthernetLink()) {
                    if (outputMqttDataToSerial) {
                        printf("Ethernet disconnected\r\n");
                    }
                    reconnect();
                }
                break;

            default:
                break;
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void) {

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.LowPowerAutoPowerOff = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
    hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
    hadc1.Init.OversamplingMode = DISABLE;
    hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    /** Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void) {

    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance = CRC;
    hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    if (HAL_CRC_Init(&hcrc) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void) {

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void) {

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 3;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 500;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void) {

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 3;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 500;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void) {

    /* USER CODE BEGIN TIM6_Init 0 */

    /* USER CODE END TIM6_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM6_Init 1 */

    /* USER CODE END TIM6_Init 1 */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 64000 - 1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void) {

    /* USER CODE BEGIN TIM15_Init 0 */

    /* USER CODE END TIM15_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM15_Init 1 */

    /* USER CODE END TIM15_Init 1 */
    htim15.Instance = TIM15;
    htim15.Init.Prescaler = 3;
    htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim15.Init.Period = 500;
    htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim15.Init.RepetitionCounter = 0;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim15) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM15_Init 2 */

    /* USER CODE END TIM15_Init 2 */
    HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit =
            UART_ADVFEATURE_RXOVERRUNDISABLE_INIT | UART_ADVFEATURE_DMADISABLEONERROR_INIT;
    huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
    huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */
    huart2.Instance->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
    NVIC_SetPriority(USART2_IRQn, 2); // set priority level
    NVIC_EnableIRQ(USART2_IRQn);
    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, W5500_RESET_Pin | BOARD_LED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB,
                      W5500_CS_Pin | BrokerStatus_Pin | EthernetStatus_Pin | EthernetStatusN_Pin
                      | CH4Status_Pin | CH3Status_Pin | CH2Status_Pin | CH1Status_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BrokerStatusN_GPIO_Port, BrokerStatusN_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(CH0Status_GPIO_Port, CH0Status_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : W5500_RESET_Pin */
    GPIO_InitStruct.Pin = W5500_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(W5500_RESET_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : BOARD_LED_Pin */
    GPIO_InitStruct.Pin = BOARD_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : W5500_CS_Pin */
    GPIO_InitStruct.Pin = W5500_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(W5500_CS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : CH4Button_Pin CH3Button_Pin CH2Button_Pin CH1Button_Pin
                             CH0Button_Pin */
    GPIO_InitStruct.Pin = CH4Button_Pin | CH3Button_Pin | CH2Button_Pin | CH1Button_Pin
                          | CH0Button_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : BrokerStatusN_Pin */
    GPIO_InitStruct.Pin = BrokerStatusN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BrokerStatusN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : BrokerStatus_Pin EthernetStatus_Pin EthernetStatusN_Pin CH4Status_Pin
                             CH3Status_Pin CH2Status_Pin CH1Status_Pin */
    GPIO_InitStruct.Pin =
            BrokerStatus_Pin | EthernetStatus_Pin | EthernetStatusN_Pin | CH4Status_Pin
            | CH3Status_Pin | CH2Status_Pin | CH1Status_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : CH0Status_Pin */
    GPIO_InitStruct.Pin = CH0Status_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CH0Status_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

