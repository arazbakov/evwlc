#include <string.h>
#include "w5500_spi.h"
#include "wizchip_conf.h"
#include "stm32g0xx_hal.h"
#include "main.h"
#include "stdio.h"

extern SPI_HandleTypeDef hspi1;

uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};

wiz_NetInfo gWIZNETINFO = {
        .mac = {0x02, 0xA0, 0xD9, 0x00, 0x00, 0x00},
        .ip = {192, 168, 1, 8},
        .sn = {255, 255, 255, 0},
        .gw = {192, 168, 1, 1},
        .dns = {0, 0, 0, 0},
        .dhcp = NETINFO_STATIC
};


void wizchip_select() {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

void wizchip_deselect() {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t *buff, uint16_t len) {
    HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
}

void W5500_WriteBuff(uint8_t *pBuf, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, pBuf, len, HAL_MAX_DELAY);
}

uint8_t wizchip_read() {
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}

void wizchip_write(uint8_t wb) {
    W5500_WriteBuff(&wb, sizeof(wb));
}


//uint8_t DhcpProcess() {
//    uint8_t dhcp_res = DHCP_run();
//    switch (dhcp_res) {
//        case DHCP_IP_ASSIGN:
//        case DHCP_IP_CHANGED:
//        case DHCP_IP_LEASED:
//            getIPfromDHCP(gWIZNETINFO.ip);
//            getGWfromDHCP(gWIZNETINFO.gw);
//            getSNfromDHCP(gWIZNETINFO.sn);
//            getDNSfromDHCP(gWIZNETINFO.dns);
//            gWIZNETINFO.dhcp = NETINFO_DHCP;
//            ctlnetwork(CN_SET_NETINFO, (void *) &gWIZNETINFO);
//#ifdef _DHCP_DEBUG_
//            //printf("\r\n>> DHCP IP Leased Time : %ld Sec\r\n", getDHCPLeasetime());
//#endif
//            break;
//        case DHCP_FAILED:
//#ifdef _DHCP_DEBUG_
//            printf(">> DHCP Failed\r\n");
//#endif
//            gWIZNETINFO.dhcp = NETINFO_STATIC;
//            break;
//    }
//    return dhcp_res;
//}

void W5500Init(Config_t *config) {


    memcpy(gWIZNETINFO.mac, config->macAddress, sizeof gWIZNETINFO.mac);
    memcpy(gWIZNETINFO.ip, config->ipAddress, sizeof gWIZNETINFO.ip);
    memcpy(gWIZNETINFO.sn, config->subnetMask, sizeof gWIZNETINFO.sn);
    memcpy(gWIZNETINFO.gw, config->gateway, sizeof gWIZNETINFO.gw);

    wizchip_deselect();

    // reset pulse
    HAL_GPIO_WritePin(W5500_RESET_GPIO_Port, W5500_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(W5500_RESET_GPIO_Port, W5500_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

    int8_t result = 0;

    printf("before wizchip_init.\r\n");
    result = wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
    printf("wizchip_init: %d.\r\n", result);

//    uint8_t temp;
//    //check phy status
//    do
//    {
//        if (ctlwizchip(CW_GET_PHYLINK, (void*)&temp) == -1)
//        {
//            printf("Unknown PHY link status.\r\n");
//        }
//    } while (temp == PHY_LINK_OFF);

    wizchip_setnetinfo(&gWIZNETINFO);
//    setSHAR(gWIZNETINFO.mac);

    ctlnetwork(CN_SET_NETINFO, (void *) &gWIZNETINFO);
//
//    if (gWIZNETINFO.dhcp == NETINFO_DHCP) {
//        DHCP_init(dhcpSocket, tmpBuffer);
//        uint8_t dhcp_ret = DHCP_STOPPED;
//
//        while (!((dhcp_ret == DHCP_IP_ASSIGN) || (dhcp_ret == DHCP_IP_CHANGED) ||
//                 (dhcp_ret == DHCP_FAILED) || (dhcp_ret == DHCP_IP_LEASED))) {
//            dhcp_ret = DhcpProcess();
//        }
//    }

//	wizchip_getnetinfo(&gWIZNETINFO);
}

uint8_t isEthernetLink() {
    uint8_t temp;
    //check phy status
    ctlwizchip(CW_GET_PHYLINK, (void*)&temp);

    if (temp == PHY_LINK_ON) {
        return 1;
    }

    return 0;
}

void printNetworkInfo() {
    printf("Mac address: %02x:%02x:%02x:%02x:%02x:%02x\n\r", gWIZNETINFO.mac[0], gWIZNETINFO.mac[1],
           gWIZNETINFO.mac[2], gWIZNETINFO.mac[3], gWIZNETINFO.mac[4], gWIZNETINFO.mac[5]);
    if (gWIZNETINFO.dhcp == NETINFO_DHCP)
        printf("DHCP\n\r");
    else
        printf("Static IP\n\r");
    printf("IP Address     : %d.%d.%d.%d\n\r", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2],
           gWIZNETINFO.ip[3]);
    printf("Subnet Mask    : %d.%d.%d.%d\n\r", gWIZNETINFO.sn[0], gWIZNETINFO.sn[1], gWIZNETINFO.sn[2],
           gWIZNETINFO.sn[3]);
    printf("Gateway        : %d.%d.%d.%d\n\r", gWIZNETINFO.gw[0], gWIZNETINFO.gw[1], gWIZNETINFO.gw[2],
           gWIZNETINFO.gw[3]);
    printf("DNS Server     : %d.%d.%d.%d\n\r", gWIZNETINFO.dns[0], gWIZNETINFO.dns[1],
           gWIZNETINFO.dns[2], gWIZNETINFO.dns[3]);
}
