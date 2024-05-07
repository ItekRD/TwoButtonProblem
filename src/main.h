#ifndef __MAIN_H
#define __MAIN_H
#include "FunctionDefine.h"
#include "uart_async_adapter.h"

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/gpio.h>

#if deShareFunction
#define deValueReset			0
#define deDisable				0
#define deEnable				1
#endif

#if deUartFunction
#define UART0_DEVICE_MCU DT_NODELABEL(uart0)
#define UART1_DEVICE_MCU DT_NODELABEL(uart1)
#define MSG_SIZE 32
#endif

#if deMCU_URFunc
#define deMCU_UR_On				uart_irq_rx_enable(MCU_uart)
#define deMCU_UR_Off			uart_irq_rx_disable(MCU_uart)

#define deMCU_CommStartNum			0x80	// commend Start Number
#define deMCU_CommEndNum			0x81	// commend End	 Number

#define deBLE_DataMod				0x01
#define deRFID_DataMod				0x02
#define deOB_DataMod				0x03

uint8_t	u8MCUUR_Buffer[255];
uint8_t u8BLE_DataBuf[255];					// Connend is BLE Data get Commend
uint8_t u8RFID_DataBuf[50];					// Connend is RFID Data get Commend
uint8_t u8OB_DataBuf[50];					// Connend is Out Board Action Commend

uint8_t	u8MCUURGDDFlag				= 0;	// MUC Uart Get Data Done Flag
uint8_t	u8MCUURGD_Flag				= 0;	// MCU Uart Get Data Flag
uint8_t	u8MCUURDataLen				= 0;	// MCU Uart Get Data Lenght
uint8_t	u8MCUURCRC					= 0;	// MCU Uart Data End CRC data 

uint8_t u8OB_DataLen				= 0;
uint16_t u16MCUUR_BufNum			= 0;	// MCU Uart Buffer Number
#endif


#if deDebugFunction

#define de_LED1_ON		dk_set_led(DK_LED1, deEnable)
#define de_LED1_OFF		dk_set_led(DK_LED1, deDisable)
#define de_LED2_ON		dk_set_led(DK_LED2, deEnable)
#define de_LED2_OFF		dk_set_led(DK_LED2, deDisable)
#define de_LED3_ON		dk_set_led(DK_LED3, deEnable)
#define de_LED3_OFF		dk_set_led(DK_LED3, deDisable)
#define de_LED4_ON		dk_set_led(DK_LED4, deEnable)
#define de_LED4_OFF		dk_set_led(DK_LED4, deDisable)

uint8_t u8LED1_SwitchFlag			= 0;
uint8_t u8LED2_SwitchFlag			= 0;

#if deUR_CheckValue1
uint8_t	u8Debug1ValueBuf[] = {'!',' ','!','\n'};
uint8_t	u8Debug2ValueBuf[] = {'@',' ','@','\n'};
#endif
#endif

#if deButtonFunction
#define SW0_NODE	DT_ALIAS(sw0)
#define SW1_NODE	DT_ALIAS(sw1)
#define SW2_NODE	DT_ALIAS(sw2)
#define SW3_NODE	DT_ALIAS(sw3)
#endif
#if deChangePinMod
uint8_t	u8RxPinChangeLock			= 0;
uint8_t u8RxButChange				= 0; // Rx Button Change			
#endif
#endif
